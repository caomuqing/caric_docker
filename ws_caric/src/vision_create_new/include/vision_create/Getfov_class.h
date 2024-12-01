#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <visualization_msgs/Marker.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <string>
#include <chrono>
#include <random>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include "Pcl_Pro.h"
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
using namespace std::chrono_literals;

class Getfov_class
{
private:
    /* data */
    ros::NodeHandlePtr nh_ptr;
    double fov_horizontal_deg = 60.0; // 水平FOV（度）
    double fov_vertical_deg = 45.0;   // 垂直FOV（度）
    double fov_distance = 5;          //FOV的长度
    visualization_msgs::Marker line_list;
    std::vector<Eigen::Vector3d> viewPoints_save;
    int ik = 0; 
    PclPro* pp =new PclPro();
    ros::Publisher lmarker_pub;

public:
    Getfov_class(ros::NodeHandlePtr &nh_ptr_): nh_ptr(nh_ptr_) {
        lmarker_pub = nh_ptr->advertise<visualization_msgs::Marker>("/ViewPoints_lines", 10);
    }
    ~Getfov_class() {}

    // 运行系统,输入是点云,生成视点并进行TSP处理
    bool run(pcl::PointCloud<pcl::PointXYZ>::Ptr& Lidar_Data) {
        pp->Lidar_Data = Lidar_Data;
        pp->Map_Pro();
        //主处理函数
        pp->Process();
        // std::cout << "=======" << pp->ViewPoints.size() << std::endl;
        //处理完了 发送数据
        if(pp->ViewPoints.size()>3)
        {
            pp->Tsp_Cal();
            return true;
        }
        return false;
    }

    // 获取视点结果(排序后)
    std::vector<Eigen::Vector3d> get_viewpoint() {
        std::vector<Eigen::Vector3d> viewPoints_tem_unsort;
        // std::vector<Eigen::Vector3d> viewPoints_tem_sort;
        viewPoints_save.clear();
        geometry_msgs::Point lasttp;
        std::ifstream inputFile("/home/ztd/ws_caric/src/vision_create_11_20/3rdprty/Tsp_data.tsp");
        std::cout << "reading file" << std::endl;
        if (!inputFile.is_open()) {
            std::cerr << "Failed to open the file. 请先运行run()" << std::endl;
            return viewPoints_save;
        }

        std::string line;
        while (std::getline(inputFile, line)) {
            if (line.find("NODE_COORD_SECTION") != std::string::npos) {
                while (std::getline(inputFile, line)) {
                    if (line.find_first_not_of("0123456789. -") != std::string::npos) {
                        continue;
                    }
                    std::istringstream iss(line);
                    Eigen::Vector3d point;
                    int id;
                    iss >> id >> point[0] >> point[1] >> point[2];
                    viewPoints_tem_unsort.push_back(point);
                }
                break;  // Exit the loop when reaching the NODE_COORD_SECTION
            }
        }
        inputFile.close();
        
        std::cout << "finish reading file!" << viewPoints_tem_unsort.size() << std::endl;
        line_list.points.clear();
        line_list.header.frame_id = "world";
        line_list.pose.orientation.w = 1;
        line_list.header.stamp = ros::Time::now();
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.id = 2;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.scale.x = 0.3;  // 设置线宽
        line_list.color.r = 1.0;
        line_list.color.a = 1.0;

        for (int index : pp->tourInd)
        {
            if (index >= 0 && index <= pp->tourInd.size()) 
            {
                // 使用索引从 ViewPoints 中提取信息
                Eigen::Vector3d viewpoint = viewPoints_tem_unsort[index-1];
                viewPoints_save.push_back(viewpoint);

                geometry_msgs::Point tp;
                tp.x = viewpoint[0];
                tp.y = viewpoint[1];
                tp.z = viewpoint[2];

                // std::cout << "line_strip's points:(" << tp.x << ","<< tp.y << "," << tp.z << "," << "), index:" << index << std::endl; 
                if(!lasttp.x == 0.0 && !lasttp.y == 0.0 && !lasttp.z == 0.0 && lasttp != tp) {
                    line_list.points.push_back(lasttp);
                    line_list.points.push_back(tp);
                }
                lasttp = tp;
            }
        }
        // viewPoints_save.insert(viewPoints_save.end(), viewPoints_tem_sort.begin(), viewPoints_tem_sort.end());
        return viewPoints_save;
    }

    void pub_result() {
        lmarker_pub.publish(line_list);
    }

    void reset() {
        pp->reset();
    }
};
