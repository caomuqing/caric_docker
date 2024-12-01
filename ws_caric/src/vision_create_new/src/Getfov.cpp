#include <ros/ros.h>
#include <thread>
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
#include <mutex>
using namespace std::chrono_literals;
geometry_msgs::PoseArray poseArrayMsg;
visualization_msgs::Marker points, line_strip, line_list;
int ik = 0;
// std::mutex view_mut;

// //显示FOV 待做
// void Show_fov(pcl::visualization::PCLVisualizer::Ptr views)
// {
//     for(int i=0;i<1;i=i+40)
//     {
//         //输入数据
//         float x=0;//fov_interst[i][0];
//         float y=0;//fov_interst[i][1];
//         float z=0;//fov_interst[i][2];
//         float yaw_deg=0;//fov_interst[i][4];
//         float pitch_deg=0;//fov_interst[i][3];
//         float roll_deg=0;//fov_interst[i][5];
//         // 将角度转换为弧度
//         double pitch_rad = pitch_deg * M_PI / 180.0;
//         double yaw_rad = yaw_deg * M_PI / 180.0;
//         double roll_rad = roll_deg * M_PI / 180.0;
//         // 创建一个变换矩阵来表示位姿
//         Eigen::Affine3f transform = Eigen::Affine3f::Identity();
//         transform.translation() << x, y, z; // 设置平移
//         transform.rotate(Eigen::AngleAxisf(yaw_rad, Eigen::Vector3f::UnitY())); // 设置偏航
//         transform.rotate(Eigen::AngleAxisf(pitch_rad, Eigen::Vector3f::UnitX())); // 设置俯仰
//         transform.rotate(Eigen::AngleAxisf(roll_rad, Eigen::Vector3f::UnitZ())); // 设置横滚
//         // 计算FOV的边界点
//         double fov_half_width = std::tan(fov_horizontal_deg / 2.0 * M_PI / 180.0) * fov_distance;
//         double fov_half_height = std::tan(fov_vertical_deg / 2.0 * M_PI / 180.0) * fov_distance;

//         Eigen::Vector3f fov_center(0.0, 0.0, fov_distance); // FOV中心点坐标
//         Eigen::Vector3f fov_top_left(-fov_half_width, fov_half_height, fov_distance);
//         Eigen::Vector3f fov_top_right(fov_half_width, fov_half_height, fov_distance);
//         Eigen::Vector3f fov_bottom_left(-fov_half_width, -fov_half_height, fov_distance);
//         Eigen::Vector3f fov_bottom_right(fov_half_width, -fov_half_height, fov_distance);
//         Eigen::Vector3f fov_base(x,y,z);

//         // 应用位姿变换到FOV边界点
//         fov_center = transform * fov_center;
//         fov_top_left = transform * fov_top_left;
//         fov_top_right = transform * fov_top_right;
//         fov_bottom_left = transform * fov_bottom_left;
//         fov_bottom_right = transform * fov_bottom_right;
        
//         // 绘制FOV的边界线
//         std::string temp_str1="fova";temp_str1+=std::to_string(i);std::string temp_str2="fovb";temp_str2+=std::to_string(i);
//         std::string temp_str3="fovc";temp_str3+=std::to_string(i);std::string temp_str4="fovd";temp_str4+=std::to_string(i);
//         std::string temp_str5="fove";temp_str5+=std::to_string(i);std::string temp_str6="fovf";temp_str6+=std::to_string(i);
//         std::string temp_str7="fovg";temp_str7+=std::to_string(i);std::string temp_str8="fovh";temp_str8+=std::to_string(i);
//         views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_top_left.x(), fov_top_left.y(), fov_top_left.z()), pcl::PointXYZ(fov_top_right.x(), fov_top_right.y(), fov_top_right.z()), 1.0, 0.0, 0.0,temp_str1);
//         views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_top_right.x(), fov_top_right.y(), fov_top_right.z()), pcl::PointXYZ(fov_bottom_right.x(), fov_bottom_right.y(), fov_bottom_right.z()), 1.0, 0.0, 0.0,temp_str2);
//         views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_bottom_right.x(), fov_bottom_right.y(), fov_bottom_right.z()), pcl::PointXYZ(fov_bottom_left.x(), fov_bottom_left.y(), fov_bottom_left.z()), 1.0, 0.0, 0.0,temp_str3);
//         views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_bottom_left.x(), fov_bottom_left.y(), fov_bottom_left.z()), pcl::PointXYZ(fov_top_left.x(), fov_top_left.y(), fov_top_left.z()), 1.0, 0.0, 0.0, temp_str4);

//         views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_base.x(), fov_base.y(), fov_base.z()), pcl::PointXYZ(fov_top_left.x(), fov_top_left.y(), fov_top_left.z()), 1.0, 0.0, 0.0,temp_str5);
//         views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_base.x(), fov_base.y(), fov_base.z()), pcl::PointXYZ(fov_top_right.x(), fov_top_right.y(), fov_top_right.z()), 1.0, 0.0, 0.0,temp_str6);
//         views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_base.x(), fov_base.y(), fov_base.z()), pcl::PointXYZ(fov_bottom_left.x(), fov_bottom_left.y(), fov_bottom_left.z()), 1.0, 0.0, 0.0,temp_str7);
//         views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_base.x(), fov_base.y(), fov_base.z()), pcl::PointXYZ(fov_bottom_right.x(), fov_bottom_right.y(), fov_bottom_right.z()), 1.0, 0.0, 0.0,temp_str8);
//     }
// }
// //检测FOV里有没有点 待做
// void IsPointOffFov(pcl::visualization::PCLVisualizer::Ptr views) 
// {   //y是height x是width
//     Eigen::Affine3f transform = Eigen::Affine3f::Identity();
//     transform.translation() << 0, 0, 0; // 设置平移        
//     transform.rotate(Eigen::AngleAxisf(0 * M_PI / 180.0 ,Eigen::Vector3f::UnitY())); // 设置偏航 y
//     transform.rotate(Eigen::AngleAxisf(0 * M_PI / 180.0 , Eigen::Vector3f::UnitX())); // 设置俯仰 p 
//     transform.rotate(Eigen::AngleAxisf(0 * M_PI / 180.0 , Eigen::Vector3f::UnitZ())); // 设置横滚 r
//     int x=1;
//     int y=0;
//     int z=3;
//     float TR_X = std::tan(fov_horizontal_deg / 2.0 * M_PI / 180.0) * fov_distance;
//     float TR_Y = std::tan(fov_vertical_deg / 2.0 * M_PI / 180.0) * fov_distance;
//     float TR_V = (fov_vertical_deg/2)*M_PI/180;
//     float TR_H = (fov_horizontal_deg/2)*M_PI/180;
//     Eigen::Vector3f CandidatedPoint(x,y,z);
//     Eigen::Vector3f TransformedPoint = transform.inverse() * CandidatedPoint;
//     if(fabs(TransformedPoint.x())<=TR_X && fabs(TransformedPoint.y())<=TR_Y && TransformedPoint.z()<=fov_distance && TransformedPoint.z()>0
//             && std::atan2(abs(TransformedPoint.y()),abs(TransformedPoint.z()))<TR_V && std::atan2(abs(TransformedPoint.x()),abs(TransformedPoint.z()))<TR_H)
//         std::cout << "在FOV内" <<std::endl;
//     views->addSphere(pcl::PointXYZ(x,y,z), 0.20, 0.0, 0.0, 1.0, "cen1");//blue
//     views->addSphere(pcl::PointXYZ(TransformedPoint.x(),TransformedPoint.y(),TransformedPoint.z()), 0.20, 0.0, 1.0, 0.0, "cen2");
// }


void Send_Fov(PclPro* pp, std::string namespace_)
{
    std::string path_out = pp->Tsp_Path + namespace_ + "Tsp_data.tsp";
    std::ifstream inputFile(path_out);
    // std::cout << "reading file" << std::endl;
    if (!inputFile.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
        return;
    }

    std::vector<PclPro::CameraViewPoint> viewPoints_save;

    std::string line;
    while (std::getline(inputFile, line)) {
        if (line.find("NODE_COORD_SECTION") != std::string::npos) {
            while (std::getline(inputFile, line)) {
                if (line.find_first_not_of("0123456789. -") != std::string::npos) {
                    continue;
                }
                std::istringstream iss(line);
                PclPro::CameraViewPoint point;
                int id;
                iss >> id >> point.x >> point.y >> point.z;
                viewPoints_save.push_back(point);
            }
            break;  // Exit the loop when reaching the NODE_COORD_SECTION
        }
    }
    inputFile.close();
    // std::cout << "finish reading file &&&&&&&&&&&&&&&&&&&&&&&&&&&&" << viewPoints_save.size() << std::endl;
    // view_mut.lock();
    poseArrayMsg.header.frame_id = "world"; // 设置帧ID
    poseArrayMsg.poses.clear();
    // line_strip.points.clear();
    line_list.points.clear();
    geometry_msgs::Point lasttp;
    for (int index : pp->tourInd)
    {
        if (index >= 0 && index <= pp->tourInd.size()) 
        {
            // 使用索引从 ViewPoints 中提取信息
            const PclPro::CameraViewPoint& viewpoint = viewPoints_save[index-1];

            // 创建位姿（pose）
            geometry_msgs::Pose tpose;
            tpose.position.x = viewpoint.x;
            tpose.position.y = viewpoint.y;
            tpose.position.z = viewpoint.z;
            // tpose.orientation.x = viewpoint.yaw;
            // tpose.orientation.y = viewpoint.pitch;
            //tpose.orientation.z = viewpoint.roll;
            //tpose.orientation.w = pp->ViewPoints.size();  // 这里你可以设置任何合适的值

            geometry_msgs::Point tp;
            tp.x = viewpoint.x;
            tp.y = viewpoint.y;
            tp.z = viewpoint.z;
           
            // 将位姿添加到 PoseArray 消息中
            poseArrayMsg.poses.push_back(tpose);

            points.points.push_back(tp);
            // line_strip.points.push_back(tp);
            // std::cout << "line_strip's points:(" << tp.x << ","<< tp.y << "," << tp.z << "," << "), index:" << index << std::endl; 
            if(!lasttp.x == 0.0 && !lasttp.y == 0.0 && !lasttp.z == 0.0 && lasttp != tp) {
                line_list.points.push_back(lasttp);
                line_list.points.push_back(tp);
            }
            lasttp = tp;
        }
    }
    // std::cout << "in fuc*********************" << poseArrayMsg.poses.size() << std::endl;
    // view_mut.unlock();
    // std::cout << "line_strip.points:[ ";
    // for (geometry_msgs::Point line : line_strip.points) {
    //         std::cout << line << " ";
    //     }
    // std::cout << "];" <<std::endl;
}

//主回调函数 累加地图
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, PclPro* pp) 
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //输入激光雷达数据
    pcl::fromROSMsg(*cloud_msg, *(pp->Lidar_Data));
    //地图处理函数
    pp->Map_Pro();
    //地图累加完毕 显示标志位
    pp->map_flag=1;
    //处理标志位
    pp->Update_flag=1;
}

//显示FOV空间坐标
void Show_ball1(pcl::visualization::PCLVisualizer::Ptr viewer, PclPro* pp) 
{
    for (const PclPro::CameraViewPoint& viewpoint : pp->FusionViewPoints) 
    {
        ik++;
        double radius = 0.4;
        pcl::PointXYZ sphere_center(viewpoint.x,viewpoint.y,viewpoint.z);
        viewer->addSphere(sphere_center, radius, 0.0, 0.0, 5.0, "hfsphere_" + std::to_string(ik));
    }
}

//显示FOV空间坐标
void Show_ball2(pcl::visualization::PCLVisualizer::Ptr viewer, PclPro* pp) 
{
    viewer->removeAllShapes(); // 删除所有形状
    for (const PclPro::CameraViewPoint& viewpoint : pp->ViewPoints) 
    {
        ik++;
        double radius = 0.1;
        pcl::PointXYZ sphere_center(viewpoint.x,viewpoint.y,viewpoint.z);
        viewer->addSphere(sphere_center, radius, 5.0, 0.0, 0.0, "hfsphere_" + std::to_string(ik));
    }
}

//显示bbox
void Show_Bbox(pcl::visualization::PCLVisualizer::Ptr viewer, PclPro* pp) 
{
        for (size_t i = 0; i < pp->boxInfos_s.size(); ++i) 
        {
            const PclPro::BoxInfo& boxInfo = pp->boxInfos_s[i];
            Eigen::Vector3f center = boxInfo.center;
            Eigen::Vector3f size = boxInfo.size;
            Eigen::Matrix4f orientation = boxInfo.orientation;
            Eigen::MatrixXf matrix(4, 8);
            matrix << 0.5*boxInfo.size(0),0.5*boxInfo.size(0),-0.5*boxInfo.size(0),-0.5*boxInfo.size(0),0.5*boxInfo.size(0),0.5*boxInfo.size(0),-0.5*boxInfo.size(0),-0.5*boxInfo.size(0),
                      0.5*boxInfo.size(1),-0.5*boxInfo.size(1),0.5*boxInfo.size(1),-0.5*boxInfo.size(1),0.5*boxInfo.size(1),-0.5*boxInfo.size(1),0.5*boxInfo.size(1),-0.5*boxInfo.size(1),
                      0.5*boxInfo.size(2),0.5*boxInfo.size(2),0.5*boxInfo.size(2),0.5*boxInfo.size(2),-0.5*boxInfo.size(2),-0.5*boxInfo.size(2),-0.5*boxInfo.size(2),-0.5*boxInfo.size(2),
                      1, 1, 1, 1, 1, 1, 1, 1;
            Eigen::MatrixXf ans = orientation*matrix; 
            Eigen::Vector3f max_values;
            Eigen::Vector3f min_values;
            max_values(0) = ans.row(0).maxCoeff();
            max_values(1) = ans.row(1).maxCoeff();
            max_values(2) = ans.row(2).maxCoeff();
            min_values(0) = ans.row(0).minCoeff();
            min_values(1) = ans.row(1).minCoeff();
            min_values(2) = ans.row(2).minCoeff();
            viewer->addCube(min_values(0),max_values(0),min_values(1),max_values(1),min_values(2),max_values(2),1.0, 1.0, 1.0, "1cube" + std::to_string(i));
        }
}

void Show_Occupymap(pcl::visualization::PCLVisualizer::Ptr viewer, PclPro* pp)
{
    viewer->removeAllShapes(); // 删除所有形状
    int temp = 0;
     for (int x = 0; x < pp->x_cells; ++x) 
     {
        for (int y = 0; y < pp->y_cells; ++y) 
        {
            for (int z = 0; z < pp->z_cells; ++z) 
            {
                if (pp->occupancy_grid_[x][y][z]) 
                { // 栅格被占用
                    // 计算栅格中心点的坐标
                    float center_x = pp->Bbox_min.x() + (x + 0.5) * pp->grid_resolution;
                    float center_y = pp->Bbox_min.y() + (y + 0.5) * pp->grid_resolution;
                    float center_z = pp->Bbox_min.z() + (z + 0.5) * pp->grid_resolution;

                    // 将栅格中心点添加到点云
                    pcl::PointXYZ center_point;
                    center_point.x = center_x;
                    center_point.y = center_y;
                    center_point.z = center_z;
                    viewer->addSphere(center_point, 0.1, 5.0, 0.0, 0.0, "occupysp" + std::to_string(temp));
                    temp++;
                }
            }
        }
    }

}

//可视化进程
void refreshview(PclPro* pp) 
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (pp->Cloud,"sample cloud");//显示点云
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");//设置点的大小
    viewer->addCoordinateSystem(1.0);
    pcl::PointXYZ Bbox_max(pp->Bbox_max.x(),pp->Bbox_max.y(),pp->Bbox_max.z());
    viewer->addSphere(Bbox_max, 0.8, 5.0, 0.0, 0.0, "bboxmax");
    pcl::PointXYZ Bbox_min(pp->Bbox_min.x(),pp->Bbox_min.y(),pp->Bbox_min.z());
    viewer->addSphere(Bbox_min, 0.8, 5.0, 0.0, 0.0, "bboxmin");
    viewer->addCube(Bbox_min.x, Bbox_max.x, Bbox_min.y, Bbox_max.y, Bbox_min.z, Bbox_max.z, 0, 1, 0, "bbox");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, "bbox");
    Show_Bbox(viewer,pp);
    while (!viewer->wasStopped()) 
    {
        //阻塞式更新点云数据
        if(pp->map_flag)
        {
            //viewer->removePointCloud("sample cloud");
            //viewer->updatePointCloud(pp->cloud,"sample cloud");
            viewer->removePointCloud("normals");
            viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (pp->Cloud, pp->Cloud_Normals, 1.5, 0.4, "normals"); // 显示法向量
            // Show_Occupymap(viewer,pp);
            pp->map_flag=0;
        }
        if(pp->Show_Fov)
        {
            Show_ball2(viewer,pp);
            Show_ball1(viewer,pp);
            pp->Show_Fov = 0;
        }
        // 更新点云显示
        viewer->spinOnce(100);
    }
}

//视点处理线程
// void GetFov(PclPro* pp) 
// {
//     while(1)
//     {
//         if(pp->Update_flag==1)
//         {  
//             pp->Update_flag=0;
//             //主处理函数
//             pp->Process();
//             // std::cout << "=======" << pp->ViewPoints.size() << std::endl;
//             //处理完了 发送数据
//             // pp->Send_Fovs = 1;
//             std::cout << "pp->Send_Fovs: " << pp->Send_Fovs << std::endl;
//             if(pp->Send_Fovs==1 && pp->ViewPoints.size()>3)
//             {
//                 pp->Tsp_Cal();
//                 pp->Send_Fovs = 0;
//                 Send_Fov(pp); 
//             }
//         }
//     }
// }

//主函数
int main(int argc, char** argv) 
{
    ros::init(argc, argv, "vision_create");
    ros::NodeHandle nh;
    std::string namespace_;
    namespace_ = nh.getNamespace();
    //点云处理器
    PclPro* pp =new PclPro(nh);
    //启动可视化线程
    // std::thread pclThread(refreshview, pp);
    //订阅数据进入回调
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/slf_kf_cloud", 1, boost::bind(&pointCloudCallback, _1, pp));
    //发布信息
    ros::Publisher pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("/ViewPoints", 10);
    ros::Publisher octomap_pub = nh.advertise<sensor_msgs::PointCloud2>("/Octomap", 1, true);
    sensor_msgs::PointCloud2 Octopc;
    ros::Publisher Pmarker_pub = nh.advertise<visualization_msgs::Marker>("/ViewPoints_Points", 1);
    // ros::Publisher Lmarker_pub = nh.advertise<visualization_msgs::Marker>("/ViewPoints_Lines", 1);
    ros::Publisher lmarker_pub = nh.advertise<visualization_msgs::Marker>("/ViewPoints_lines", 1);
        points.header.frame_id = line_list.header.frame_id = line_strip.header.frame_id = "world";
        points.header.stamp = line_list.header.stamp = line_strip.header.stamp = ros::Time::now();
        points.ns = line_list.ns = line_strip.ns = "points_and_lines";
        points.action = line_list.action = line_strip.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = line_list.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        points.id = 0;
        line_strip.id = 1;
        line_list.id = 2;
        points.type = visualization_msgs::Marker::POINTS;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        points.scale.x = 0.5;
        points.scale.y = 0.5;
        line_list.scale.x = 0.3;  // 设置线宽
        line_strip.scale.x = 0.3;  // 设置线宽
        points.color.g = 1.0f;
        points.color.a = 1.0;
        line_list.color.r = 1.0;
        line_list.color.a = 1.0;
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;
    //启动视点处理线程
    // std::thread FovThread(GetFov, pp);
    //视点排序发送线程
    // std::thread Tsp_Send_Thread(Tsp_Send, pp);
    // 发布PoseArray消息
    while (ros::ok()) {
        if(pp->Update_flag==1)
        {  
            pp->Update_flag=0;
            //主处理函数
            pp->Process();
            // std::cout << "=======" << pp->ViewPoints.size() << std::endl;
            //处理完了 发送数据
            // pp->Send_Fovs = 1;
            // std::cout << "pp->Send_Fovs: " << pp->Send_Fovs << std::endl;
            if(pp->Send_Fovs==1 && pp->ViewPoints.size()>3)
            {
                pp->Tsp_Cal();
                pp->Send_Fovs = 0;
                Send_Fov(pp, namespace_); 
            }
        }
        // std::cout << "*********************" << poseArrayMsg.poses.size() << std::endl;
        if(!poseArrayMsg.poses.empty()) {
            // view_mut.lock();
            pose_array_pub.publish(poseArrayMsg);
            // view_mut.unlock();
        }
        // pcl::toROSMsg(*(pp->pro_cloud), Octopc);
        // Octopc.header.frame_id = "world";
        octomap_pub.publish(Octopc);
        pcl::toROSMsg(*(pp->Occupancy_Cloud), Octopc);
        Octopc.header.frame_id = "world";
        octomap_pub.publish(Octopc);
        Pmarker_pub.publish(points);
        // Lmarker_pub.publish(line_strip);
        lmarker_pub.publish(line_list);
        ros::spinOnce();
        ros::Duration(1.0).sleep(); // 发布频率，这里设置为1秒
    }
    return 0;
}