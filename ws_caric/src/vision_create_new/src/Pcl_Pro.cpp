#include "Pcl_Pro.h"
#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <fstream>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <yaml-cpp/yaml.h>
#include <random>
#include <pcl/io/pcd_io.h>
#include <mutex>
#include <Eigen/Core>
#include <string>
readWriteLock rwLock; 

//降采样函数
void PclPro::DownSample(pcl::PointCloud<pcl::PointXYZ>::Ptr clouds)
{
    //pcl::VoxelGrid<pcl::PointXYZ> sor;
    //sor.setInputCloud(clouds);
    //sor.setLeafSize(Filter_Para,Filter_Para,Filter_Para); 
    //sor.filter(*clouds);
    Cloud_Size = clouds->size();
    // std::cout << "降采样后点云数量为:" <<clouds->size()<<std::endl;
}

//读取设置参数
void PclPro::GetParam(void)
{
    
    Param_Path += scene + "_param.yaml";
    std::cout << Param_Path << std::endl;
    YAML::Node config = YAML::LoadFile(Param_Path);
    //获取参数
    // Filter_Para = config["config"]["Filter_Para"].as<float>();
    Normal_Radius = config["config"]["Normal_Radius"].as<float>();
    // Lidar_filter = config["config"]["Lidar_filter"].as<float>();
    grid_resolution = config["config"]["grid_resolution"].as<float>();
    Box_x = config["config"]["Box_x"].as<float>();
    Box_y = config["config"]["Box_y"].as<float>();
    Box_z = config["config"]["Box_z"].as<float>();
    Fov_Min_Dis = config["config"]["Fov_Min_Dis"].as<float>();
    fov_distance = config["config"]["fov_distance"].as<float>();
    std::cout << "(((((((((((((((((()))))))))))))))))) GetParam成功" << std::endl;
}

//获取BoundingBox的数据以及生成小box的数据
void PclPro::Get_BoundingBox(void)
{
    std::string map = Yaml_Path;
    YAML::Node config = YAML::LoadFile(map);
    for (const auto& boxNode : config) {
        BoxInfo temp_box;
        std::vector<double> Data = boxNode.second["center"].as<std::vector<double>>();
        temp_box.center = Eigen::Vector3f(Data[0], Data[1], Data[2]);

        Data = boxNode.second["size"].as<std::vector<double>>();
        temp_box.size = Eigen::Vector3f(Data[0], Data[1], Data[2]);

        std::vector<double> orientationData = boxNode.second["orientation"].as<std::vector<double>>();
        Eigen::Matrix4f orientation;
        for (int i = 0; i < 16; ++i) 
        {
            orientation(i / 4, i % 4) = orientationData[i];
        }
        orientation.block<3, 1>(0, 3) = temp_box.center;
        temp_box.orientation = orientation;

        boxInfos.push_back(temp_box);
    }
    //这里加入内盒参数(用box检测)
    for (const auto& boxNode : config) {
        BoxInfo temp_box;
        std::vector<double> Data = boxNode.second["center"].as<std::vector<double>>();
        temp_box.center = Eigen::Vector3f(Data[0], Data[1], Data[2]);

        Data = boxNode.second["size"].as<std::vector<double>>();
        temp_box.size = Eigen::Vector3f(Box_x*Data[0], Box_y*Data[1], Box_z*Data[2]);

        std::vector<double> orientationData = boxNode.second["orientation"].as<std::vector<double>>();
        Eigen::Matrix4f orientation;
        for (int i = 0; i < 16; ++i) 
        {
            orientation(i / 4, i % 4) = orientationData[i];
        }
        temp_box.orientation = orientation;
        
        boxInfos_s.push_back(temp_box);
    }

    //计算最大点 和最小点
    Eigen::Vector3f min_xyz(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    Eigen::Vector3f max_xyz(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
    for (const BoxInfo& box : boxInfos) {

        Eigen::Vector3f corners[8];

        for (int i = 0; i < 8; i++) {
            Eigen::Vector3f offset;
            for (int j = 0; j < 3; j++) 
            {
                offset[j] = (i & (1 << j)) ? box.size[j] * 0.5f : -box.size[j] * 0.5f;
            }
            // 应用旋转变换
            corners[i] = box.orientation.block<3, 3>(0, 0) * offset + box.center;
        }
        // 更新全局最小值和最大值
        for (int i = 0; i < 8; i++) {
            min_xyz = min_xyz.cwiseMin(corners[i]);
            max_xyz = max_xyz.cwiseMax(corners[i]);
        }
    }
    Bbox_max = max_xyz;
    Bbox_min = min_xyz;
    x_cells = static_cast<int>((Bbox_max.x() - Bbox_min.x()) / grid_resolution)+1;
    y_cells = static_cast<int>((Bbox_max.y() - Bbox_min.y()) / grid_resolution)+1;
    z_cells = static_cast<int>((Bbox_max.z() - Bbox_min.z()) / grid_resolution)+1;
    std::cout<<"栅格地图实际长："<<Bbox_max.x()-Bbox_min.x()<<" 宽："<<Bbox_max.y()-Bbox_min.y()<<" 高："<<Bbox_max.z()-Bbox_min.z()<<std::endl;
    std::cout<<"栅格地尺寸："<<x_cells<<" 宽："<<y_cells<<" 高："<<z_cells<<std::endl;
    std::cout << "(((((((((((((((((()))))))))))))))))) Get_BoundingBox成功" << std::endl;
}

//检测点是否在box内
bool PointIsInsideBox(const pcl::PointXYZ& point, PclPro::BoxInfo box) 
{
    // 将点从全局坐标系转换到盒子的局部坐标系中
    Eigen::Vector3f local_point(
        point.x - box.center[0],
        point.y - box.center[1],
        point.z - box.center[2]
    );

    // 将点根据盒子的方向矩阵旋转到局部坐标系
    Eigen::Vector3f local_point_rotated = box.orientation.block<3, 3>(0, 0).transpose() * local_point;

    // 检查点是否在盒子的长方形范围内
    Eigen::Vector3f half_size = 0.5f * box.size;
    return (
        std::abs(local_point_rotated[0]) <= half_size[0] &&
        std::abs(local_point_rotated[1]) <= half_size[1] &&
        std::abs(local_point_rotated[2]) <= half_size[2]
    );
}

//1 遍历点云删除不在box内的点云 2 遍历之前的地图将与之前地图距离靠近的点删除
void PclPro::FilterOutsideBoxes(pcl::PointCloud<pcl::PointXYZ>::Ptr clouds) 
{
    // if(!clouds->empty()) {
    //     pcl::PointIndices::Ptr indices_in_boxes(new pcl::PointIndices);
    //     pcl::KdTreeFLANN<pcl::PointXYZ> lKdTree;
    //     lKdTree.setInputCloud(clouds);
    //     std::vector<int> p_indices;
    //     std::vector<float> p_Distances;

    //     for (std::size_t i = 0; i < clouds->size(); ++i) {
    //         const pcl::PointXYZ& point = clouds->at(i);
    //         bool inside_any_box = false;
    //         for (BoxInfo box : boxInfos) {
    //             if (PointIsInsideBox(point,box)) 
    //             {
    //                 inside_any_box = true;
    //                 break;
    //             }
    //         } 
    //         if(lKdTree.radiusSearch(clouds->at(i), Lidar_filter, p_indices, p_Distances) > 0) {
    //             inside_any_box = false; 
    //         }      
    //         if(inside_any_box) {
    //             indices_in_boxes->indices.push_back(static_cast<int>(i));
    //         }
    //     }
    //     // 创建一个点云过滤器，仅保留在任何盒子内的点
    //     pcl::ExtractIndices<pcl::PointXYZ> extract;
    //     extract.setInputCloud(clouds);
    //     extract.setIndices(indices_in_boxes);
    //     extract.setNegative(false); // 设置为false以仅保留选定的点
    //     extract.filter(*clouds);
    // } else {
        pcl::PointIndices::Ptr indices_in_boxes(new pcl::PointIndices);
        for (std::size_t i = 0; i < clouds->size(); ++i) {
            const pcl::PointXYZ& point = clouds->at(i);
            bool inside_any_box = false;
            for (BoxInfo box : boxInfos) {
                if (PointIsInsideBox(point,box) && point.z >= 0.1) {
                    inside_any_box = true;
                    break;
                }
            }       
            if(inside_any_box) {
                indices_in_boxes->indices.push_back(static_cast<int>(i));
            }
        }
        // 创建一个点云过滤器，仅保留在任何盒子内的点
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(clouds);
        extract.setIndices(indices_in_boxes);
        extract.setNegative(false); // 设置为false以仅保留选定的点
        extract.filter(*clouds);
    // }
}   

//计算法向量
void PclPro::ComputeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr clouds)
{
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(clouds);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(Normal_Radius); // 设置搜索半径，根据你的数据调整
    ne.compute(*Cloud_Normals);
}

//根据盒子校正法向量
void PclPro::BoxCorrectNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr clouds) 
{
    // 对每个盒子执行法向量校正
    for (int i = 0; i < clouds->size(); ++i) 
    {
        pcl::PointXYZ point = clouds->points[i];
        pcl::Normal point_normal = Cloud_Normals->points[i];

        for (BoxInfo box : boxInfos) {
            if (PointIsInsideBox(point, box)) 
            {
                Eigen::Vector3f box_normal = box.orientation.block<3, 1>(0, 2);
                Eigen::Vector3f projected_normal = box.orientation.block<3, 3>(0, 0) * point_normal.getNormalVector3fMap();
                if (projected_normal.dot(point.getVector3fMap() - box.center) < 0) 
                {
                    point_normal.getNormalVector3fMap() *= -1;
                }
                point_normal.getNormalVector3fMap() *= -1;
                Cloud_Normals->points[i] = point_normal;
                break;  
            }
        }
    }
}

//计算FOV
void PclPro::Compute_Fov(const pcl::Normal& normal, const pcl::PointXYZ& start_point, CameraViewPoint& tempfovs)
{
    // 计算法向量的方向向量
    Eigen::Vector3d view_direction(normal.normal_x, normal.normal_y, normal.normal_z);
    view_direction.normalize();
    // 计算FOV的位置（xyz坐标）
    Eigen::Vector3d fov_position(start_point.x - fov_distance * view_direction.x(),
                                start_point.y - fov_distance * view_direction.y(),
                                start_point.z - fov_distance * view_direction.z());
    tempfovs.x=fov_position.x();
    tempfovs.y=fov_position.y();
    tempfovs.z=fov_position.z();
    // 计算俯仰角 
    tempfovs.pitch = std::atan2(view_direction.y(),sqrt(view_direction.x()*view_direction.x()+view_direction.z()*view_direction.z())); 
    tempfovs.pitch = -tempfovs.pitch * 180.0 / M_PI; // 俯仰角转换为度
    // 计算偏航角
    tempfovs.yaw = std::atan2(view_direction.x(),view_direction.z());
    tempfovs.yaw = tempfovs.yaw * 180.0 / M_PI; // 偏航角转换为度
    // 设置横滚角为0 现在测试打开;
    tempfovs.roll = 90.0;
} 

// // 计算两个点的欧氏距离
// float euclideanDistance(const Eigen::Vector3f& point1,const Eigen::Vector3f& point2) 
// {
//     return (point1 - point2).norm();
// }
// // 判断是否小于阈值
// bool isDistanceBelowThreshold(const Eigen::Vector3f& point1, const pcl::PointXYZ& pointP, float threshold) {
//     Eigen::Vector3f point2;
//     point2 << pointP.x, pointP.y, pointP.z;
//     return euclideanDistance(point1, point2) < threshold;
// }

// 提取增量索引
void PclPro::Extract_Indics(void)
{
    std::vector<int> indices_to_extract;
    float num = 0;
    if(Last_Cloud->empty()) {
        *Last_Cloud = *pro_cloud;
        num = Last_Cloud->size();
        for (size_t i = 0; i < pro_cloud->points.size(); ++i) 
        {
            indices_to_extract.push_back(i);
        }
    } else {
        // 创建一个kd树以加速最近邻搜索
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(Last_Cloud);
        for (size_t i = 0; i < pro_cloud->points.size(); ++i) 
        {
            pcl::PointXYZ search_point = pro_cloud->points[i];
            std::vector<int> nn_indices;
            std::vector<float> nn_dists;
            
            if (kdtree.radiusSearch(search_point, 0.25, nn_indices, nn_dists) > 0) {
                ;
            } 
            else 
                indices_to_extract.push_back(i);
        }
        *Last_Cloud = *pro_cloud;
        num  = indices_to_extract.size();
    }
    Index_Add=indices_to_extract;
    // std::cout << "增量点云数量为:" << num <<std::endl;
}

//TSP 3.0调整视点顺序
void PclPro::Tsp_Cal(void)
{
    //编写数据配置文件
    std::string temp = Tsp_Path + namespace_ + "Tsp_data.tsp";
    std::ofstream inputFile1(temp);
    inputFile1 << "COMMENT : path" << std::endl;
    inputFile1 << "TYPE: TSP" << std::endl;
    inputFile1 << "DIMENSION: " << ViewPoints.size() << std::endl;
    inputFile1 << "EDGE_WEIGHT_TYPE: EUC_3D" << std::endl;
    inputFile1 << "NODE_COORD_SECTION" << std::endl;
    // std::cout << "ViewPoints size: " << ViewPoints.size() << std::endl;
    for (size_t i = 0; i < ViewPoints.size(); i++) 
    {
        inputFile1 << i + 1 << " " << ViewPoints[i].x << " " << ViewPoints[i].y << " " << ViewPoints[i].z << std::endl;
        // inputFile1 << i + 1 << " " << FusionViewPoints[i].x << " " << FusionViewPoints[i].y << " " << FusionViewPoints[i].z << std::endl;
    }
    // std::cout << temp << std::endl;
    inputFile1.close();
    //编写启动文件
    temp = Tsp_Path + namespace_ + "Tsp_run.par";
    std::ofstream inputFile2(temp);
    inputFile2 << "PROBLEM_FILE = " << Tsp_Path << namespace_ << "Tsp_data.tsp" << std::endl;
    inputFile2 << "OPTIMUM = 378032 " << std::endl;
    inputFile2 << "PATCHING_C = 3" << std::endl;
    inputFile2 << "PATCHING_A = 2" << std::endl;
    inputFile2 << "TOUR_FILE = "<< Tsp_Path << namespace_ << "output.txt" << std::endl;
    std::string command = Tsp_Path + "/" + "LKH" + " " + Tsp_Path + namespace_ + "Tsp_run.par";
    // std::cout << command << std::endl;
    inputFile2.close();
    int exitCode = std::system(command.c_str());
    //读取输出数据
    std::ifstream iofile(Tsp_Path + namespace_ + "output.txt");
    std::string line;
    std::vector<int> tourIndices; 
    bool tourSection = false;
    while (std::getline(iofile, line)) 
    {
        if (tourSection) 
        {
            int number;
            if (std::stringstream(line) >> number) 
            {
                if(number==-1)
                    break;
                tourIndices.push_back(number);
            }
        } 
        else if (line == "TOUR_SECTION") 
        {
            tourSection = true;
        }
    }

    // std::cout << "After LKH:[";
    // for (int index : tourIndices) {
    //         std::cout << index << " ";
    //     }
    // std::cout << "];" <<std::endl;

    iofile.close();
    tourInd=tourIndices;
    // std::cout << "tourInd size: " << tourInd.size() << std::endl;
    //按索引push进去
    /*OutViewPoints.clear();
    for(int i=0;i<ViewPoints.size();i++)
    {
        OutViewPoints.push_back(ViewPoints[tourIndices[i]-1]);
    }
    std::cout<<"tsp后的size"<< OutViewPoints.size()<<std::endl;*/
}

//筛选视点
void PclPro::SelectViewpoint(pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_datas)
{   
    //提取增量数据索引
    Extract_Indics();
    //采样所有视点
    for(int i=0;i<Index_Add.size();i++) {
        // if 
        pcl::Normal nl = pro_normals->points[Index_Add[i]];
        pcl::PointXYZ pt = pro_cloud->points[Index_Add[i]];
        CameraViewPoint tempfov; 
        Compute_Fov(nl,pt,tempfov);
        tempfov.Center_Point = Eigen::Vector3f(pro_cloud->points[Index_Add[i]].x,pro_cloud->points[Index_Add[i]].y,pro_cloud->points[Index_Add[i]].z);  
        // IspointOffFov(pro_cloud, pro_cloud->at(Index_Add[i]), tempfov);
        ViewPoints.push_back(tempfov);
        // std::cout << "FOV 中心点坐标 (x, y, z):(" << pro_cloud->points[Index_Add[i]].x << ", " << pro_cloud->points[Index_Add[i]].y << ", " << pro_cloud->points[Index_Add[i]].z << ")" << std::endl;
    }

    // CreateViewpoint();

    // int numvp = 0;
    //用栅格地图筛选掉内点
    for (auto it = ViewPoints.begin(); it != ViewPoints.end();) 
    {
        CameraViewPoint& viewpoint = *it;
    
        int grid_x = static_cast<int>((viewpoint.x - Bbox_min.x()) / grid_resolution);
        int grid_y = static_cast<int>((viewpoint.y - Bbox_min.y()) / grid_resolution);
        int grid_z = static_cast<int>((viewpoint.z - Bbox_min.z()) / grid_resolution);
        pcl::PointXYZ point;
        point.x = viewpoint.x; // x 坐标
        point.y = viewpoint.y; // y 坐标
        point.z = viewpoint.z; // z 坐标
        // std::cout << "出错位置SelectViewpoint()-011, grid_x: " << grid_x << " grid_y: " << grid_y << " grid_z: " << grid_z << std::endl;
        bool isOccupied = true;
        if ((0 <= grid_x && grid_x <= x_cells) && (0 <= grid_y && grid_y <= y_cells) && (0 <= grid_z && grid_z <= z_cells)) {
            isOccupied = occupancy_grid_[grid_x][grid_y][grid_z];
        }
        
        //稀疏视点 过于靠近的视点排除掉
        for (auto other_it = ViewPoints.begin(); other_it != ViewPoints.end(); ++other_it) 
        {
            if (it != other_it) 
            {
                CameraViewPoint& other_viewpoint = *other_it;
                double distance = sqrt((viewpoint.x - other_viewpoint.x) * (viewpoint.x - other_viewpoint.x) +
                                    (viewpoint.y - other_viewpoint.y) * (viewpoint.y - other_viewpoint.y) +
                                    (viewpoint.z - other_viewpoint.z) * (viewpoint.z - other_viewpoint.z));

                if (distance < Fov_Min_Dis) 
                {
                    isOccupied = true;
                    break;
                }
            }
        }
        //用视点约束筛点
        if(viewpoint.pitch>80 || viewpoint.pitch<-80)
        {
            isOccupied = true;
        }
        //用小box筛点
        for (BoxInfo box : boxInfos_s) 
        {              
            if (PointIsInsideBox(point, box) || isOccupied) 
            {
                isOccupied = true;
                break;
            }
        }
        if (isOccupied) 
        {
            it = ViewPoints.erase(it);
            // numvp++;
        } else {
            ++it;
        }
    }
    //调整视点顺序TSP
    // std::cout << "视点数量"<< ViewPoints.size() <<std::endl;
    // std::cout << "删除内点数量"<< numvp <<std::endl;
} 

//填充空洞
void PclPro::Occupy_Map_Fill(void)
{
    int fillnums=0;
    for (int x = 1; x < x_cells - 1; ++x)
    {
        for (int y = 1; y < y_cells - 1; ++y)
        {
            for (int z = 1; z < z_cells - 1; ++z)
            {
                // 检查前后左右上下六个相邻的栅格
                bool is_occupied = (occupancy_grid_[x-1][y][z])&&
                                   (occupancy_grid_[x+1][y][z]) &&
                                   (occupancy_grid_[x][y-1][z]) &&
                                   (occupancy_grid_[x][y+1][z]) &&
                                   (occupancy_grid_[x][y][z-1]) &&
                                   (occupancy_grid_[x][y][z+1]);
                if(is_occupied)
                    occupancy_grid_[x][y][z] = true, fillnums++;
            }
        }
    }
    // std::cout << "填充内部栅格数量为:" << fillnums <<std::endl;
}

//维护栅格占用地图 筛激光数据 
void PclPro::Occupy_Map(void)
{
    pcl::PointCloud<pcl::PointXYZ> new_point_cloud;
    for (const pcl::PointXYZ& point : Lidar_Data->points) {
        int x_cell = static_cast<int>((point.x - Bbox_min.x()) / grid_resolution);
        int y_cell = static_cast<int>((point.y - Bbox_min.y()) / grid_resolution);
        int z_cell = static_cast<int>((point.z - Bbox_min.z()) / grid_resolution);
        if(occupancy_grid_[x_cell][y_cell][z_cell] == true)
        {
            ;
        } else {
            occupancy_grid_[x_cell][y_cell][z_cell] = true;  // 设置新的栅格占用
            new_point_cloud.points.push_back(point);

            BoxInfo current_box;
            for (BoxInfo box : boxInfos) {
                if (PointIsInsideBox(point, box)) {
                    current_box = box;
                    break;
                }
            }

            pcl::PointXYZ start_point = point;pcl::PointXYZ new_point;pcl::PointXYZ vector;
            for (const pcl::PointXYZ& center : current_box.center_column) {
                float side_diff = std::abs(point.data[current_box.max_side] - center.data[current_box.max_side]);
                // std::cout << "z_diff: (" << z_diff << ")?" << grid_resolution << std::endl;
                if (side_diff < 0.5 * grid_resolution) {  // 使用一个小的阈值判断相差一个 grid_resolution
                    // std::cout << "Point: (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
                    // std::cout << "Found a point in center_column with z difference of one grid_resolution:" << std::endl;
                    // std::cout << "Center: (" << center.x << ", " << center.y << ", " << center.z << ")" << std::endl;
                    vector = center;
                    break;
                }
            }

            vector.x -= start_point.x;
            vector.y -= start_point.y;
            vector.z -= start_point.z;

            float length = sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
            vector.x /= length;
            vector.y /= length;
            vector.z /= length;

            // Calculate the number of points to draw along the line
            int num_points = static_cast<int>(length / grid_resolution);
            // Draw points along the line
            for (int i = 0; i <= num_points; ++i) {
                new_point = start_point;
                new_point.x += i * grid_resolution * vector.x;
                new_point.y += i * grid_resolution * vector.y;
                new_point.z += i * grid_resolution * vector.z;

                int x_cell = static_cast<int>((new_point.x - Bbox_min.x()) / grid_resolution);
                int y_cell = static_cast<int>((new_point.y - Bbox_min.y()) / grid_resolution);
                int z_cell = static_cast<int>((new_point.z - Bbox_min.z()) / grid_resolution);
                occupancy_grid_[x_cell][y_cell][z_cell] = true;
                Occupancy_Cloud->push_back(new_point);
            }
        }
    }
    Lidar_Data->points = new_point_cloud.points;
}

void PclPro::Occupy_Map_Init(void)
{
    occupancy_grid_ = std::make_unique<std::unique_ptr<std::unique_ptr<bool[]>[]>[]>(x_cells);
    for (int x = 0; x < x_cells; ++x) {
        occupancy_grid_[x] = std::make_unique<std::unique_ptr<bool[]>[]>(y_cells);
        for (int y = 0; y < y_cells; ++y) 
        {
            occupancy_grid_[x][y] = std::make_unique<bool[]>(z_cells);
        }
    }
    // 初始化栅格占用地图
    for (int x = 0; x < x_cells; ++x) {
        for (int y = 0; y < y_cells; ++y) 
        {
            for (int z = 0; z < z_cells; ++z) 
            {
                occupancy_grid_[x][y][z] = false; // 初始状态为未占用
            }
        }
    }
    for (BoxInfo& box : boxInfos) {
        int x_center = ((box.center.x() - Bbox_min.x()) / grid_resolution);
        int y_center = ((box.center.y() - Bbox_min.y()) / grid_resolution);
        int z_center = ((box.center.z() - Bbox_min.z()) / grid_resolution);
        occupancy_grid_[x_center][y_center][z_center] = true;
        pcl::PointXYZ point = {box.center.x(), box.center.y(), box.center.z()};
        Occupancy_Cloud->push_back(point);

        int x_occupany;int y_occupany;int z_occupany;
        box.size.maxCoeff(&box.max_side);
        float max_size = box.size[box.max_side];
        Eigen::Vector4f guide_point = {0 ,0 ,0, 1.0f};
        for (float offset = -max_size / 2.0f; offset <= max_size / 2.0f; offset += grid_resolution) {
            std::cout << "good start" << std::endl;
            guide_point = {0 ,0 ,0, 1.0f};
            guide_point(box.max_side) = guide_point(box.max_side) + offset;
            guide_point = box.orientation * guide_point;
            point = {guide_point.x(), guide_point.y(), guide_point.z()};
            box.center_column.push_back(point);
            x_occupany = ((guide_point.x() - Bbox_min.x()) / grid_resolution);
            y_occupany = ((guide_point.y() - Bbox_min.y()) / grid_resolution);
            z_occupany = ((guide_point.z() - Bbox_min.z()) / grid_resolution);
            std::cout << x_occupany << " " << y_occupany << " " << z_occupany << " " << std::endl;
            // if (x_occupany > 0 && x_occupany < x_cells && y_occupany > 0 && y_occupany < y_cells && z_occupany > 0 && z_occupany < z_cells) {
            occupancy_grid_[x_occupany][y_occupany][z_occupany] = true;
            point.x = guide_point.x();point.y = guide_point.y();point.z = guide_point.z(); 
            Occupancy_Cloud->push_back(point);
            // }
        }
    }
    
   
    //复制一份？
    new_occupancy_grid_ = std::make_unique<std::unique_ptr<std::unique_ptr<bool[]>[]>[]>(x_cells);
    for (int x = 0; x < x_cells; ++x) {
        new_occupancy_grid_[x] = std::make_unique<std::unique_ptr<bool[]>[]>(y_cells);
        for (int y = 0; y < y_cells; ++y) 
        {
            new_occupancy_grid_[x][y] = std::make_unique<bool[]>(z_cells);
        }
    }
    // 初始化栅格占用地图
    for (int x = 0; x < x_cells; ++x) {
        for (int y = 0; y < y_cells; ++y) 
        {
            for (int z = 0; z < z_cells; ++z) 
            {
                new_occupancy_grid_[x][y][z] = false; // 初始状态为未占用
            }
        }
    }
}

//
void PclPro::CreateViewpoint(pcl::PointCloud<pcl::PointXYZ>::Ptr clouds) 
{
    int cover_size = 0;
    //创建一个临时点云用于修改
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    *tmp_cloud = *clouds;
    // std::cout << tmp_cloud->size() << std::endl;

    //遍历已经产生的FOV将已被覆盖的点云删除
    // MoveCoverPoints(tmp_cloud);
    for (int i = 0; i < tmp_cloud->size(); ++i) {
        if (std::find(UsedPointIndices.begin(), UsedPointIndices.end(), i) != UsedPointIndices.end()) {
            // 如果点的索引不在 usedPointIndices 中，将其添加到未使用的点的索引中
            tmp_cloud->points[i] = pcl::PointXYZ();
        } 
    }    
    
    //删减后的视角
    //创建KdTree对象 输入增量数据
    // pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    // kdtree.setInputCloud(tmp_cloud); 
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    kdtree->setInputCloud(tmp_cloud);

    std::vector<int> pointIdxNKNSearch(5);
    std::vector<float> pointNKNSquaredDistance(5);
    std::vector<int> fifthNearestPoint;
    
    kdtree->nearestKSearch(*(tmp_cloud->begin() + tmp_cloud->size() - 500 - 1), 1, pointIdxNKNSearch, pointNKNSquaredDistance);
    fifthNearestPoint.push_back(pointIdxNKNSearch[0]);
    auto it = tmp_cloud->at(pointIdxNKNSearch[0]);
    int i = 0;
    while (i <= 500) {
        kdtree->nearestKSearch(it, 5, pointIdxNKNSearch, pointNKNSquaredDistance);
        // std::cout << "pointIdxNKNSearch:" << pointIdxNKNSearch[4] << std::endl;
        if (std::find(UsedPointIndices.begin(), UsedPointIndices.end(), pointIdxNKNSearch[4]) != UsedPointIndices.end()) {
            std::random_device rd;
            std::mt19937 generator(rd());
            std::uniform_int_distribution<int> distribution(tmp_cloud->size() - 500, tmp_cloud->size());
            int randomValue;
            do
            {randomValue = distribution(generator);}while(UsedPointIndices.find(randomValue - 1) != UsedPointIndices.end()); 
            // std::cout << "randomValue:" << randomValue << std::endl;
            // std::cout << i << std::endl;
            it = tmp_cloud->at(randomValue - 1);
            // UsedPointIndices.insert(randomValue - 1);
            // i += 1;
            continue;
        }
        // UsedPointIndices.insert(pointIdxNKNSearch[4]);
        UsedPointIndices.insert(pointIdxNKNSearch.begin(), pointIdxNKNSearch.end());
        fifthNearestPoint.push_back(pointIdxNKNSearch[4]);
        it = tmp_cloud->at(pointIdxNKNSearch[4]);  // 将it更新为 pointIdxNKNSearch 中的第五个索引对应的点
        i += 5;  // 增加迭代计数器
    }

    for(const int index : fifthNearestPoint) {
        pcl::Normal nl = Cloud_Normals->points[index];
        pcl::PointXYZ pt = tmp_cloud->points[index];
        CameraViewPoint tmp_fov;
        Compute_Fov(nl,pt,tmp_fov);
        tmp_fov.Center_Point = Eigen::Vector3f(tmp_cloud->points[index].x,tmp_cloud->points[index].y,tmp_cloud->points[index].z); 
        IspointOffFov(tmp_cloud, tmp_cloud->at(index), tmp_fov);
        ViewPoints.push_back(tmp_fov);
    }
}

//检测FOV里有没有点
void PclPro::IspointOffFov(pcl::PointCloud<pcl::PointXYZ>::Ptr clouds, const pcl::PointXYZ& points, CameraViewPoint& tmp_fovs)  //, std::vector<int>& indices
{   
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    // kdtree->setInputCloud(clouds);
    kdtree->setInputCloud(Cloud);
    float SecondSearchRadius = Fov_P * Fov_Radius;
    std::vector<int> indices;
    std::vector<float> SecondPointDistances;
    //搜索该点2.5倍半径内的点
    kdtree->radiusSearch(points, SecondSearchRadius, indices, SecondPointDistances); //2.5

    //y是height x是width
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << tmp_fovs.x, tmp_fovs.y, tmp_fovs.z; // 设置平移        
    transform.rotate(Eigen::AngleAxisf(tmp_fovs.yaw * M_PI / 180.0 ,Eigen::Vector3f::UnitY())); // 设置偏航 yaw
    transform.rotate(Eigen::AngleAxisf(tmp_fovs.pitch * M_PI / 180.0 , Eigen::Vector3f::UnitX())); // 设置俯仰 pitch 
    transform.rotate(Eigen::AngleAxisf(tmp_fovs.roll * M_PI / 180.0 , Eigen::Vector3f::UnitZ())); // 设置横滚 roll
    SelectedPointIndices.push_back(indices[0]);
    for(std::vector<int>::iterator it = indices.begin(); it != indices.end(); ++it) {
        pcl::PointXYZ point0 = clouds->points[*it];
        Eigen::Vector3f CandidatedPoint(point0.x,point0.y,point0.z);
        Eigen::Vector3f TransformedPoint = transform.inverse() * CandidatedPoint;
        if(fabs(TransformedPoint.x())<=TR_X && fabs(TransformedPoint.y())<=TR_Y && TransformedPoint.z()<=fov_distance+3 && TransformedPoint.z()>=0
                && std::atan2(abs(TransformedPoint.y()),abs(TransformedPoint.z()))<=TR_V && std::atan2(abs(TransformedPoint.x()),abs(TransformedPoint.z()))<=TR_H) {
            SelectedPointIndices.push_back(*it);
            tmp_fovs.Point_in_fov.push_back(*it);
        }
    }
}

//
void PclPro::UpdateViewpoint(void)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr View_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<CameraViewPoint> BubblingViewPoints;
    // 假设你有一个存储 CameraViewPoint 结构体的容器
    if (ViewPoints.size() > 100) {
        BubblingViewPoints = std::vector<CameraViewPoint>(ViewPoints.end() - 100, ViewPoints.end());
    }
    else {
        return;
    }
    

    // 使用冒泡排序对 BubblingViewPoints 结构体按 Point_in_fov.size() 进行从大到小的排序
    for (size_t i = 0; i < BubblingViewPoints.size(); ++i) {
        for (size_t j = 0; j < BubblingViewPoints.size() - i - 1; ++j) {
            if (BubblingViewPoints[j].Point_in_fov.size() < BubblingViewPoints[j + 1].Point_in_fov.size()) {
                // 交换 BubblingViewPoints[j] 和 BubblingViewPoints[j + 1]
                std::swap(BubblingViewPoints[j], BubblingViewPoints[j + 1]);
            }
        }
    }

    // 将 BubblingViewPoints 转换为 pcl::PointXYZ 并存储在点云中
    for (size_t i = 0; i < BubblingViewPoints.size(); ++i) {
        const CameraViewPoint& fov = BubblingViewPoints[i];
        pcl::PointXYZ tmp_point;
        tmp_point.x = fov.Center_Point.x();
        tmp_point.y = fov.Center_Point.y();
        tmp_point.z = fov.Center_Point.z();
        View_cloud->push_back(tmp_point);
        BubblingViewPoints[i].index_in_fov = i;
    }

    for (int i = 1; i <= View_cloud->size(); ++i) {
        UsedBubblePointIndices.insert(i);
    }

    // 建立 KdTree
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    kdtree->setInputCloud(View_cloud);
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    
    int i = 0;
    auto search_point = *(View_cloud->begin());
    while (i <= View_cloud->size()) {
        if (!UsedBubblePointIndices.empty()) {
            search_point = View_cloud->at(*(UsedBubblePointIndices.begin()));
        }
        kdtree->radiusSearch(search_point, Fov_P, pointIdxRadiusSearch, pointRadiusSquaredDistance);
        CameraViewPoint fixed_Fusionfov;
        CameraViewPoint offset_Fusionfov;
        offset_Fusionfov.x=0;offset_Fusionfov.y=0;offset_Fusionfov.z=0;offset_Fusionfov.yaw=0;offset_Fusionfov.pitch=0;
        CameraViewPoint tmp_Fusionfov;
        for (std::vector<int>::iterator it = pointIdxRadiusSearch.begin(); it != pointIdxRadiusSearch.end(); ++it) {
            if (it == pointIdxRadiusSearch.begin())
            {
                fixed_Fusionfov.x = BubblingViewPoints[*it].x;
                fixed_Fusionfov.y = BubblingViewPoints[*it].y;
                fixed_Fusionfov.z = BubblingViewPoints[*it].z;
                fixed_Fusionfov.yaw = BubblingViewPoints[*it].yaw;
                fixed_Fusionfov.pitch = BubblingViewPoints[*it].pitch;
            } else {
                offset_Fusionfov.x += (BubblingViewPoints[*it].x - BubblingViewPoints[*(pointIdxRadiusSearch.begin())].x)*((BubblingViewPoints[*it].Point_in_fov.size())/(BubblingViewPoints[*(pointIdxRadiusSearch.begin())].Point_in_fov.size()));
                offset_Fusionfov.y += (BubblingViewPoints[*it].y - BubblingViewPoints[*(pointIdxRadiusSearch.begin())].y)*((BubblingViewPoints[*it].Point_in_fov.size())/(BubblingViewPoints[*(pointIdxRadiusSearch.begin())].Point_in_fov.size()));
                offset_Fusionfov.z += (BubblingViewPoints[*it].z - BubblingViewPoints[*(pointIdxRadiusSearch.begin())].z)*((BubblingViewPoints[*it].Point_in_fov.size())/(BubblingViewPoints[*(pointIdxRadiusSearch.begin())].Point_in_fov.size()));
                offset_Fusionfov.yaw += (BubblingViewPoints[*it].yaw - BubblingViewPoints[*(pointIdxRadiusSearch.begin())].yaw)*((BubblingViewPoints[*it].Point_in_fov.size())/(BubblingViewPoints[*(pointIdxRadiusSearch.begin())].Point_in_fov.size()));
                offset_Fusionfov.pitch += (BubblingViewPoints[*it].pitch - BubblingViewPoints[*(pointIdxRadiusSearch.begin())].pitch)*((BubblingViewPoints[*it].Point_in_fov.size())/(BubblingViewPoints[*(pointIdxRadiusSearch.begin())].Point_in_fov.size()));
            }
            UsedBubblePointIndices.erase(*it);
            View_cloud->points[*it] = pcl::PointXYZ(); // 将对应索引的点设置为空点
        }

        // std::cout << "pointIdxRadiusSearch:[ ";
        // for (int index : pointIdxRadiusSearch) {
        //         std::cout << index << " ";
        //     }
        // std::cout << "];" <<std::endl;

        tmp_Fusionfov.x = fixed_Fusionfov.x + offset_Fusionfov.x;
        tmp_Fusionfov.y = fixed_Fusionfov.y + offset_Fusionfov.y;
        tmp_Fusionfov.z = fixed_Fusionfov.z + offset_Fusionfov.z;
        tmp_Fusionfov.yaw = fixed_Fusionfov.yaw + offset_Fusionfov.yaw;
        tmp_Fusionfov.pitch = fixed_Fusionfov.pitch + offset_Fusionfov.pitch;

        i += pointIdxRadiusSearch.size();
        FusionViewPoints.push_back(tmp_Fusionfov);
        
        while (BubblingViewPoints[*(UsedBubblePointIndices.begin())].Point_in_fov.empty()) {
            UsedBubblePointIndices.erase(UsedBubblePointIndices.begin());
            i++;
            if (i >= 500)
                break;
        }
    }
}

//主函数
int timefov=0;
void PclPro::Process(void) 
{
    rwLock.readLock();
    *pro_cloud=*Cloud;
    *pro_normals=*Cloud_Normals;
    // *Occupancy_Cloud=*Cloud;
    rwLock.readUnlock();
    Times++;
    // std::cout << "现在的时间是:" <<Times<<std::endl;
    //筛选FOV覆盖视点
    if(Cloud_Size-timefov>100 )
    {
        timefov=Cloud_Size;
        SelectViewpoint(pro_cloud);
        // UpdateViewpoint();
        Show_Fov = 1;
        Send_Fovs = 1;
    }
}
void PclPro::Map_Pro(void) 
{
    //筛选box内的点云
    FilterOutsideBoxes(Lidar_Data);
    //填充内部
    Occupy_Map_Fill();
    //构建栅格占用地图
    Occupy_Map();
    //点云拼接
    rwLock.writeLock();
    *Cloud+=*Lidar_Data;
    rwLock.writeUnlock();
    DownSample(Cloud);
    //计算法向量
    ComputeNormals(Cloud);
    //校正法向量
    BoxCorrectNormals(Cloud); 
}
PclPro::PclPro(ros::NodeHandle& nh) : nh_(nh)
{
    namespace_ = nh_.getNamespace();
    if (nh_.getParam("/scene", scene)) {
        ROS_INFO("Got parameter: %s", scene.c_str());
    } else {
        ROS_ERROR("Failed to get parameter 'scene'");
    }
    std::cout << "****************" << scene << std::endl;
    Yaml_Path = file_path + "/../caric_mission/models/" + scene + "/bounding_boxes/" + "box_description.yaml";
    std::cout << Yaml_Path << std::endl;
    Cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    Cloud_Normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
    pro_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pro_normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
    Lidar_Data = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    Last_Cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    Occupancy_Cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    GetParam();
    Get_BoundingBox();
    Occupy_Map_Init();
}