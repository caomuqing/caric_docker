#ifndef PCL_PRO_H
#define PCL_PRO_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <mutex>
#include <set>
#include <ros/ros.h>
#include <ros/package.h>

class PclPro 
{
    public:
        std::string namespace_;
        ros::NodeHandle& nh_;    
        bool Update_flag = 0;
        bool map_flag = 0;
        pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud;
        pcl::PointCloud<pcl::Normal>::Ptr Cloud_Normals;
        pcl::PointCloud<pcl::PointXYZ>::Ptr pro_cloud;
        pcl::PointCloud<pcl::Normal>::Ptr pro_normals;
        pcl::PointCloud<pcl::PointXYZ>::Ptr Lidar_Data;             //激光雷达数据
        pcl::PointCloud<pcl::PointXYZ>::Ptr Last_Cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr Occupancy_Cloud;
        std::vector<int> Index_Add;           
        std::unique_ptr<std::unique_ptr<std::unique_ptr<bool[]>[]>[]> occupancy_grid_;
        std::unique_ptr<std::unique_ptr<std::unique_ptr<bool[]>[]>[]> new_occupancy_grid_;
        float grid_resolution = 1.5; //param
        int x_cells=0;
        int y_cells=0;
        int z_cells=0;
        struct CameraViewPoint {                                    //视点结构体
            float x;
            float y;
            float z;
            float yaw;
            float pitch;
            float roll;
            Eigen::Vector3f Center_Point;
            int index_in_fov;
            std::vector<int> Point_in_fov = {};
        };
        std::vector<CameraViewPoint> ViewPoints;
        std::vector<CameraViewPoint> FusionViewPoints;
        std::vector<CameraViewPoint> OutViewPoints;
        std::vector<int> tourInd; 
        std::set<int> UsedPointIndices;                             //之前的最大点索引
        std::set<int> UsedBubblePointIndices;
        std::vector<int> SelectedPointIndices;
        struct BoxInfo {                                            //定义boundingbox存储类型
            Eigen::Vector3f center;
            Eigen::Vector3f size;
            Eigen::Matrix4f orientation;
            std::vector<pcl::PointXYZ> center_column;
            int max_side;
        };
        std::vector<BoxInfo> boxInfos;
        std::vector<BoxInfo> boxInfos_s;
        float Box_x=1.0;    //param
        float Box_y=1.0;    //param
        float Box_z=1.0;    //param
        Eigen::Vector3f Bbox_max;
        Eigen::Vector3f Bbox_min;
        int Show_Fov = 0;                 //显示FOVflag
        int Send_Fovs = 0;                //显示FOVflag

        std::string file_path = ros::package::getPath("vision_create_new");   
        // if (ros::package::getPath("your_package", file_path))
        // {     
        // std::string Yaml_Path = file_path + "/config/mbs.yaml"; //yaml 路径 crane mbs plane/home/ztd/ws_caric/src/caric_mission/models/crane
        // std::string Yaml_Path = file_path + "/../caric_mission/models/config/plane.yaml"; //yaml 路径 crane mbs plane
        std::string Yaml_Path;   //yaml 路径 crane mbs plane
        // std::string Yaml_Path = file_path + "/config/crane.yaml"; //yaml 路径 crane mbs plane
        // std::string Param_Path = file_path + "/config/param.yaml"; //yaml 路径 crane mbs plane
        std::string Param_Path = file_path + "/config/";
        std::string Tsp_Path = file_path + "/3rdprty";

    public:
        PclPro(ros::NodeHandle& nh);
        void DownSample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        void FilterOutsideBoxes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud); 
        void ComputeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        void BoxCorrectNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr clouds);
        void SelectViewpoint(pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_datas);
        void CreateViewpoint(pcl::PointCloud<pcl::PointXYZ>::Ptr clouds);
        void IspointOffFov(pcl::PointCloud<pcl::PointXYZ>::Ptr clouds, const pcl::PointXYZ& points, CameraViewPoint& tmp_fovs);
        void Compute_Fov(const pcl::Normal& normal, const pcl::PointXYZ& start_point, CameraViewPoint& tempfovs);
        void UpdateViewpoint(void);
        void Extract_Indics(void);
        void Get_BoundingBox(void);
        void GetParam(void);
        void Process(void);
        void Map_Pro(void);
        void Occupy_Map(void);
        void Occupy_Map_Init(void);
        void Occupy_Map_Fill(void);
        void Tsp_Cal(void);

        // void reset();

    private:
        std::string scene;

        // float Filter_Para = 2;            //点云滤波器参数 param
        // float Lidar_filter = 1;           //lidar filter param
        // int Fov_Size = 0;                 //生成的视点数量
        // float Fov_Dis = 2.0;              //FOV中心点的最小距离

        double Normal_Radius = 4.0;       //点云法向量搜索半径参数 param
        int Times = 0;                    //迭代器的时间
        float Fov_Radius = 3.0;           //视点覆盖半径
        float Fov_P = 4.0;                //视点搜索半径倍率 R=Fov_P*Fov_Radius
        int Cloud_Size = 0;               //生成的点云数量
        int timevp = 0;

        float fov_distance = 1.5;            //FOV的长度
        float Fov_Min_Dis = 1;               //靠近阈值
        double fov_horizontal_deg = 90.0;    // 水平FOV（度）
        double fov_vertical_deg = 60.0;      // 垂直FOV（度）
        float TR_X = std::tan(fov_horizontal_deg / 2.0 * M_PI / 180.0) * fov_distance;
        float TR_Y = std::tan(fov_vertical_deg / 2.0 * M_PI / 180.0) * fov_distance;
        float TR_V = (fov_vertical_deg/2) * M_PI / 180;
        float TR_H = (fov_horizontal_deg/2) * M_PI / 180;
        // std::string Yaml_Path = "/home/ztd/ws_caric/src/vision_create_11_20/config/mbs.yaml"; //yaml 路径 crane mbs plane
        // std::string Param_Path = "/home/ztd/ws_caric/src/vision_create_11_20/config/param.yaml"; //yaml 路径 crane mbs plane
        // std::string Tsp_Path = "/home/ztd/ws_caric/src/vision_create_11_20/3rdprty";
        // std::string file_path = ros::package::getPath("vision_create_new");   
        // // if (ros::package::getPath("your_package", file_path))
        // // {     
        // // std::string Yaml_Path = file_path + "/config/mbs.yaml"; //yaml 路径 crane mbs plane/home/ztd/ws_caric/src/caric_mission/models/crane
        // // std::string Yaml_Path = file_path + "/../caric_mission/models/config/plane.yaml"; //yaml 路径 crane mbs plane
        // std::string Yaml_Path;   //yaml 路径 crane mbs plane
        // // std::string Yaml_Path = file_path + "/config/crane.yaml"; //yaml 路径 crane mbs plane
        // std::string Param_Path = file_path + "/config/param.yaml"; //yaml 路径 crane mbs plane
        // std::string Tsp_Path = file_path + "/3rdprty";
};

//线程锁 防止同时读写
class readWriteLock {
private:
    std::mutex readMtx;
    std::mutex writeMtx;
    int readCnt; // 已加读锁个数
public:
    readWriteLock() : readCnt(0) {}
    void readLock()
    {
        readMtx.lock();
        if (++readCnt == 1) {
            writeMtx.lock();  // 存在线程读操作时，写加锁（只加一次）
        }
        readMtx.unlock();
    }
    void readUnlock()
    {
        readMtx.lock();
        if (--readCnt == 0) { // 没有线程读操作时，释放写锁
            writeMtx.unlock();
        }
        readMtx.unlock();
    }
    void writeLock()
    {
        writeMtx.lock();
    }
    void writeUnlock()
    {
        writeMtx.unlock();
    }
};

#endif