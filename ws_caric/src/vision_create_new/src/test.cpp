#include "Getfov_class.h"
#include <ros/ros.h>

int main (int argc, char** argv) {
    ros::init(argc, argv, "viewpoint_test");
    // ros::NodeHandlePtr nh;
    ros::NodeHandlePtr nh(new ros::NodeHandle);
    std::cout << "good1" << std::endl;
    Getfov_class test(nh);

    std::cout << "good2" << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/ztd/ws_caric/src/caric_baseline-main/data/jurong_box_0.pcd", *cloud1) == -1) {
        PCL_ERROR("Couldn't read file\n");
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/ztd/ws_caric/src/caric_baseline-main/data/jurong_box_1.pcd", *cloud2) == -1) {
        PCL_ERROR("Couldn't read file\n");
        return -1;
    }

    test.run(cloud1);
    std::vector<Eigen::Vector3d> result1 = test.get_viewpoint();
    test.reset();
    test.run(cloud2);
    std::vector<Eigen::Vector3d> result2 = test.get_viewpoint();

    ros::Rate rate(1);
    while(ros::ok()) {
        // std::vector<Eigen::Vector3d> result = test.get_viewpoint();
        // std::cout << result.size() << std::endl;
        test.pub_result();
        rate.sleep();
    }
     

    return 0;
}