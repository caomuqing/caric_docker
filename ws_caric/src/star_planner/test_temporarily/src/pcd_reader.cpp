#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char** argv) {
  // Initialize the ROS node
  ros::init(argc, argv, "pcd_reader_node");
  ros::NodeHandle nh("~");

  // Create a publisher to publish the PCD point cloud as a ROS message
  ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("caric_interest_point", 10);

  // Load the PCD file
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/zager/workspace/ws_caric_fuel/src/caric_mission/models/mbs/mbs_interest_points_dense.pcd", *cloud) == -1) {
    PCL_ERROR("Couldn't read PCD file\n");
    return -1;
  }

  // Convert the PCL point cloud to a ROS point cloud message
  sensor_msgs::PointCloud2 ros_cloud;
  pcl::toROSMsg(*cloud, ros_cloud);
  ros_cloud.header.frame_id = "world";

  ros::Rate loop_rate(5.0);  

  while (ros::ok()) {
    std::cout << "Publishing a point cloud..." << std::endl;
    cloud_pub.publish(ros_cloud);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
