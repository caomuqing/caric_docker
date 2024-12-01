#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include "geometry_msgs/PoseStamped.h" 

main (int argc, char **argv)
{
  ros::init (argc, argv, "pcl_load");  //初始化节点，创建节点名称
  ros::NodeHandle nh;	//节点处理句柄
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1); //发布名称为pcl_out的话题，消息队列长度为1，消息类型为sensor_msgs::PointCloud2
  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::PointCloud2 output;
  pcl::io::loadPCDFile ("/home/nuc/ws_caric/src/vision_create/model/real_pcd.pcd", cloud); //修改自己pcd文件所在路径

  pcl::toROSMsg(cloud, output); //转换为ROS的消息类型
  output.header.frame_id = "odom";//this has been done in order to be able to visualize our PointCloud2 message on the RViz visualizer    
  ros::Rate loop_rate(1);  //控制发布的信息的快慢,即循环内sleep的间隔
  while (ros::ok())
  {
    pcl_pub.publish(output);
    ros::spinOnce();  //监听反馈函数
    loop_rate.sleep();
  }
  return 0;
}
