#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <eigen3/Eigen/Eigen>

using namespace std;
using namespace Eigen;

double estimation_rate;

Matrix4d cam2world, cam02body;
Eigen::Quaterniond cam2world_quat;

ros::Timer estimation_timer;

ros::Subscriber odom_sub,servo_odom_sub;
ros::Publisher lidar_pos_pub,servo_pos_pub;
nav_msgs::Odometry _odom;
geometry_msgs::PoseStamped servo_pose;

void rcvOdometryCallbck(const nav_msgs::Odometry &odom)
{
  _odom = odom;
  Matrix4d Pose_receive = Matrix4d::Identity();

  Eigen::Vector3d request_position;
  Eigen::Quaterniond request_pose;
  request_position.x() = odom.pose.pose.position.x;
  request_position.y() = odom.pose.pose.position.y;
  request_position.z() = odom.pose.pose.position.z;
  request_pose.x() = odom.pose.pose.orientation.x;
  request_pose.y() = odom.pose.pose.orientation.y;
  request_pose.z() = odom.pose.pose.orientation.z;
  request_pose.w() = odom.pose.pose.orientation.w;
  Pose_receive.block<3, 3>(0, 0) = request_pose.toRotationMatrix();
  Pose_receive(0, 3) = request_position(0);
  Pose_receive(1, 3) = request_position(1);
  Pose_receive(2, 3) = request_position(2);

  Matrix4d body_pose = Pose_receive;
  // convert to cam pose
  cam2world = body_pose * cam02body;
  cam2world_quat = cam2world.block<3, 3>(0, 0);
}
void rcvServoOdomCallbck(const nav_msgs::Odometry &servo_odom){
  servo_pose.header = servo_odom.header;
  servo_pose.header.frame_id = "world";
  servo_pose.pose = servo_odom.pose.pose;
}
void pubCameraPose(const ros::TimerEvent &event)
{
  geometry_msgs::PoseStamped camera_pose;
  camera_pose.header = _odom.header;
  camera_pose.header.frame_id = "world";
  camera_pose.pose.position.x = cam2world(0, 3);
  camera_pose.pose.position.y = cam2world(1, 3);
  camera_pose.pose.position.z = cam2world(2, 3);
  camera_pose.pose.orientation.w = cam2world_quat.w();
  camera_pose.pose.orientation.x = cam2world_quat.x();
  camera_pose.pose.orientation.y = cam2world_quat.y();
  camera_pose.pose.orientation.z = cam2world_quat.z();
  
  lidar_pos_pub.publish(camera_pose);
  servo_pos_pub.publish(servo_pose);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sensor_pose_trans_node");
  ros::NodeHandle nh("~");

  nh.getParam("estimation_rate", estimation_rate);

  cam02body <<  1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0;

  odom_sub = nh.subscribe("odom", 50, rcvOdometryCallbck);
  servo_odom_sub = nh.subscribe("servo_odom", 50, rcvServoOdomCallbck);

  lidar_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("sensor_pose", 1000);
  servo_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("servo_pose", 1000);


  double estimate_duration = 1.0 / estimation_rate;
  estimation_timer = nh.createTimer(ros::Duration(estimate_duration), pubCameraPose);
  ros::Rate rate(100);
  bool status = ros::ok();

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}