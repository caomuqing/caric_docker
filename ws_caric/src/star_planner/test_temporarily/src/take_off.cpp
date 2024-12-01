#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <Eigen/Eigen>

bool odom_flag = false, trigger_flag = false, take_off_flag = false, first_time_flag = false, first_odom_flag = false;
double start_x, start_y, start_z, pos_x, pos_y, pos_z;
ros::Subscriber odom_sub, trigger_sub, take_off_sub;
ros::Publisher trajectory_pub, trigger_pub;
ros::Time take_off_time;

void odom_callback(const nav_msgs::OdometryConstPtr &msg)
{
    odom_flag = true;
    if (!first_odom_flag)
    {
        start_x = msg->pose.pose.position.x;
        start_y = msg->pose.pose.position.y;
        start_z = msg->pose.pose.position.z;
        first_odom_flag = true;
    }
    pos_x = msg->pose.pose.position.x;
    pos_y = msg->pose.pose.position.y;
    pos_z = msg->pose.pose.position.z;
}

void trigger_callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    trigger_flag = true;
}

void takeOffCallBack(const std_msgs::EmptyConstPtr &msg)
{
    take_off_flag = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "take_off_node");
    ros::NodeHandle nh("~");

    double take_off_height_ = 5.0;

    std::string namespace_;
    nh.param<std::string>("namespace_", namespace_, std::string("null"));
    nh.param("take_off_height", take_off_height_, 1.0);

    trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/" + namespace_ + "/command/trajectory", 10);
    trigger_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

    odom_sub = nh.subscribe("/" + namespace_ + "/ground_truth/odometry", 10, odom_callback);
    trigger_sub = nh.subscribe("/move_base_simple/goal", 10, trigger_callback);
    take_off_sub = nh.subscribe("/" + namespace_ + "/take_off", 10, takeOffCallBack);
    ros::Rate rate(10);

    while (ros::ok())
    {
        if (odom_flag && take_off_flag)
        {
            if (!first_time_flag)
            {
                take_off_time = ros::Time::now();
                first_time_flag = true;
                ROS_ERROR("!!!!!!!!!!!TAKE OFF!!!!!!!!!!!!!!!!!!");
            }
            ROS_ERROR("!!!!!!!!!!!TAKE OFF!!!!!!!!!!!!!!!!!!%.2f", (ros::Time::now() - take_off_time).toSec());
            if ((Eigen::Vector3d(pos_x, pos_y, pos_z) - Eigen::Vector3d(start_x, start_y, start_z + take_off_height_)).norm() < 0.1)
            {
                ros::Duration(2.0).sleep();
                trigger_pub.publish(geometry_msgs::PoseStamped());
                return 0;
            }

            if (trigger_flag)
                return 0;

            ROS_INFO("%s Take off!", namespace_.c_str());
            double target_yaw = 0.0;
            std::vector<double> target_pos = {start_x, start_y, start_z + take_off_height_};
            std::vector<double> target_vel = {0.0, 0.0, 0.0};
            std::vector<double> target_acc = {0.0, 0.0, 0.0};

            trajectory_msgs::MultiDOFJointTrajectory trajset_msg;
            trajectory_msgs::MultiDOFJointTrajectoryPoint trajpt_msg;

            geometry_msgs::Transform transform_msg;
            transform_msg.translation.x = target_pos[0];
            transform_msg.translation.y = target_pos[1];
            transform_msg.translation.z = target_pos[2];
            transform_msg.rotation.z = sin(target_yaw * 0.5);
            transform_msg.rotation.w = cos(target_yaw * 0.5);

            trajpt_msg.transforms.push_back(transform_msg);

            geometry_msgs::Twist vel_msg;
            vel_msg.linear.x = target_vel[0];
            vel_msg.linear.y = target_vel[1];
            vel_msg.linear.z = target_vel[2];

            geometry_msgs::Twist accel_msg;
            accel_msg.linear.x = target_acc[0];
            accel_msg.linear.y = target_acc[1];
            accel_msg.linear.z = target_acc[2];

            trajpt_msg.velocities.push_back(vel_msg);
            trajpt_msg.accelerations.push_back(accel_msg);

            trajset_msg.points.push_back(trajpt_msg);

            trajset_msg.header.stamp = ros::Time::now();
            trajectory_pub.publish(trajset_msg);
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
