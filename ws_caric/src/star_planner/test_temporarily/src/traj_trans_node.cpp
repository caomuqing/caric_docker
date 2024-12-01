#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>

ros::Publisher trajectory_pub, gimbal_pub;

void positionCommandCallback(const quadrotor_msgs::PositionCommandConstPtr &msg)
{
    // Create a MultiDOFJointTrajectory message
    trajectory_msgs::MultiDOFJointTrajectory multi_dof_traj;
    multi_dof_traj.header = msg->header;

    // Create a MultiDOFJointTrajectoryPoint message
    trajectory_msgs::MultiDOFJointTrajectoryPoint point;

    // Set position
    geometry_msgs::Transform transform;
    geometry_msgs::Vector3 translation;
    translation.x = msg->position.x;
    translation.y = msg->position.y;
    translation.z = msg->position.z;

    // Assign the translation vector to transform
    transform.translation = translation;
    // You may need to set an appropriate quaternion
    transform.rotation = geometry_msgs::Quaternion();
    point.transforms.push_back(transform);

    // Set velocity
    geometry_msgs::Twist velocity;
    velocity.linear = msg->velocity;
    point.velocities.push_back(velocity);

    // Set acceleration
    geometry_msgs::Twist acceleration;
    acceleration.linear = msg->acceleration;
    point.accelerations.push_back(acceleration);

    // Set yaw (you may need to convert yaw to quaternion)
    double yaw = msg->yaw;
    // double yaw = 0.0;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, yaw);
    // quat.setRPY(0, 0, yaw);

    geometry_msgs::Quaternion quaternion_msg;
    quaternion_msg.x = quat.x();
    quaternion_msg.y = quat.y();
    quaternion_msg.z = quat.z();
    quaternion_msg.w = quat.w();
    point.transforms[0].rotation = quaternion_msg;

    // Add the trajectory point to the multi-DOF trajectory
    multi_dof_traj.points.push_back(point);

    // Publish the multi-DOF trajectory
    trajectory_pub.publish(multi_dof_traj);

    // geometry_msgs::Twist gimbal_msg;
    // gimbal_msg.linear.x = 1.0; // setting linear.x to -1.0 enables velocity control mode.
    // gimbal_msg.linear.y = 0.0;  // if linear.x set to 1.0, linear,y and linear.z are the
    // gimbal_msg.linear.z = 0.0;  // target pitch and yaw angle, respectively.
    // gimbal_msg.angular.x = 0.0;
    // gimbal_msg.angular.y = 0.0; // in velocity control mode, this is the target pitch velocity
    // gimbal_msg.angular.z = 0.0;   // in velocity control mode, this is the target yaw velocity
    // gimbal_pub.publish(gimbal_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_trans_node");

    ros::NodeHandle nh("~");
    std::string namespace_;
    nh.param<std::string>("namespace_", namespace_, "");

    // Create a subscriber for the position command topic
    ros::Subscriber position_command_sub = nh.subscribe(
        "/" + namespace_ + "/planning/pos_cmd", 10, positionCommandCallback);

    // Create a publisher for the multi-DOF trajectory
    trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
        "/" + namespace_ + "/command/trajectory", 10);

    gimbal_pub = nh.advertise<geometry_msgs::Twist>(
        "/" + namespace_ + "/command/gimbal", 10);

    ros::spin();

    return 0;
}
