#include <ros/ros.h>
#include <std_msgs/String.h>
#include <caric_mission/CreatePPComTopic.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
ros::Publisher odom_pub_;
ros::Subscriber odom_sub_, trigger_sub_, gcs_odom_sub_;
caric_mission::CreatePPComTopic srv;
ros::Time last_publish_occ_time_, last_publish_free_time_, last_publish_odom_time_;
bool trigger_flag_ = false;

const vector<string> drone_names_ = {"jurong", "raffles", "sentosa", "changi", "nanyang"};
std::string drone_name_;
ros::ServiceClient create_ppcom_topic;

bool setCreatePPcomInfo(caric_mission::CreatePPComTopic &srv, const std::string &source,
                        const std::vector<std::string> &targets, const std::string &topic_name,
                        const std::string &package_name, const std::string &message_type)
{
    srv.request.source = source;
    srv.request.targets = targets;
    srv.request.topic_name = topic_name;
    srv.request.package_name = package_name;
    srv.request.message_type = message_type;

    if (create_ppcom_topic.call(srv))
    {
        ROS_INFO("Response: %s", srv.response.result.c_str());
        return true;
    }
    else
    {
        ROS_ERROR("Failed to call service create_ppcom_topic");
        return false;
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (trigger_flag_)
        return;
    if ((ros::Time::now() - last_publish_odom_time_).toSec() < 0.01)
        return;

    ROS_INFO_THROTTLE(1.0, "%s send odom", drone_name_.c_str());
    odom_pub_.publish(msg);
    last_publish_odom_time_ = ros::Time::now();
}

void gcsOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (trigger_flag_)
        return;

    ROS_INFO_THROTTLE(1.0, "%s send odom", drone_name_.c_str());
    odom_pub_.publish(msg);
}

void triggerCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    trigger_flag_ = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gcs_talker_node");
    ros::NodeHandle nh;
    ros::NodeHandle node("~");

    // Wait for service to appear
    ros::service::waitForService("create_ppcom_topic");

    node.param("drone_name", drone_name_, std::string("gcs"));

    ROS_WARN("[ppcom_talker_node] drone_name is %s", drone_name_.c_str());

    // Create a service client
    if (drone_name_ == "gcs")
    {
        create_ppcom_topic = nh.serviceClient<caric_mission::CreatePPComTopic>("create_ppcom_topic");
        setCreatePPcomInfo(srv, drone_name_, {"jurong", "raffles", "changi", "nanyang", "sentosa"}, "/odometry", "nav_msgs", "Odometry");
    }

    // Create the pub and sub
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("/odometry", 10);
    odom_sub_ = nh.subscribe("/odom/gcs", 1, odomCallback);

    gcs_odom_sub_ = nh.subscribe("/gcs/ground_truth/odometry", 1, gcsOdomCallback);
    trigger_sub_ = nh.subscribe("/move_base_simple/goal", 1, triggerCallback);

    while (ros::ok())
    {
        ros::spinOnce();

        if (trigger_flag_)
        {
            return 0;
        }

        ros::Duration(0.01).sleep();
    }

    return 0;
}
