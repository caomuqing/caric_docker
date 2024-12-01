#include <ros/ros.h>
#include <std_msgs/String.h>
#include <caric_mission/CreatePPComTopic.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
ros::Publisher map_occ_pub_, map_free_pub_, odom_pub_;
ros::Subscriber map_occ_sub_, map_free_sub_, odom_sub_, trigger_sub_;
caric_mission::CreatePPComTopic srv_map_occ, srv_map_free, srv_to_gcs;
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

void swarmMapOccCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    if ((ros::Time::now() - last_publish_occ_time_).toSec() < 0.1)
        return;

    ROS_INFO_THROTTLE(1.0, "%s send swarmMapOcc", drone_name_.c_str());
    map_occ_pub_.publish(msg);
    last_publish_occ_time_ = ros::Time::now();
}

void swarmMapFreeCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    if ((ros::Time::now() - last_publish_free_time_).toSec() < 0.1)
        return;

    ROS_INFO_THROTTLE(1.0, "%s send swarmMapFree", drone_name_.c_str());
    map_free_pub_.publish(msg);
    last_publish_free_time_ = ros::Time::now();
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (trigger_flag_)
        return;
    if ((ros::Time::now() - last_publish_odom_time_).toSec() < 0.1)
        return;

    ROS_INFO_THROTTLE(1.0, "%s send odom", drone_name_.c_str());
    odom_pub_.publish(msg);
    last_publish_odom_time_ = ros::Time::now();
}

void triggerCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    trigger_flag_ = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ppcom_talker_node");
    ros::NodeHandle nh;
    ros::NodeHandle node("~");

    // Wait for service to appear
    ros::service::waitForService("create_ppcom_topic");

    node.param("drone_name", drone_name_, std::string("null"));
    // node.param("scenario_name",scenario_name_,std::string("null"));

    ROS_WARN("[ppcom_talker_node] drone_name is %s", drone_name_.c_str());

    if (drone_name_ == std::string("jurong"))
        ros::Duration(0.1).sleep();
    else if (drone_name_ == std::string("raffles"))
        ros::Duration(0.2).sleep();
    else if (drone_name_ == std::string("sentosa"))
        ros::Duration(0.3).sleep();
    else if (drone_name_ == std::string("changi"))
        ros::Duration(0.4).sleep();
    else if (drone_name_ == std::string("nanyang"))
        ros::Duration(0.5).sleep();
    else if (drone_name_ == std::string("gcs"))
        ros::Duration(0.05).sleep();
    else
        ROS_ERROR("[ppcom_talker_node] drone_name is wrong!!!");

    // Create a service client
    if (drone_name_ == "jurong")
    {
        create_ppcom_topic = nh.serviceClient<caric_mission::CreatePPComTopic>("create_ppcom_topic");
        setCreatePPcomInfo(srv_map_free, drone_name_, {"changi", "nanyang", "sentosa"}, "/map_free", "sensor_msgs", "PointCloud2");
        setCreatePPcomInfo(srv_map_occ, drone_name_, {"changi", "nanyang", "sentosa"}, "/map_occ", "sensor_msgs", "PointCloud2");
        setCreatePPcomInfo(srv_to_gcs, drone_name_, {"gcs"}, "/odom", "nav_msgs", "Odometry");
    }
    else if (drone_name_ == "raffles")
    {
        create_ppcom_topic = nh.serviceClient<caric_mission::CreatePPComTopic>("create_ppcom_topic");
        setCreatePPcomInfo(srv_map_free, drone_name_, {"changi", "nanyang", "sentosa"}, "/map_free", "sensor_msgs", "PointCloud2");
        setCreatePPcomInfo(srv_map_occ, drone_name_, {"changi", "nanyang", "sentosa"}, "/map_occ", "sensor_msgs", "PointCloud2");
        setCreatePPcomInfo(srv_to_gcs, drone_name_, {"gcs"}, "/odom", "nav_msgs", "Odometry");
    }
    else if (drone_name_ == "changi")
    {
        create_ppcom_topic = nh.serviceClient<caric_mission::CreatePPComTopic>("create_ppcom_topic");
        setCreatePPcomInfo(srv_to_gcs, drone_name_, {"gcs"}, "/odom", "nav_msgs", "Odometry");
    }
    else if (drone_name_ == "nanyang")
    {
        create_ppcom_topic = nh.serviceClient<caric_mission::CreatePPComTopic>("create_ppcom_topic");
        setCreatePPcomInfo(srv_to_gcs, drone_name_, {"gcs"}, "/odom", "nav_msgs", "Odometry");
    }
    else if (drone_name_ == "sentosa")
    {
        create_ppcom_topic = nh.serviceClient<caric_mission::CreatePPComTopic>("create_ppcom_topic");
        setCreatePPcomInfo(srv_to_gcs, drone_name_, {"gcs"}, "/odom", "nav_msgs", "Odometry");
    }
    else if (drone_name_ == "gcs")
    {
        create_ppcom_topic = nh.serviceClient<caric_mission::CreatePPComTopic>("create_ppcom_topic");
        setCreatePPcomInfo(srv_to_gcs, drone_name_, {"jurong", "raffles", "changi", "nanyang", "sentosa"}, "/odometry", "nav_msgs", "Odometry");
    }

    // Create the pub and sub
    map_occ_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/map_occ", 10);
    map_occ_sub_ = nh.subscribe("/" + drone_name_ + "/sdf_map/occupancy_all", 1, swarmMapOccCallback);
    map_free_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/map_free", 10);
    map_free_sub_ = nh.subscribe("/" + drone_name_ + "/sdf_map/free", 1, swarmMapFreeCallback);

    odom_pub_ = nh.advertise<nav_msgs::Odometry>("/odom", 10);
    odom_sub_ = nh.subscribe("/" + drone_name_ + "/ground_truth/odometry", 1, odomCallback);
    trigger_sub_ = nh.subscribe("/move_base_simple/goal", 1, triggerCallback);

    while (ros::ok())
    {
        ros::spinOnce();

        if (trigger_flag_ && (drone_name_ == "changi" || drone_name_ == "nanyang" || drone_name_ == "sentosa"))
        {
            return 0;
        }

        ros::Duration(0.01).sleep();
    }

    return 0;
}
