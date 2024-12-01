#ifndef _MAP_ROS_H
#define _MAP_ROS_H

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <memory>
#include <random>

using std::default_random_engine;
using std::normal_distribution;
using std::shared_ptr;

namespace fast_planner
{
  class SDFMap;

  class MapROS
  {
  public:
    MapROS();
    ~MapROS();
    void setMap(SDFMap *map);
    void init();

  private:
    void depthPoseCallback(const sensor_msgs::ImageConstPtr &img,
                           const geometry_msgs::PoseStampedConstPtr &pose);
    void cloudPoseCallback(const sensor_msgs::PointCloud2ConstPtr &msg,
                           const geometry_msgs::PoseStampedConstPtr &pose);
    void cloudPoseServoCallback(const sensor_msgs::PointCloud2ConstPtr &msg,
                                const geometry_msgs::PoseStampedConstPtr &lidar_pose,
                                const geometry_msgs::PoseStampedConstPtr &servo_pose);
    void swarmMapOccCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void swarmMapFreeCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void slfPoseCallback(const geometry_msgs::PoseStampedConstPtr &pose);
    void updateESDFCallback(const ros::TimerEvent & /*event*/);
    void visCallback(const ros::TimerEvent & /*event*/);
    void swarmMapCallback(const ros::TimerEvent & /*event*/);

    void publishMapAll();
    void publishMapLocal();
    void publishESDF();
    void publishUpdateRange();
    void publishUnknown();
    void publishFree();
    void publishDepth();

    void proessDepthImage();

    SDFMap *map_;
    // may use ExactTime?
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped>
        SyncPolicyImagePose;
    typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                            geometry_msgs::PoseStamped>
        SyncPolicyCloudPose;
    typedef shared_ptr<message_filters::Synchronizer<SyncPolicyCloudPose>> SynchronizerCloudPose;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                            geometry_msgs::PoseStamped, geometry_msgs::PoseStamped>
        SyncPolicyCloudPoseServo;
    typedef shared_ptr<message_filters::Synchronizer<SyncPolicyCloudPoseServo>> SynchronizerCloudPoseServo;

    ros::NodeHandle node_;
    shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
    shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
    shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
    shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> servo_sub_;
    SynchronizerImagePose sync_image_pose_;
    SynchronizerCloudPose sync_cloud_pose_;
    SynchronizerCloudPoseServo sync_cloud_pose_servo_;
    
    ros::Publisher map_local_pub_, map_local_inflate_pub_, esdf_pub_, map_all_pub_, unknown_pub_,
        update_range_pub_, depth_pub_, completion_cloud_pub_, free_pub_;
    ros::Publisher swarm_map_occ_pub_, swarm_map_free_pub_;
    ros::Subscriber swarm_map_occ_sub_, swarm_map_free_sub_, slf_pose_sub_;
    ros::Timer esdf_timer_, vis_timer_, swarm_map_timer_;

    // params, depth projection
    double cx_, cy_, fx_, fy_;
    double depth_filter_maxdist_, depth_filter_mindist_;
    int depth_filter_margin_;
    double k_depth_scaling_factor_;
    int skip_pixel_;
    string frame_id_,drone_name_;
    bool is_inspect_;
    // msg publication
    double esdf_slice_height_;
    double visualization_truncate_height_, visualization_truncate_low_;
    bool show_esdf_time_, show_occ_time_;
    bool show_all_map_;

    // data
    // flags of map state
    bool local_updated_, esdf_need_update_;
    // input
    Eigen::Vector3d camera_pos_, lidar_pos_, slf_pos_;
    Eigen::Quaterniond camera_q_;
    unique_ptr<cv::Mat> depth_image_;
    vector<Eigen::Vector3d> proj_points_;
    int proj_points_cnt;
    double fuse_time_, esdf_time_, max_fuse_time_, max_esdf_time_;
    int fuse_num_, esdf_num_;
    pcl::PointCloud<pcl::PointXYZ> point_cloud_;

    normal_distribution<double> rand_noise_;
    default_random_engine eng_;

    ros::Time map_start_time_;

    friend SDFMap;
  };
}

#endif