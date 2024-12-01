#include <plan_env/sdf_map.h>
#include <plan_env/map_ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

#include <fstream>

namespace fast_planner
{
  MapROS::MapROS()
  {
  }

  MapROS::~MapROS()
  {
  }

  void MapROS::setMap(SDFMap *map)
  {
    this->map_ = map;
  }

  void MapROS::init()
  {
    node_.param("map_ros/fx", fx_, -1.0);
    node_.param("map_ros/fy", fy_, -1.0);
    node_.param("map_ros/cx", cx_, -1.0);
    node_.param("map_ros/cy", cy_, -1.0);
    node_.param("map_ros/depth_filter_maxdist", depth_filter_maxdist_, -1.0);
    node_.param("map_ros/depth_filter_mindist", depth_filter_mindist_, -1.0);
    node_.param("map_ros/depth_filter_margin", depth_filter_margin_, -1);
    node_.param("map_ros/k_depth_scaling_factor", k_depth_scaling_factor_, -1.0);
    node_.param("map_ros/skip_pixel", skip_pixel_, -1);

    node_.param("map_ros/esdf_slice_height", esdf_slice_height_, -0.1);
    node_.param("map_ros/visualization_truncate_height", visualization_truncate_height_, -0.1);
    node_.param("map_ros/visualization_truncate_low", visualization_truncate_low_, -0.1);
    node_.param("map_ros/show_occ_time", show_occ_time_, false);
    node_.param("map_ros/show_esdf_time", show_esdf_time_, false);
    node_.param("map_ros/show_all_map", show_all_map_, false);
    node_.param("map_ros/frame_id", frame_id_, string("world"));

    node_.param("exploration/is_inspect", is_inspect_, false);
    node_.param("exploration/drone_name", drone_name_, string("null"));

    proj_points_.resize(640 * 480 / (skip_pixel_ * skip_pixel_));
    point_cloud_.points.resize(640 * 480 / (skip_pixel_ * skip_pixel_));

    proj_points_cnt = 0;

    local_updated_ = false;
    esdf_need_update_ = false;
    fuse_time_ = 0.0;
    esdf_time_ = 0.0;
    max_fuse_time_ = 0.0;
    max_esdf_time_ = 0.0;
    fuse_num_ = 0;
    esdf_num_ = 0;
    depth_image_.reset(new cv::Mat);

    rand_noise_ = normal_distribution<double>(0, 0.1);
    random_device rd;
    eng_ = default_random_engine(rd());

    esdf_timer_ = node_.createTimer(ros::Duration(0.05), &MapROS::updateESDFCallback, this);
    swarm_map_timer_ = node_.createTimer(ros::Duration(0.1), &MapROS::swarmMapCallback, this);

    map_all_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_all", 10);
    // map_local_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_local", 10);
    // map_local_inflate_pub_ =
    //     node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_local_inflate", 10);
    // unknown_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/unknown", 10);
    free_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/free", 10);
    // esdf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/esdf", 10);
    // update_range_pub_ = node_.advertise<visualization_msgs::Marker>("/sdf_map/update_range", 10);
    // depth_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/depth_cloud", 10);
    // completion_cloud_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/completion_cloud", 10);

    if (!is_inspect_)
    {
      cloud_sub_.reset(
          new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_, "/map_ros/cloud", 50));
      pose_sub_.reset(
          new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "/map_ros/pose", 25));
      servo_sub_.reset(
          new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "/map_ros/servo_pose", 25));

      sync_cloud_pose_servo_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyCloudPoseServo>(
          MapROS::SyncPolicyCloudPoseServo(100), *cloud_sub_, *pose_sub_, *servo_sub_));
      sync_cloud_pose_servo_->registerCallback(
          boost::bind(&MapROS::cloudPoseServoCallback, this, _1, _2, _3));

      // Explorer pub map
      // swarm_map_occ_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/swarm_map/occupied", 10);
      // swarm_map_free_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/swarm_map/free", 10);
    }
    else
    {
      // Inspecter sub map
      slf_pose_sub_ = node_.subscribe("/map_ros/pose", 10, &MapROS::slfPoseCallback, this);
      swarm_map_occ_sub_ = node_.subscribe("/map_occ/" + drone_name_, 10, &MapROS::swarmMapOccCallback, this);
      swarm_map_free_sub_ = node_.subscribe("/map_free/" + drone_name_, 10, &MapROS::swarmMapFreeCallback, this);
    }

    map_start_time_ = ros::Time::now();
  }

  void MapROS::swarmMapCallback(const ros::TimerEvent &e)
  {
    // Only pub occ and free for inspect (alse for visualization)
    publishMapAll();
    publishFree();
  }

  void MapROS::updateESDFCallback(const ros::TimerEvent & /*event*/)
  {
    if (!esdf_need_update_)
      return;
    auto t1 = ros::Time::now();

    map_->updateESDF3d();
    esdf_need_update_ = false;

    auto t2 = ros::Time::now();
    esdf_time_ += (t2 - t1).toSec();
    max_esdf_time_ = max(max_esdf_time_, (t2 - t1).toSec());
    esdf_num_++;
    if (show_esdf_time_)
      ROS_WARN("ESDF t: cur: %lf, avg: %lf, max: %lf", (t2 - t1).toSec(), esdf_time_ / esdf_num_,
               max_esdf_time_);
  }

  void MapROS::cloudPoseServoCallback(const sensor_msgs::PointCloud2ConstPtr &msg,
                                      const geometry_msgs::PoseStampedConstPtr &lidar_pose,
                                      const geometry_msgs::PoseStampedConstPtr &servo_pose)
  {
    lidar_pos_(0) = servo_pose->pose.position.x;
    lidar_pos_(1) = servo_pose->pose.position.y;
    lidar_pos_(2) = servo_pose->pose.position.z;

    pcl::PointCloud<pcl::PointXYZ> cloud, completion_cloud;

    // Convert sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
    pcl::fromROSMsg(*msg, cloud);

    // ROS_WARN("before completion -> cloudPoints num: %d", num);

    // Get transformation of sensor wrt world
    tf::Transform tf_world_sensor;
    tf::poseMsgToTF(servo_pose->pose, tf_world_sensor);
    tf::Transform tf_sensor_world = tf_world_sensor.inverse();

    int hfov_min_yaw = -180;
    int hfov_max_yaw = 180;
    int hfov_num = hfov_max_yaw - hfov_min_yaw;
    int vfov_min_pitch = -15;
    int vfov_max_pitch = 15;
    int vfov_num = vfov_max_pitch - vfov_min_pitch;
    // Create a bool vector to record if a point at particular yaw-pitch combination exists
    std::vector<std::vector<bool>> return_map(hfov_num, std::vector<bool>(vfov_num, false));

    float max_laser_dist = 50.0;

    // Process each point in the point cloud
    for (pcl::PointXYZ &point : cloud.points)
    {
      // Transform point
      tf::Vector3 local_vector(point.x, point.y, point.z);
      tf::Vector3 world_vector = tf_sensor_world * local_vector;

      // Get the yaw and pitch of the point
      double yaw = atan2(world_vector.y(), world_vector.x());
      double pitch = asin(world_vector.z() / world_vector.length());

      // Convert angles to degree and shift the pitch range to [0, MAX]
      int yaw_deg = (int)(yaw * 180 / M_PI - hfov_min_yaw + 0.5);
      int pitch_deg = (int)(pitch * 180 / M_PI - vfov_min_pitch + 0.5);

      // Make sure the calculated indices are in the range
      if (yaw_deg >= 0 && yaw_deg < hfov_num && pitch_deg >= 0 && pitch_deg < vfov_num)
      {
        return_map[yaw_deg][pitch_deg] = true;
      }
    }

    // Process the return map
    for (int yaw_deg = 0; yaw_deg < hfov_num; yaw_deg += 2)
    {
      for (int pitch_deg = 0; pitch_deg < vfov_num; pitch_deg += 2)
      {
        // the (yaw,pitch) around have returned point
        bool flag_around_return = false;
        for (int i = -6; i <= 6; i++)
        {
          if (flag_around_return)
            break;
          for (int j = -4; j <= 4; j++)
          {
            if (yaw_deg + i >= 0 && yaw_deg + i < hfov_num && pitch_deg + j >= 0 && pitch_deg + j < vfov_num)
            {
              if (return_map[yaw_deg + i][pitch_deg + j])
              {
                flag_around_return = true;
                break;
              }
            }
          }
        }

        if (flag_around_return)
          continue;

        // There is no return at this point. Create a point at maximum distance in this direction
        pcl::PointXYZ far_point;
        double yaw = (yaw_deg + hfov_min_yaw) / 180.0 * M_PI;
        double pitch = (pitch_deg + vfov_min_pitch) / 180.0 * M_PI;
        far_point.x = cos(pitch) * cos(yaw) * max_laser_dist;
        far_point.y = cos(pitch) * sin(yaw) * max_laser_dist;
        far_point.z = sin(pitch) * max_laser_dist;

        tf::Vector3 local_vector(far_point.x, far_point.y, far_point.z);
        tf::Vector3 world_vector = tf_world_sensor * local_vector;

        far_point.x = world_vector.x();
        far_point.y = world_vector.y();
        far_point.z = world_vector.z();
        cloud.points.push_back(far_point);
        // completion_cloud.points.push_back(far_point);
      }
    }

    // sensor_msgs::PointCloud2 output_cloud_msg;
    // completion_cloud.width = completion_cloud.points.size();
    // completion_cloud.height = 1;
    // completion_cloud.is_dense = true;
    // completion_cloud.header.frame_id = frame_id_;
    // pcl::toROSMsg(completion_cloud, output_cloud_msg);
    // completion_cloud_pub_.publish(output_cloud_msg);

    int num = cloud.points.size();

    // ROS_WARN("after completion -> cloudPoints num: %d", num);

    map_->inputPointCloud(cloud, num, lidar_pos_);

    if (local_updated_)
    {
      map_->clearAndInflateLocalMap();
      esdf_need_update_ = true;
      local_updated_ = false;
    }
  }

  void MapROS::swarmMapOccCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    // explorater don't need this
    // ROS_ERROR("swarmMapOccCallback");
    if (!is_inspect_)
      return;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    int num = cloud.points.size();
    map_->inputMap(cloud, num, true, slf_pos_);

    if (local_updated_)
    {
      map_->clearAndInflateLocalMap();
      esdf_need_update_ = true;
      local_updated_ = false;
    }
  }

  void MapROS::swarmMapFreeCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    // explorater don't need this
    if (!is_inspect_)
      return;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // Add slf pose to set free
    pcl::PointXYZ new_point(slf_pos_(0), slf_pos_(1), slf_pos_(2));

    cloud.push_back(new_point);

    int num = cloud.points.size();
    map_->inputMap(cloud, num, false, slf_pos_);

    if (local_updated_)
    {
      map_->clearAndInflateLocalMap();
      esdf_need_update_ = true;
      local_updated_ = false;
    }
  }

  void MapROS::slfPoseCallback(const geometry_msgs::PoseStampedConstPtr &pose)
  {
    if (!is_inspect_)
      return;

    slf_pos_(0) = pose->pose.position.x;
    slf_pos_(1) = pose->pose.position.y;
    slf_pos_(2) = pose->pose.position.z;
    
    map_->updateSlfPose(slf_pos_);
  }

  void MapROS::proessDepthImage()
  {
    proj_points_cnt = 0;

    uint16_t *row_ptr;
    int cols = depth_image_->cols;
    int rows = depth_image_->rows;
    double depth;
    Eigen::Matrix3d camera_r = camera_q_.toRotationMatrix();
    Eigen::Vector3d pt_cur, pt_world;
    const double inv_factor = 1.0 / k_depth_scaling_factor_;

    for (int v = depth_filter_margin_; v < rows - depth_filter_margin_; v += skip_pixel_)
    {
      row_ptr = depth_image_->ptr<uint16_t>(v) + depth_filter_margin_;
      for (int u = depth_filter_margin_; u < cols - depth_filter_margin_; u += skip_pixel_)
      {
        depth = (*row_ptr) * inv_factor;
        row_ptr = row_ptr + skip_pixel_;

        // // filter depth
        // if (depth > 0.01)
        //   depth += rand_noise_(eng_);

        // TODO: simplify the logic here
        if (*row_ptr == 0 || depth > depth_filter_maxdist_)
          depth = depth_filter_maxdist_;
        else if (depth < depth_filter_mindist_)
          continue;

        pt_cur(0) = (u - cx_) * depth / fx_;
        pt_cur(1) = (v - cy_) * depth / fy_;
        pt_cur(2) = depth;
        pt_world = camera_r * pt_cur + camera_pos_;
        auto &pt = point_cloud_.points[proj_points_cnt++];
        pt.x = pt_world[0];
        pt.y = pt_world[1];
        pt.z = pt_world[2];
      }
    }

    publishDepth();
  }

  void MapROS::publishMapAll()
  {
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
    Eigen::Vector3i min_idx, max_idx;

    map_->posToIndex(map_->md_->all_min_, min_idx);
    map_->posToIndex(map_->md_->all_max_, max_idx);

    map_->boundIndex(min_idx);
    map_->boundIndex(max_idx);

    for (int x = min_idx[0]; x <= max_idx[0]; ++x)
      for (int y = min_idx[1]; y <= max_idx[1]; ++y)
        for (int z = min_idx[2]; z <= max_idx[2]; ++z)
        {
          if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] >
              map_->mp_->min_occupancy_log_)
          {
            Eigen::Vector3d pos;
            map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
            if (pos(2) > visualization_truncate_height_)
              continue;
            if (pos(2) < visualization_truncate_low_)
              continue;
            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);
            cloud1.push_back(pt);
          }
        }
    cloud1.width = cloud1.points.size();
    cloud1.height = 1;
    cloud1.is_dense = true;
    cloud1.header.frame_id = frame_id_;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud1, cloud_msg);
    map_all_pub_.publish(cloud_msg);
    // if (!is_inspect_)
    //   swarm_map_occ_pub_.publish(cloud_msg);
  }

  void MapROS::publishMapLocal()
  {
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud2;
    Eigen::Vector3i min_cut = map_->md_->local_bound_min_;
    Eigen::Vector3i max_cut = map_->md_->local_bound_max_;
    map_->boundIndex(min_cut);
    map_->boundIndex(max_cut);

    // for (int z = min_cut(2); z <= max_cut(2); ++z)
    for (int x = min_cut(0); x <= max_cut(0); ++x)
      for (int y = min_cut(1); y <= max_cut(1); ++y)
        for (int z = map_->mp_->box_min_(2); z < map_->mp_->box_max_(2); ++z)
        {
          if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->min_occupancy_log_)
          {
            // Occupied cells
            Eigen::Vector3d pos;
            map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
            if (pos(2) > visualization_truncate_height_)
              continue;
            if (pos(2) < visualization_truncate_low_)
              continue;

            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);
            cloud.push_back(pt);
          }
          // else if (map_->md_->occupancy_buffer_inflate_[map_->toAddress(x, y, z)] == 1)
          // {
          //   // Inflated occupied cells
          //   Eigen::Vector3d pos;
          //   map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          //   if (pos(2) > visualization_truncate_height_)
          //     continue;
          //   if (pos(2) < visualization_truncate_low_)
          //     continue;

          //   pt.x = pos(0);
          //   pt.y = pos(1);
          //   pt.z = pos(2);
          //   cloud2.push_back(pt);
          // }
        }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = frame_id_;
    cloud2.width = cloud2.points.size();
    cloud2.height = 1;
    cloud2.is_dense = true;
    cloud2.header.frame_id = frame_id_;
    sensor_msgs::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud, cloud_msg);
    map_local_pub_.publish(cloud_msg);
    pcl::toROSMsg(cloud2, cloud_msg);
    map_local_inflate_pub_.publish(cloud_msg);
  }

  void MapROS::publishUnknown()
  {
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    Eigen::Vector3i min_idx, max_idx;

    map_->posToIndex(map_->md_->all_min_, min_idx);
    map_->posToIndex(map_->md_->all_max_, max_idx);

    map_->boundIndex(min_idx);
    map_->boundIndex(max_idx);

    for (int x = min_idx[0]; x <= max_idx[0]; ++x)
      for (int y = min_idx[1]; y <= max_idx[1]; ++y)
        for (int z = min_idx[2]; z <= max_idx[2]; ++z)
        {
          if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] < map_->mp_->clamp_min_log_ - 1e-3)
          {
            Eigen::Vector3d pos;
            map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
            if (pos(2) > visualization_truncate_height_)
              continue;
            if (pos(2) < visualization_truncate_low_)
              continue;
            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);
            cloud.push_back(pt);
          }
        }
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = frame_id_;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    unknown_pub_.publish(cloud_msg);
  }

  void MapROS::publishFree()
  {
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    Eigen::Vector3i min_idx, max_idx;

    map_->posToIndex(map_->md_->all_min_, min_idx);
    map_->posToIndex(map_->md_->all_max_, max_idx);

    map_->boundIndex(min_idx);
    map_->boundIndex(max_idx);

    for (int x = min_idx[0]; x <= max_idx[0]; ++x)
      for (int y = min_idx[1]; y <= max_idx[1]; ++y)
        for (int z = min_idx[2]; z <= max_idx[2]; ++z)
        {
          if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] < map_->mp_->clamp_min_log_ - 1e-3)
            continue;
          if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->min_occupancy_log_)
            continue;
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_height_)
            continue;
          if (pos(2) < visualization_truncate_low_)
            continue;
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = frame_id_;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    free_pub_.publish(cloud_msg);

    // if (!is_inspect_)
    //   swarm_map_free_pub_.publish(cloud_msg);
  }

  void MapROS::publishDepth()
  {
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (int i = 0; i < proj_points_cnt; ++i)
    {
      cloud.push_back(point_cloud_.points[i]);
    }
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = frame_id_;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    depth_pub_.publish(cloud_msg);
  }

  void MapROS::publishUpdateRange()
  {
    Eigen::Vector3d esdf_min_pos, esdf_max_pos, cube_pos, cube_scale;
    visualization_msgs::Marker mk;
    map_->indexToPos(map_->md_->local_bound_min_, esdf_min_pos);
    map_->indexToPos(map_->md_->local_bound_max_, esdf_max_pos);

    cube_pos = 0.5 * (esdf_min_pos + esdf_max_pos);
    cube_scale = esdf_max_pos - esdf_min_pos;
    mk.header.frame_id = frame_id_;
    mk.header.stamp = ros::Time::now();
    mk.type = visualization_msgs::Marker::CUBE;
    mk.action = visualization_msgs::Marker::ADD;
    mk.id = 0;
    mk.pose.position.x = cube_pos(0);
    mk.pose.position.y = cube_pos(1);
    mk.pose.position.z = cube_pos(2);
    mk.scale.x = cube_scale(0);
    mk.scale.y = cube_scale(1);
    mk.scale.z = cube_scale(2);
    mk.color.a = 0.3;
    mk.color.r = 1.0;
    mk.color.g = 0.0;
    mk.color.b = 0.0;
    mk.pose.orientation.w = 1.0;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;

    update_range_pub_.publish(mk);
  }

  void MapROS::publishESDF()
  {
    double dist;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::PointXYZI pt;

    const double min_dist = 0.0;
    const double max_dist = 3.0;

    Eigen::Vector3i min_cut = map_->md_->local_bound_min_ - Eigen::Vector3i(map_->mp_->local_map_margin_,
                                                                            map_->mp_->local_map_margin_,
                                                                            map_->mp_->local_map_margin_);
    Eigen::Vector3i max_cut = map_->md_->local_bound_max_ + Eigen::Vector3i(map_->mp_->local_map_margin_,
                                                                            map_->mp_->local_map_margin_,
                                                                            map_->mp_->local_map_margin_);
    map_->boundIndex(min_cut);
    map_->boundIndex(max_cut);

    for (int x = min_cut(0); x <= max_cut(0); ++x)
      for (int y = min_cut(1); y <= max_cut(1); ++y)
      {
        Eigen::Vector3d pos;
        map_->indexToPos(Eigen::Vector3i(x, y, 1), pos);
        pos(2) = esdf_slice_height_;
        dist = map_->getDistance(pos);
        dist = min(dist, max_dist);
        dist = max(dist, min_dist);
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = -0.2;
        pt.intensity = (dist - min_dist) / (max_dist - min_dist);
        cloud.push_back(pt);
      }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = frame_id_;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);

    esdf_pub_.publish(cloud_msg);

    // ROS_INFO("pub esdf");
  }
}