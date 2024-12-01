/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// MODULE HEADER
#include "rotors_gazebo_plugins/gazebo_ppcom_plugin.h"

// SYSTEM LIBS
#include <stdio.h>
#include <boost/bind.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <fstream>
#include <thread>
#include <algorithm>

#include "std_msgs/ColorRGBA.h"
#include "visualization_msgs/Marker.h"
#include "rotors_comm/PPComTopology.h"

// Physics
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/World.hh>

// 3RD PARTY
#include "mav_msgs/default_topics.h"

// // USER HEADERS
// #include "ConnectGazeboToRosTopic.pb.h"

#define RAD2DEG(x) (x * 180.0 / M_PI)
#define DEG2RAD(x) (x * M_PI / 180.0)

using namespace std;

namespace gazebo
{
    // Constructors
    IndexedInterestPoint::IndexedInterestPoint()
    {
        detected_order = -1;
        scored_point.x = 0;
        scored_point.y = 0;
        scored_point.z = 0;
        scored_point.intensity = 0;
        scored_point.normal_x  = 0;
        scored_point.normal_y  = 0;
        scored_point.normal_z  = 0;
        scored_point.curvature = 0;
    }
    IndexedInterestPoint::IndexedInterestPoint(const int &detected_order_, const PointXYZIN &scored_point_):
        detected_order(detected_order_), scored_point(scored_point_) {}
    // Simple destructor
    IndexedInterestPoint::~IndexedInterestPoint() {};    

    // PPCom class, helper of the plugin
    PPComNode::PPComNode()
    {
        // Set cov diagonal to -1 to indicate no reception yet
        for (auto &c : this->odom_msg.pose.covariance)
            c = -1;
    }

    PPComNode::PPComNode(const string &name_, const string &role_, const double &offset_)
        : name(name_), role(role_), offset(offset_)
    {
        // Set cov diagonal to -1 to indicate no reception yet
        for (auto &c : this->odom_msg.pose.covariance)
            c = -1;
    }

    PPComNode::PPComNode(const string &name_, const string &role_, const double &offset_,
                         const double &hfov, const double &vfov, const double &cam_x,
                         const double &cam_y, const double &cam_z, const double &exposure,
                         const double &trig_interval, const double &focal_length_,
                         const double &pixel_size_, const double &desired_mm_per_pixel_,
                         const double &gimbal_pitch_max_, const double &gimbal_yaw_max_,
                         const double &gimbal_rate_max_)
        : name(name_), role(role_), offset(offset_), fov_h(hfov), fov_v(vfov), 
          exposure(exposure), capture_interval(trig_interval), focal_length(focal_length_),
          pixel_size(pixel_size_), desired_mm_per_pixel(desired_mm_per_pixel_),
          gimbal_pitch_max(gimbal_pitch_max_*0.01745329251), 
          gimbal_yaw_max(gimbal_yaw_max_*0.01745329251),
          gimbal_rate_max(gimbal_rate_max_*0.01745329251)
    {
        // Set cov diagonal this to -1 to indicate no reception yet
        for (auto &c : this->odom_msg.pose.covariance)
            c = -1;

        topology_mtx = boost::shared_ptr<std::mutex>(new std::mutex());

        Cam_rel_Uav = Vector3d(cam_x, cam_y, cam_z);
        double l = focal_length/pixel_size * 0.001 * desired_mm_per_pixel;
        double side_h = l / cos(hfov*0.00872664625); // 1/2/180*pi
        double side_v = l * tan(vfov*0.00872664625);
        visible_radius = sqrt(side_h*side_h + side_v*side_v) + 5.0;
    }

    PPComNode::~PPComNode() {}

    Eigen::MatrixXd PPComNode::GetTopology()
    {
       std::lock_guard<std::mutex> lock(*topology_mtx);
       return topology;
    }

    void PPComNode::SetTopology(Eigen::MatrixXd topology_)
    {
       std::lock_guard<std::mutex> lock(*topology_mtx);
       topology = topology_;
    }

    // Plugin definition
    GazeboPPComPlugin::GazeboPPComPlugin()
        : ModelPlugin(), gz_node_handle_(0) {}

    GazeboPPComPlugin::~GazeboPPComPlugin() {}

    void GazeboPPComPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        if (kPrintOnPluginLoad)
            gzdbg << __FUNCTION__ << "() called." << endl;

        gzdbg << "_model = " << _model->GetName() << endl;

        // Store the pointer to the model, world, and physics
        model_ = _model;
        world_ = model_->GetWorld();
        physics_ = world_->Physics();

        physics_->InitForThread();

        // default params
        namespace_.clear();

        // // Create a rayshape object for communication links
        // ray_topo_ = boost::dynamic_pointer_cast<gazebo::physics::RayShape>
        //                     (physics_->CreateShape("ray", gazebo::physics::CollisionPtr()));

        // // Create rayshape object for camera field of view check
        // ray_inpo_ = boost::dynamic_pointer_cast<gazebo::physics::RayShape>
        //                     (physics_->CreateShape("ray", gazebo::physics::CollisionPtr()));

        //==============================================//
        //========== READ IN PARAMS FROM SDF ===========//
        //==============================================//

        if (_sdf->HasElement("robotNamespace"))
            namespace_ = _sdf->GetElement("robotNamespace")->Get<string>();
        else
            gzerr << "[gazebo_ppcom_plugin] Please specify a robotNamespace.\n";

        if (_sdf->HasElement("linkName"))
            self_link_name_ = _sdf->GetElement("linkName")->Get<string>();
        else
            gzerr << "[gazebo_ppcom_plugin] Please specify a linkName.\n";

        if (_sdf->HasElement("ppcomId"))
            ppcom_id_ = _sdf->GetElement("ppcomId")->Get<string>();
        else
            gzerr << "[gazebo_ppcom_plugin] Please specify a ppcomId.\n";

        if (_sdf->HasElement("ppcomConfig"))
            ppcom_config_ = _sdf->GetElement("ppcomConfig")->Get<string>();
        else
            gzerr << "[gazebo_ppcom_plugin] Please specify ppcomConfig.\n";

        if (_sdf->HasElement("ppcomHz"))
            ppcom_hz_ = _sdf->GetElement("ppcomHz")->Get<double>();
        else
            gzerr << "[gazebo_ppcom_plugin] Please specify a ppcomId.\n";

        std::string fstring_;
        if (_sdf->HasElement("interestPcd"))
            fstring_ = _sdf->GetElement("interestPcd")->Get<string>();
        else
            gzerr << "[gazebo_caric_plugin] Please specify input interest points.\n";

        // Get the ppcom topic where data is published to
        getSdfParam<string>(_sdf, "ppcomTopic", ppcom_topic_, "ppcom");

        // Report on params obtained from sdf
        printf(KGRN "PPCom Id %s is set. Linkname %s. Config %s!\n" RESET,
               ppcom_id_.c_str(), self_link_name_.c_str(), ppcom_config_.c_str());

        // Open the config file and read the links
        std::ifstream ppcom_config_file(ppcom_config_.c_str());

        readPCloud(fstring_);

        // Read the declared nodes
        ppcom_nodes_.clear();
        string line;
        while (getline(ppcom_config_file, line))
        {
            // Process the line here
            cout << "Reading " << line << endl;

            // Remove spaces in the line
            line.erase(std::remove_if(line.begin(), line.end(), ::isspace), line.end());

            // Skip the line of its commented out
            if (line[0] == '#')
                continue;

            // Decode the line and construct the node
            vector<string> parts = Util::split(line, ",");
            ppcom_nodes_.push_back(PPComNode(parts[0], parts[1], stod(parts[2]), stod(parts[3]),
                                             stod(parts[4]), stod(parts[5]), stod(parts[6]), 
                                             stod(parts[7]), stod(parts[8]), stod(parts[9]),
                                             stod(parts[10]), stod(parts[11]), stod(parts[12]),
                                             stod(parts[13]), stod(parts[14]), stod(parts[15])));
        }

        // Assert that ppcom_id_ is found in the network
        bool ppcom_id_in_network = false;
        for (int idx = 0; idx < ppcom_nodes_.size(); idx++)
            if (ppcom_id_ == ppcom_nodes_[idx].name)
            {
                ppcom_id_in_network = true;
                ppcom_slf_idx_ = idx;
                break;
            }

        assert(ppcom_id_in_network);

        // Number of nodes
        Nnodes_ = ppcom_nodes_.size();

        //==============================================//
        //========== CREATE THE TRANSPORT STRUCTURES ===//
        //==============================================//

        // Create a gazebo node handle and initialize with the namespace
        gz_node_handle_ = transport::NodePtr(new transport::Node());
        gz_node_handle_->Init();

        // Create a ros node
        ros_node_handle_ = new ros::NodeHandle("/firefly" + ppcom_id_ + "rosnode");

        // Subscribe to the odom topics
        int node_idx = -1;
        for (PPComNode &node : ppcom_nodes_)
        {
            node_idx++;

            // Create the subscriber to each nodes
            node.odom_sub = ros_node_handle_->subscribe<nav_msgs::Odometry>("/" + node.name + "/ground_truth/odometry", 1,
                                                                            boost::bind(&GazeboPPComPlugin::OdomCallback, this, _1, node_idx));

            node.trigger_sub = ros_node_handle_->subscribe<rotors_comm::BoolStamped>("/" + node.name + "/command/trigger", 1,
                                                                            boost::bind(&GazeboPPComPlugin::triggerCallback, this, _1, node_idx));
            // Publisher for the topology
            if (node.role == "manager")
                node.topo_pub = ros_node_handle_->advertise<rotors_comm::PPComTopology>("/ppcom_topology", 1);

            // Publisher for camera pyramid visuals
            node.camera_pyramid_pub = ros_node_handle_->advertise<visualization_msgs::Marker>(
                "/" + node.name + "/visualize", 1);

            // Create the storage of nodes to each object
            node.odom_msg_received = false;

            // Knowlege on the topology
            node.SetTopology(-1 * MatrixXd::Ones(Nnodes_, Nnodes_));

            // Create a rayshape object for communication link
            node.ray_topo = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(physics_->CreateShape("ray", gazebo::physics::CollisionPtr()));

            // Create rayshape object for camera field of view check
            node.ray_inpo = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(physics_->CreateShape("ray", gazebo::physics::CollisionPtr()));

            // Create a pointcloud of detected points
            node.InPoLog = {};

            node.CloudDetectedInpoPub = ros_node_handle_->advertise<sensor_msgs::PointCloud2>("/" + node.name + "/detected_interest_points", 1);

            // Preload the interest points for manager
            if (node.role == "manager")
            {

                for(int i = 0; i < cloud_inpo_->size(); i++)
                {
                    node.InPoLog[i] = IndexedInterestPoint(i, cloud_inpo_->points[i]);
                    node.InPoLog[i].scored_point.intensity = 0;
                    assert(node.InPoLog[i].scored_point.curvature == i);
                }
            }

            // GCS no need gimbal
            if (node.name == "gcs")
                continue;

            node.timer_update = ros_node_handle_->createTimer(ros::Duration(1.0 / gimbal_update_hz_),
                                                              boost::bind(&GazeboPPComPlugin::TimerCallback, this, _1, node_idx));

            node.gimbal_sub = ros_node_handle_->subscribe<geometry_msgs::Twist>("/" + node.name + "/command/gimbal", 1,
                                                                                boost::bind(&GazeboPPComPlugin::GimbalCallback, this, _1, node_idx));
            node.gimbal_pub = ros_node_handle_->advertise<geometry_msgs::TwistStamped>("/" + node.name + "/gimbal", 1);
            node.gimbal_cmd = Eigen::VectorXd(6);
            node.gimbal_cmd.setZero();
            node.gimbal_cmd_last_update = ros::Time::now();

            node.odom_deque.clear();
            node.trigger_deque.clear();
            node.cam_rpy_deque.clear();
            node.cam_rpy_rate_deque.clear();
            node.cam_state_time_deque.clear();
            node.last_auto_capture = world_->SimTime();
            node.last_manual_capture = ros::Time::now();
        }

        cloud_inpo_pub_ = ros_node_handle_->advertise<sensor_msgs::PointCloud2>("/interest_cloud", 100);

        // Listen to the update event. This event is broadcast every simulation iteration.
        last_time_topo_ = world_->SimTime();
        last_time_inpo_ = last_time_topo_;
        this->updateConnection_topology_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboPPComPlugin::OnUpdateCheckTopology, this, _1));
        this->updateConnection_interestpoints_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboPPComPlugin::OnUpdateCheckInterestPoints, this, _1));
    }

    void GazeboPPComPlugin::GimbalCallback(const geometry_msgs::TwistConstPtr &msg, int node_idx)
    {
        ppcom_nodes_[node_idx].gimbal_cmd_last_update = ros::Time::now();
        ppcom_nodes_[node_idx].gimbal_cmd(0) = msg->linear.x;
        ppcom_nodes_[node_idx].gimbal_cmd(1) = msg->linear.y;
        ppcom_nodes_[node_idx].gimbal_cmd(2) = msg->linear.z;
        ppcom_nodes_[node_idx].gimbal_cmd(3) = msg->angular.x;
        ppcom_nodes_[node_idx].gimbal_cmd(4) = msg->angular.y;
        ppcom_nodes_[node_idx].gimbal_cmd(5) = msg->angular.z;
    }

    void GazeboPPComPlugin::OdomCallback(const nav_msgs::OdometryConstPtr &msg, int node_idx)
    {
        ppcom_nodes_[node_idx].odom_msg = *msg;
        ppcom_nodes_[node_idx].odom_msg_received = true;

        ppcom_nodes_[node_idx].odom_deque.push_back(*msg);
        if (ppcom_nodes_[node_idx].odom_deque.size() > ppcom_nodes_[node_idx].odom_save_size_)
            ppcom_nodes_[node_idx].odom_deque.pop_front();
    }

    void GazeboPPComPlugin::OnUpdateCheckTopology(const common::UpdateInfo &_info)
    {
        if (kPrintOnUpdates)
            gzdbg << __FUNCTION__ << "() called." << endl;

        // common::Time current_time = world_->SimTime();

        if (ppcom_nodes_[ppcom_slf_idx_].role != "manager")
            return;

        TicToc tt_topo;

        // Update the topology
        UpdateTopology();

        tt_topo.Toc();

        // printf("Topo Time: %.3f. Ip Time: %.3f.\n", tt_topo.GetLastStop());
    }

    void GazeboPPComPlugin::UpdateTopology()
    {
        gazebo::common::Time current_time = world_->SimTime();
        double dt_topo = (current_time - last_time_topo_).Double();
        // Update the ray casting every 0.1s
        if (dt_topo > 1.0 / ppcom_hz_)
        {
            last_time_topo_ = current_time;

            Eigen::MatrixXd distMat = -1 * Eigen::MatrixXd::Ones(Nnodes_, Nnodes_);
            vector<vector<bool>> los_check(Nnodes_, vector<bool>(Nnodes_, false));

            for (int i = 0; i < Nnodes_; i++)
            {
                PPComNode &node_i = ppcom_nodes_[i];

                if (!node_i.odom_msg_received)
                    continue;

                // Create the start points
                Vector3d pi(node_i.odom_msg.pose.pose.position.x,
                            node_i.odom_msg.pose.pose.position.y,
                            node_i.odom_msg.pose.pose.position.z);

                for (int j = i + 1; j < Nnodes_; j++)
                {
                    PPComNode &node_j = ppcom_nodes_[j];

                    if (!node_j.odom_msg_received)
                        continue;

                    assert(node_i.name != node_j.name);

                    // Find the position of the neighbour
                    Vector3d pj(node_j.odom_msg.pose.pose.position.x,
                                node_j.odom_msg.pose.pose.position.y,
                                node_j.odom_msg.pose.pose.position.z);

                    los_check[i][j] = los_check[j][i] = CheckTopoLOS(pi, node_i.offset, pj, node_j.offset, node_i.ray_topo);

                    // Assign the distance if there is line of sight
                    if (los_check[i][j])
                    {
                        distMat(i, j) = (pi - pj).norm();
                        distMat(j, i) = distMat(i, j);
                    }
                }

                node_i.SetTopology(distMat);
            }

            // Publish the information of the topology
            rotors_comm::PPComTopology topo_msg;
            topo_msg.header.frame_id = "world";
            topo_msg.header.stamp = Util::GazeboTimeToRosTime(world_->SimTime());

            topo_msg.node_id.clear();
            for (PPComNode &node : ppcom_nodes_)
            {
                topo_msg.node_id.push_back(node.name);
                topo_msg.node_role.push_back(node.role);
                topo_msg.node_odom.push_back(node.odom_msg);
                // printf("Node %s. OdomCov: %f\n", node.name.c_str(), node.odom_msg.pose.covariance[0]);
            }

            topo_msg.range.clear();
            for (int i = 0; i < Nnodes_; i++)
                for (int j = i + 1; j < Nnodes_; j++)
                    topo_msg.range.push_back(distMat(i, j));

            ppcom_nodes_[ppcom_slf_idx_].topo_pub.publish(topo_msg);
        }
    }

    void GazeboPPComPlugin::OnUpdateCheckInterestPoints(const common::UpdateInfo &_info)
    {
        if (kPrintOnUpdates)
            gzdbg << __FUNCTION__ << "() called." << endl;

        // common::Time current_time = world_->SimTime();

        if (ppcom_nodes_[ppcom_slf_idx_].role != "manager")
            return;

        TicToc tt_ip;

        // Update the interest point
        UpdateInterestPoints();

        tt_ip.Toc();

        // Calculate and publish the score
        TallyScore();

        // printf("Ip Time: %.3f.\n", tt_ip.GetLastStop());
    }

    void GazeboPPComPlugin::UpdateInterestPoints()
    {
        bool published_int_cloud = false;
        for (int i = 0; i < Nnodes_; i++)
        {
            PPComNode &node_i = ppcom_nodes_[i];

            // GCS no need to check detection
            if (node_i.name == "gcs")
                continue;

            if (!node_i.odom_msg_received)
                continue;

            gazebo::common::Time current_time = world_->SimTime();
            double dt_inspection = (current_time - node_i.last_auto_capture).Double();
            if (dt_inspection > node_i.capture_interval)
            {
                node_i.last_auto_capture = current_time;
                visualization_msgs::Marker m;
                m.type = visualization_msgs::Marker::LINE_STRIP;
                m.action = visualization_msgs::Marker::DELETEALL;
                m.id = 1; // % 3000;  // Start the id again after ___ points published (if not RVIZ goes very slow)
                m.ns = "view_agent_1";

                m.color.a = 1.0; // Don't forget to set the alpha!
                m.color.r = 0.0;
                m.color.g = 1.0;
                m.color.b = 0.0;

                m.scale.x = 0.20;
                m.scale.y = 0.0001;
                m.scale.z = 0.0001;
                m.header.stamp = Util::GazeboTimeToRosTime(world_->SimTime());
                m.header.frame_id = "world";
                node_i.camera_pyramid_pub.publish(m);

                if (!node_i.manual_trigger)
                {
                    EvaluateCam(node_i, ros::Time::now());
                }
                else
                {
                    while (!node_i.trigger_deque.empty())
                    {
                        if (node_i.trigger_deque.front().data == false) continue;
                        if ((node_i.trigger_deque.front().header.stamp - node_i.last_manual_capture).toSec()>
                             node_i.capture_interval) 
                        {
                            EvaluateCam(node_i, node_i.trigger_deque.front().header.stamp);
                            node_i.last_manual_capture = node_i.trigger_deque.front().header.stamp;
                        }
                        node_i.trigger_deque.pop_front();
                    }
                    
                }
                
                std::map<int, IndexedInterestPoint> &nodeIPLog = node_i.InPoLog;
                // Copy the detected points to the cloud and publish it
                CloudXYZINPtr CloudDtectedInPo(new CloudXYZIN());
                CloudDtectedInPo->resize(nodeIPLog.size());

                // Copy the points into the pointcloud structure
                #pragma omp parallel for num_threads(MAX_THREADS)
                for(map< int, IndexedInterestPoint >::iterator itr = nodeIPLog.begin(); itr != nodeIPLog.end(); itr++)
                {
                    CloudDtectedInPo->points[itr->second.detected_order] = itr->second.scored_point;
                    CloudDtectedInPo->points[itr->second.detected_order].curvature = itr->second.detected_order;
                }

                // Publish the pointcloud
                Util::publishCloud(node_i.CloudDetectedInpoPub, *CloudDtectedInPo, ros::Time::now(), "world");   

                // Visualize the frustrum
                myTf<double> tf_uav(node_i.odom_msg);
                Vector3d p_cam_world = tf_uav * node_i.Cam_rel_Uav;
                Vector3d ypr(tf_uav.yaw() + node_i.cam_rpy(2) / M_PI * 180.0, node_i.cam_rpy(1) / M_PI * 180.0, 0.0);
                myTf<double> tf_cam(Util::YPR2Rot(ypr), p_cam_world);                
                std::vector<Vector3d> point_list_in_world;
                std::vector<Vector3d> point_list_in_world_close;
                point_list_in_world.push_back(p_cam_world);
                double dist = node_i.focal_length/node_i.pixel_size * 0.001 * node_i.desired_mm_per_pixel+5.0;
                for (double k : {1.0, -1.0})
                {
                    for (double l : {1.0, -1.0})
                    {
                        Vector3d point_in_cam(dist,
                                              k * tan(node_i.fov_h * 0.00872664625) * dist,
                                              l * tan(node_i.fov_v * 0.00872664625) * dist);
                        Vector3d point_in_world = tf_cam * point_in_cam;
                        point_list_in_world.push_back(point_in_world);
                        point_list_in_world_close.push_back(tf_cam * (point_in_cam * 0.4));
                    }
                    point_list_in_world.push_back(p_cam_world);
                }
                point_list_in_world.push_back(point_list_in_world[1]);
                point_list_in_world.push_back(point_list_in_world[4]);
                point_list_in_world.push_back(point_list_in_world[5]);
                point_list_in_world.push_back(point_list_in_world[2]);

                point_list_in_world.push_back(point_list_in_world_close[1]);
                point_list_in_world.push_back(point_list_in_world_close[3]);
                point_list_in_world.push_back(point_list_in_world_close[2]);
                point_list_in_world.push_back(point_list_in_world_close[0]);
                point_list_in_world.push_back(point_list_in_world_close[1]);

                // visualization_msgs::Marker m;
                // m.type   = visualization_msgs::Marker::LINE_STRIP;
                // m.action = visualization_msgs::Marker::DELETEALL;
                // m.id     = 1; // % 3000;  // Start the id again after ___ points published (if not RVIZ goes very slow)
                // m.ns     = "view_agent_1";

                // m.color.a = 1.0; // Don't forget to set the alpha!
                // m.color.r = 0.0;
                // m.color.g = 1.0;
                // m.color.b = 0.0;

                // m.scale.x = 0.15;
                // m.scale.y = 0.0001;
                // m.scale.z = 0.0001;
                // m.header.stamp = ros::Time::now();
                // m.header.frame_id = "world";
                // node_i.camera_pyramid_pub.publish(m);

                m.action = visualization_msgs::Marker::ADD;
                // pose is actually not used in the marker, but if not RVIZ complains about the quaternion
                m.pose.position.x = 0.0;
                m.pose.position.y = 0.0;
                m.pose.position.z = 0.0;
                m.pose.orientation.x = 0.0;
                m.pose.orientation.y = 0.0;
                m.pose.orientation.z = 0.0;
                m.pose.orientation.w = 1.0;

                for (auto point : point_list_in_world)
                {
                    geometry_msgs::Point point1;
                    point1.x = point(0);
                    point1.y = point(1);
                    point1.z = point(2);
                    m.points.push_back(point1);
                }
                node_i.camera_pyramid_pub.publish(m);

                if (!published_int_cloud)
                {
                    published_int_cloud = true;
                    Util::publishCloud(cloud_inpo_pub_, *cloud_inpo_, ros::Time::now(), "world");
                }
            }
        }
    }
    
    void GazeboPPComPlugin::EvaluateCam(PPComNode& node_i, ros::Time time_of_evaluate)
    {
        std::map<int, IndexedInterestPoint> &nodeIPLog = node_i.InPoLog;

        nav_msgs::Odometry _odom_msg;
        Vector3d _cam_rpy;
        Vector3d _cam_rpy_rate;
        findClosestOdom(time_of_evaluate, node_i, _odom_msg);
        findClosestGimState(time_of_evaluate, node_i, _cam_rpy, _cam_rpy_rate);
        
        // See the technical note https://ntu-aris.github.io/caric/docs/CARIC_motion_blur.pdf
        // for the mathematical derivation of the following calculations

        /* #region Calculate the first order states ---------------------------------------------*/

        // Body frame pose
        myTf tf_W_B(_odom_msg);
        double psi   = DEG2RAD(tf_W_B.yaw());
        double theta = DEG2RAD(tf_W_B.pitch());
        double phi   = DEG2RAD(tf_W_B.roll());
        
        // Stabilizer pose wrt the body frame
        Vector3d tran_B_S(node_i.Cam_rel_Uav);
        Quaternd quat_B_S(Util::YPR2Rot(0, 0, -RAD2DEG(phi))*Util::YPR2Rot(0, -RAD2DEG(theta), 0));
        myTf tf_B_S(quat_B_S, tran_B_S);
        
        // Stabilizer pose wrt to the world frame
        myTf tf_W_S = tf_W_B * tf_B_S;

        // Camera angles
        double alpha = (_cam_rpy(2));
        double beta  = (_cam_rpy(1));
        double da = _cam_rpy_rate(2);
        double db = _cam_rpy_rate(1);
        
        // Camera pose
        Vector3d tran_S_C(0, 0, 0);
        Quaternd quat_S_C(Util::YPR2Rot(RAD2DEG(alpha), RAD2DEG(beta), 0));
        myTf tf_S_C(quat_S_C, tran_S_C);
        myTf tf_W_C = tf_W_S * tf_S_C;
        
        /* #endregion Calculate the first order states ------------------------------------------*/

        std::vector<int> k_idx; std::vector<float> k_distances;
        kdTreeInterestPts_.radiusSearch(tf_W_C.Point3D<PointXYZIN>(), node_i.visible_radius, k_idx, k_distances);

        for (int j = 0; j < k_idx.size(); j++) //
        {
            PointXYZIN &kpoint = cloud_inpo_->points[k_idx[j]];
            assert(kpoint.curvature == k_idx[j]);

            // Interest point coordinates in world and in camera frames
            Vector3d p_W_intpoint(kpoint.x, kpoint.y, kpoint.z);
            Vector3d p_C_intpoint = tf_W_C.inverse() * p_W_intpoint;

            /* #region Check if interest points fall in the FOV ---------------------------------*/
            
            if (p_C_intpoint(0) <= 0.0)
                continue;

            double horz_angle = atan(p_C_intpoint(1) / p_C_intpoint(0)) / M_PI * 180.0;
            double vert_angle = atan(p_C_intpoint(2) / p_C_intpoint(0)) / M_PI * 180.0;

            if (horz_angle > node_i.fov_h / 2 || horz_angle < -node_i.fov_h / 2 ||
                vert_angle > node_i.fov_v / 2 || vert_angle < -node_i.fov_v / 2)
                continue;

            /* #endregion Check if interest points fall in the FOV ------------------------------*/

            double visualized = false;
            if (CheckInterestPointLOS(tf_W_C.pos, p_W_intpoint, node_i.ray_inpo))
            {
                // Temporary point to be evaluated for actual detection
                PointXYZIN detected_point = kpoint; detected_point.intensity = 0;
                
                /* #region Evaluate the motion blur ---------------------------------------------*/

                // Linear velocity of the body frame
                Vector3d vel_W_B = tf_W_B.rot*Vector3d(_odom_msg.twist.twist.linear.x,
                                                       _odom_msg.twist.twist.linear.y,
                                                       _odom_msg.twist.twist.linear.z);

                // Angular velocity of body frame
                Vector3d omega_B_B(_odom_msg.twist.twist.angular.x,
                                   _odom_msg.twist.twist.angular.y,
                                   _odom_msg.twist.twist.angular.z);

                // Angular velocity of the stabilizer frame wrt the world frame, using equation (8) in the technical note
                Vector3d omega_W_S(0, 0, sin(phi)/cos(theta)*omega_B_B(1) + cos(phi)/cos(theta)*omega_B_B(2));

                // Shorthands
                double sa = sin(alpha);
                double ca = cos(alpha);
                double sb = sin(beta);
                double cb = cos(beta);

                // Camera linear velocity wrt world frame (note that tran_S_C = tran_B_S)
                Vector3d vel_W_C = vel_W_B + tf_W_B.rot*Util::skewSymmetric(omega_B_B)*tran_B_S;

                // Camera angular velocity wrt the world frame
                Matrix3d rotm_S_C_dot;
                rotm_S_C_dot << -da*sa*cb - db*ca*sb, -da*ca, -da*sa*sb + db*ca*cb,
                                 da*ca*cb - db*sa*sb, -da*sa,  da*ca*sb + db*sa*cb,
                                -db*cb,                0,     -db*sb;
                
                // Angular velocity of the camera frame
                Matrix3d omega_W_C_cross = Util::skewSymmetric(omega_W_S) + tf_W_S.rot*rotm_S_C_dot*tf_W_C.rot.inverse();

                // Confirm that the matrix is skew symmetric
                assert(fabs(omega_W_C_cross.trace()) < 1e-12 && fabs((omega_W_C_cross + omega_W_C_cross.transpose()).sum()) < 1e-9);

                // Retrieve the angular velocity vector
                Vector3d omega_W_C(omega_W_C_cross(2, 1), omega_W_C_cross(0, 2), omega_W_C_cross(1, 0));

                // Finding the linear velocity of the interestpoint in the camera frame
                Vector3d vel_C_intpoint = tf_W_C.rot.inverse()*Util::skewSymmetric(omega_W_C)*(-p_W_intpoint + tf_W_C.pos)
                                        - tf_W_C.rot.inverse()*vel_W_C;

                Vector3d p_C_intpoint_moved = p_C_intpoint + vel_C_intpoint * node_i.exposure;

                // horizontal camera pixels
                double pixel_move_v = node_i.focal_length * fabs(p_C_intpoint(1) / p_C_intpoint(0) -
                                        p_C_intpoint_moved(1) / p_C_intpoint_moved(0)) / node_i.pixel_size;

                // vertical camera pixels
                double pixel_move_u = node_i.focal_length * fabs(p_C_intpoint(2) / p_C_intpoint(0) -
                                        p_C_intpoint_moved(2) / p_C_intpoint_moved(0)) / node_i.pixel_size;

                double q_blur = min(1.0 / max(fabs(pixel_move_v), fabs(pixel_move_u)), 1.0);

                /* #endregion Evaluate the motion blur ------------------------------------------*/


                /* #region Evaluate the pixel size ----------------------------------------------*/

                // compute resolution requirement mm per pixel
                Vector3d Normal_world(kpoint.normal_x, kpoint.normal_y, kpoint.normal_z);
                Vector3d Normal_cam = (tf_W_C.rot.inverse() * Normal_world).normalized();

                Vector3d x_disp(-Normal_cam(2), 0.0, Normal_cam(0)); // the gradient projected on the x-z plane
                Vector3d inPoint_xplus = p_C_intpoint + 0.0005 * x_disp;
                Vector3d inPoint_xminus = p_C_intpoint - 0.0005 * x_disp;
                
                Vector3d y_disp(-Normal_cam(1), Normal_cam(0), 0.0); // the gradient projected on the x-y plane
                Vector3d inPoint_yplus = p_C_intpoint + 0.0005 * y_disp;
                Vector3d inPoint_yminus = p_C_intpoint - 0.0005 * y_disp;

                if (inPoint_yplus(0) < 0 || inPoint_yminus(0) < 0 ||
                    inPoint_xplus(0) < 0 || inPoint_xminus(0) < 0)
                {
                    printf(KRED "NOT RIGHT! X smaller than zero!!" RESET);
                    continue;
                }

                double v_plus = inPoint_xplus(2) * node_i.focal_length / inPoint_xplus(0);
                double v_minus = inPoint_xminus(2) * node_i.focal_length / inPoint_xminus(0);
                double mm_per_pixel_v = node_i.pixel_size / fabs(v_plus - v_minus);

                double u_plus = inPoint_yplus(1) * node_i.focal_length / inPoint_yplus(0);
                double u_minus = inPoint_yminus(1) * node_i.focal_length / inPoint_yminus(0);
                double mm_per_pixel_u = node_i.pixel_size / fabs(u_plus - u_minus);

                double q_res = min(node_i.desired_mm_per_pixel / max(mm_per_pixel_v, mm_per_pixel_u), 1.0);

                /* #endregion Evaluate the pixel size -------------------------------------------*/

                
                /* #region Combine the scores ---------------------------------------------------*/

                double score = q_blur * q_res;

                if (score < 0.2)
                    continue;

                /* #endregion Combine the scores ------------------------------------------------*/


                /* #region Store the detected point ---------------------------------------------*/
                
                // std::cout<<"mm_per pixel u is "<<mm_per_pixel_u<<"mm_per pixel v is "<<mm_per_pixel_v<<std::endl;
                if (score > cloud_inpo_->points[k_idx[j]].intensity)
                    cloud_inpo_->points[k_idx[j]].intensity = score;

                detected_point.intensity = score;
                int point_idx = (int)detected_point.curvature;

                if (nodeIPLog.find(point_idx) == nodeIPLog.end())
                    nodeIPLog[point_idx] = IndexedInterestPoint(nodeIPLog.size(), detected_point);
                else if (nodeIPLog[point_idx].scored_point.intensity < detected_point.intensity)
                    nodeIPLog[point_idx] = IndexedInterestPoint(nodeIPLog[point_idx].detected_order, detected_point);

                /* #endregion Store the detected point ------------------------------------------*/
            
            }
        }
    }

    void GazeboPPComPlugin::TallyScore()
    {
        // Go through the log of each node in line of sight of GCS to tally the score
        for (int i = 0; i < Nnodes_; i++)
        {
            PPComNode &node_i = ppcom_nodes_[i];
            std::map<int, IndexedInterestPoint> &node_i_iplog = node_i.InPoLog;

            // Only job of the GCS
            if (node_i.name != "gcs")
                continue;
            
            Eigen::MatrixXd topology = node_i.GetTopology();

            // printf("Topo: \n");
            // cout << topology << endl;

            // Check for other nodes in line of sight
            for(int j = i+1; j < Nnodes_; j++)
            {
                // If no link, continue
                if(topology(i, j) <= 0.0)
                    continue;

                PPComNode &node_j = ppcom_nodes_[j];
                std::map<int, IndexedInterestPoint> &node_j_iplog = node_j.InPoLog;    

                // Check the IPLog of the neigbour and update the score
                #pragma omp parallel for num_threads(MAX_THREADS)
                for(map< int, IndexedInterestPoint >::iterator itr = node_j_iplog.begin(); itr != node_j_iplog.end(); itr++)
                {
                    int global_idx = itr->second.scored_point.curvature;
                    float ip_score_from_nbr = itr->second.scored_point.intensity;

                    assert(node_i_iplog[global_idx].scored_point.curvature == global_idx);

                    if (node_i_iplog[global_idx].scored_point.intensity < ip_score_from_nbr)
                        node_i_iplog[global_idx].scored_point.intensity = ip_score_from_nbr;
                }
            }

            static ros::Publisher score_at_point_pub = ros_node_handle_->advertise<visualization_msgs::MarkerArray>("/score_at_point_txt", 10);
            static ros::Publisher score_total_pub = ros_node_handle_->advertise<visualization_msgs::Marker>("/viz_score_totalled", 10);
            static ros::Publisher score_pub = ros_node_handle_->advertise<sensor_msgs::PointCloud>("/" + node_i.name + "/score", 10);
            static ros::Publisher score_eval_pub = ros_node_handle_->advertise<sensor_msgs::PointCloud>("/score_eval", 10);

            visualization_msgs::MarkerArray score_at_point_all;

            // Create a message to broadcast the detection info from gcs
            sensor_msgs::PointCloud DetectedIP;
            DetectedIP.header.stamp = Util::GazeboTimeToRosTime(world_->SimTime());
            DetectedIP.header.frame_id = "world";
            sensor_msgs::ChannelFloat32 normal_x; normal_x.name = "normal_x"; DetectedIP.channels.push_back(normal_x);
            sensor_msgs::ChannelFloat32 normal_y; normal_y.name = "normal_y"; DetectedIP.channels.push_back(normal_y);
            sensor_msgs::ChannelFloat32 normal_z; normal_z.name = "normal_z"; DetectedIP.channels.push_back(normal_z);
            sensor_msgs::ChannelFloat32 score;    score.name    = "score";    DetectedIP.channels.push_back(score);

            // Create a message for indexing
            sensor_msgs::PointCloud DetectedIPEval = DetectedIP;
            sensor_msgs::ChannelFloat32 global_index; global_index.name = "global_index"; DetectedIPEval.channels.push_back(global_index);

            int total_detected = 0;
            double total_score = 0;
            for(map< int, IndexedInterestPoint >::iterator itr = node_i_iplog.begin(); itr != node_i_iplog.end(); itr++)
            {
                if (itr->second.scored_point.intensity > 0.0)
                {
                    total_detected++;
                    total_score += itr->second.scored_point.intensity;

                    geometry_msgs::Point32 detected_point;
                    detected_point.x = itr->second.scored_point.x;
                    detected_point.y = itr->second.scored_point.y;
                    detected_point.z = itr->second.scored_point.z;

                    DetectedIP.points.push_back(detected_point);
                    DetectedIP.channels[0].values.push_back(itr->second.scored_point.normal_x);
                    DetectedIP.channels[1].values.push_back(itr->second.scored_point.normal_y);
                    DetectedIP.channels[2].values.push_back(itr->second.scored_point.normal_z);
                    DetectedIP.channels[3].values.push_back(itr->second.scored_point.intensity);

                    DetectedIPEval.points.push_back(detected_point);
                    DetectedIPEval.channels[0].values.push_back(itr->second.scored_point.normal_x);
                    DetectedIPEval.channels[1].values.push_back(itr->second.scored_point.normal_y);
                    DetectedIPEval.channels[2].values.push_back(itr->second.scored_point.normal_z);
                    DetectedIPEval.channels[3].values.push_back(itr->second.scored_point.intensity);
                    DetectedIPEval.channels[4].values.push_back(itr->second.scored_point.curvature);

                    visualization_msgs::Marker score_atpoint;
                    score_atpoint.header.frame_id = "world";
                    score_atpoint.header.stamp = Util::GazeboTimeToRosTime(world_->SimTime());
                    score_atpoint.ns = node_i.name;
                    score_atpoint.id = total_detected;
                    score_atpoint.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                    score_atpoint.action = visualization_msgs::Marker::ADD;
                    score_atpoint.pose.position.x = detected_point.x;
                    score_atpoint.pose.position.y = detected_point.y;
                    score_atpoint.pose.position.z = detected_point.z + 0.5;
                    score_atpoint.pose.orientation.x = 0.0;
                    score_atpoint.pose.orientation.y = 0.0;
                    score_atpoint.pose.orientation.z = 0.0;
                    score_atpoint.pose.orientation.w = 1.0;
                    score_atpoint.scale.x = 1.0;
                    score_atpoint.scale.y = 1.0;
                    score_atpoint.scale.z = 1.0;
                    score_atpoint.color.r = 1.0;
                    score_atpoint.color.g = 1.0;
                    score_atpoint.color.b = 1.0;
                    score_atpoint.color.a = 1.0;
                    std::ostringstream streamObj3;
                    streamObj3 << std::fixed;
                    streamObj3 << std::setprecision(2);
                    streamObj3 << itr->second.scored_point.intensity;
                    score_atpoint.text = streamObj3.str();
                    // score_atpoint.text = std::to_string(itr->second.scored_point.intensity);
                    score_at_point_all.markers.push_back(score_atpoint);
                }
            }
            string report = myprintf("Detected: %4d / %4d. Score: %6.3f",
                                     total_detected, node_i_iplog.size(), total_score);

            // Visualize the score on rviz
            visualization_msgs::Marker marker;
            marker.header.frame_id = "score_report";
            marker.header.stamp = Util::GazeboTimeToRosTime(world_->SimTime());
            marker.ns = node_i.name;
            marker.id = 0;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = node_i.odom_msg.pose.pose.position.x;
            marker.pose.position.y = node_i.odom_msg.pose.pose.position.y;
            marker.pose.position.z = node_i.odom_msg.pose.pose.position.z;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 2.5;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker.text = report;

            score_at_point_pub.publish(score_at_point_all);
            score_total_pub.publish(marker);
            score_pub.publish(DetectedIP);
            score_eval_pub.publish(DetectedIPEval);
        }
    }
    
    void GazeboPPComPlugin::TimerCallback(const ros::TimerEvent &, int node_idx)
    {
        PPComNode &node_i = ppcom_nodes_[node_idx];

        if (node_i.name == "gcs")
            return;

        if (node_i.gimbal_cmd(0) < -1e-7 &&
            (ros::Time::now() - node_i.gimbal_cmd_last_update).toSec() > 2.0 / gimbal_update_hz_)
        {
            // change from rate control to angle control
            node_i.gimbal_cmd(0) = 0.1;
            node_i.gimbal_cmd(1) = node_i.cam_rpy(1);
            node_i.gimbal_cmd(2) = node_i.cam_rpy(2);
        }

        if (node_i.gimbal_cmd(0) > 0.0) // angle control mode
        {
            double pitch_cmd = min(node_i.gimbal_pitch_max, max(-node_i.gimbal_pitch_max, Util::wrapToPi(node_i.gimbal_cmd(1))));
            double yaw_cmd = min(node_i.gimbal_yaw_max, max(-node_i.gimbal_yaw_max, Util::wrapToPi(node_i.gimbal_cmd(2))));
            // double pitch_cmd = min(node_i.gimbal_pitch_max, max(-node_i.gimbal_pitch_max, (node_i.gimbal_cmd(1))));
            // double yaw_cmd = min(node_i.gimbal_yaw_max, max(-node_i.gimbal_yaw_max, (node_i.gimbal_cmd(2))));

            node_i.cam_rpy_rate(1) = max(-node_i.gimbal_rate_max, min(node_i.gimbal_rate_max,
                                                                (pitch_cmd - node_i.cam_rpy(1)) * gimbal_update_hz_));
            node_i.cam_rpy_rate(2) = max(-node_i.gimbal_rate_max, min(node_i.gimbal_rate_max,
                                                                (yaw_cmd - node_i.cam_rpy(2)) * gimbal_update_hz_));
            node_i.cam_rpy(1) = node_i.cam_rpy(1) + node_i.cam_rpy_rate(1) * 1.0 / gimbal_update_hz_;
            node_i.cam_rpy(2) = node_i.cam_rpy(2) + node_i.cam_rpy_rate(2) * 1.0 / gimbal_update_hz_;
        }

        if (node_i.gimbal_cmd(0) < -1e-7) // rate control mode
        {
            node_i.cam_rpy_rate(1) = max(-node_i.gimbal_rate_max, min(node_i.gimbal_rate_max, node_i.gimbal_cmd(4)));
            double pitch = node_i.cam_rpy(1) + node_i.cam_rpy_rate(1) * 1.0 / gimbal_update_hz_;
            node_i.cam_rpy(1) = max(-node_i.gimbal_pitch_max, min(node_i.gimbal_pitch_max, pitch));

            node_i.cam_rpy_rate(2) = max(-node_i.gimbal_rate_max, min(node_i.gimbal_rate_max, node_i.gimbal_cmd(5)));
            double yaw = node_i.cam_rpy(2) + node_i.cam_rpy_rate(2) * 1.0 / gimbal_update_hz_;
            node_i.cam_rpy(2) = max(-node_i.gimbal_yaw_max, min(node_i.gimbal_yaw_max, yaw));
        }

        node_i.cam_rpy_deque.push_back(node_i.cam_rpy);
        node_i.cam_rpy_rate_deque.push_back(node_i.cam_rpy_rate);
        node_i.cam_state_time_deque.push_back(ros::Time::now());
        if (node_i.cam_rpy_deque.size() > node_i.odom_save_size_)
        {
            node_i.cam_rpy_deque.pop_front();
            node_i.cam_rpy_rate_deque.pop_front();
            node_i.cam_state_time_deque.pop_front();
        }

        geometry_msgs::TwistStamped gimbal_state;
        gimbal_state.header.stamp = ros::Time::now();
        gimbal_state.header.frame_id = "gimbal";
        gimbal_state.twist.linear.x = 0.0;
        gimbal_state.twist.linear.y = node_i.cam_rpy(1);
        gimbal_state.twist.linear.z = node_i.cam_rpy(2);
        gimbal_state.twist.angular.x = 0.0;
        gimbal_state.twist.angular.y = node_i.cam_rpy_rate(1);
        gimbal_state.twist.angular.z = node_i.cam_rpy_rate(2);
        node_i.gimbal_pub.publish(gimbal_state);
    }

    void GazeboPPComPlugin::readPCloud(std::string filename)
    {
        cloud_inpo_ = CloudXYZINPtr(new CloudXYZIN);
        if (pcl::io::loadPCDFile<PointXYZIN>(filename, *cloud_inpo_) == -1) // load point cloud file
        {
            PCL_ERROR("Could not read the file");
            return;
        }

        // Add the point index into the curvature field
        #pragma omp parallel for num_threads(MAX_THREADS)
        for(int i = 0; i < cloud_inpo_->size(); i++)
            cloud_inpo_->points[i].curvature = i;

        std::cout << "Loaded" << cloud_inpo_->width * cloud_inpo_->height
                  << "data points with the following fields: "
                  << std::endl;

        // for(size_t i = 0; i < cloud_inpo_->points.size(); ++i)
        //     std::cout << "    " << cloud_inpo_->points[i].x
        //               << " "    << cloud_inpo_->points[i].y
        //               << " "    << cloud_inpo_->points[i].z
        //               << " "    << cloud_inpo_->points[i].normal_x
        //               << " "    << cloud_inpo_->points[i].normal_y
        //               << " "    << cloud_inpo_->points[i].normal_z << std::endl;

        kdTreeInterestPts_.setInputCloud(cloud_inpo_);
    }

    bool GazeboPPComPlugin::CheckTopoLOS(const Vector3d &pi, double bi, const Vector3d &pj, double bj, gazebo::physics::RayShapePtr &ray)
    {
        vector<Vector3d> Pi;
        Pi.push_back(pi + Vector3d(bi, 0, 0));
        Pi.push_back(pi - Vector3d(bi, 0, 0));
        Pi.push_back(pi + Vector3d(0, bi, 0));
        Pi.push_back(pi - Vector3d(0, bi, 0));
        Pi.push_back(pi + Vector3d(0, 0, bi));
        Pi.push_back(pi - Vector3d(0, 0, bi));
        // Make sure the virtual antenna does not go below the ground
        Pi.back().z() = max(0.1, Pi.back().z());

        vector<Vector3d> Pj;
        Pj.push_back(pj + Vector3d(bj, 0, 0));
        Pj.push_back(pj - Vector3d(bj, 0, 0));
        Pj.push_back(pj + Vector3d(0, bj, 0));
        Pj.push_back(pj - Vector3d(0, bj, 0));
        Pj.push_back(pj + Vector3d(0, 0, bj));
        Pj.push_back(pj - Vector3d(0, 0, bj));
        // Make sure the virtual antenna does not go below the ground
        Pj.back().z() = max(0.1, Pj.back().z());

        // Ray tracing from the slf node to each of the nbr node
        double rtDist;      // Raytracing distance
        double ppDist = 0;  // Peer to peer distance
        string entity_name; // Name of intersected object
        ignition::math::Vector3d start_point;
        ignition::math::Vector3d end_point;
        bool los = false;
        for (Vector3d &pa : Pi)
        {
            start_point = ignition::math::Vector3d(pa.x(), pa.y(), pa.z());

            for (Vector3d &pb : Pj)
            {
                end_point = ignition::math::Vector3d(pb.x(), pb.y(), pb.z());

                ray->SetPoints(start_point, end_point);
                ray->GetIntersection(rtDist, entity_name);

                ppDist = (pa - pb).norm();
                if (entity_name == "" || (rtDist >= ppDist - 0.1))
                    return true;
            }
        }
        return los;
    }

    bool GazeboPPComPlugin::CheckInterestPointLOS(const Eigen::Vector3d &pi, const Eigen::Vector3d &pj, gazebo::physics::RayShapePtr &ray)
    {
        // Ray tracing from the optical origin to the interestpoint
        double rtDist;      // Raytracing distance
        double ppDist = 0;  // Peer to peer distance
        string entity_name; // Name of intersected object
        ignition::math::Vector3d start_point;
        ignition::math::Vector3d end_point;
        bool los = false;
        start_point = ignition::math::Vector3d(pi(0), pi(1), pi(2));
        end_point = ignition::math::Vector3d(pj(0), pj(1), pj(2));

        ray->SetPoints(start_point, end_point);
        ray->GetIntersection(rtDist, entity_name);

        ppDist = (pi - pj).norm();
        if (fabs(rtDist - ppDist) <= 0.025)
            return true;
        return los;
    }

    void GazeboPPComPlugin::findClosestOdom(ros::Time stamp, PPComNode& node_i, nav_msgs::Odometry& odom)
    {
        if (node_i.odom_deque.back().header.stamp - stamp < ros::Duration(0))
        {
            odom = node_i.odom_deque.back();
            return;
        }
        else if (node_i.odom_deque.front().header.stamp - stamp > ros::Duration(0))
        {
            ROS_WARN("trigger time out of the range of the buffer! leave if first!!");
            odom = node_i.odom_deque.front();
            return;
        }

        int oldest = 0;
        int newest = node_i.odom_deque.size() - 1;
        if (newest <= 0)
        {
            ROS_WARN("findClosestOdom: not many odom received!!");
            return;
        }
        int mid;
        while (newest - oldest > 1)
        {
            mid = oldest + (newest - oldest) / 2;
            if (node_i.odom_deque[mid].header.stamp - stamp < ros::Duration(0))
                oldest = mid;
            else
                newest = mid;
        }

        if (stamp - node_i.odom_deque[oldest].header.stamp >
            node_i.odom_deque[newest].header.stamp - stamp)
        {
            odom = node_i.odom_deque[newest];
        }
        else
        {
            odom = node_i.odom_deque[oldest];
        }

        if (fabs((odom.header.stamp - stamp).toSec()) > 0.03)
            ROS_WARN("findClosestOdom: found odom is too far from the trigger time!!");

        return;
    }

    void GazeboPPComPlugin::findClosestGimState(ros::Time stamp, PPComNode& node_i, 
                                                Vector3d& cam_rpy, Vector3d& cam_rpy_rate)
    {
        if (node_i.cam_state_time_deque.back() - stamp < ros::Duration(0))
        {
            cam_rpy = node_i.cam_rpy_deque.back();
            cam_rpy_rate = node_i.cam_rpy_rate_deque.back();
            return;
        }
        else if (node_i.cam_state_time_deque.front() - stamp > ros::Duration(0))
        {
            ROS_WARN("trigger time out of the range of the buffer! leave if first!!");
            cam_rpy = node_i.cam_rpy_deque.front();
            cam_rpy_rate = node_i.cam_rpy_rate_deque.front();            
            return;
        }

        int oldest = 0;
        int newest = node_i.cam_state_time_deque.size() - 1;
        if (newest <= 0)
        {
            ROS_WARN("findClosestGimState: not many odom received!!");
            return;
        }
        int mid;
        while (newest - oldest > 1) //binary search
        {
            mid = oldest + (newest - oldest) / 2;
            if (node_i.cam_state_time_deque[mid] - stamp < ros::Duration(0))
                oldest = mid;
            else
                newest = mid;
        }

        int idx;
        if (stamp - node_i.cam_state_time_deque[oldest] >
            node_i.cam_state_time_deque[newest] - stamp)
        {
            idx = newest;
        }
        else
        {
            idx = oldest;;
        }

        if (fabs((node_i.cam_state_time_deque[idx] - stamp).toSec()) > 0.05)
            ROS_WARN("findClosestGimState: found gimbal state is too far from the trigger time!!");

        cam_rpy = node_i.cam_rpy_deque[idx];
        cam_rpy_rate = node_i.cam_rpy_rate_deque[idx];    
        return;
    }

    void GazeboPPComPlugin::triggerCallback(const rotors_comm::BoolStamped::ConstPtr &msg, int node_idx)
    {
        ppcom_nodes_[node_idx].trigger_deque.push_back(*msg);
        if (ppcom_nodes_[node_idx].trigger_deque.size() > 10)
        {
            ROS_WARN("Many camera capture commands in the buffer!!");
        }
    }

    GZ_REGISTER_MODEL_PLUGIN(GazeboPPComPlugin);

} // namespace gazebo
