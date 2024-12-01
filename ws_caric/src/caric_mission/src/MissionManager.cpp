/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <thread>
#include <chrono>
#include <deque>
#include <condition_variable>

#include <iostream>
#include <Eigen/Eigen>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/JointWrench.hh>

#include "rotors_comm/PPComTopology.h"

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/ColorRGBA.h"
#include "std_msgs/Duration.h"

// Message synchronization
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/impl/uniform_sampling.hpp>
#include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include "unicon/Stop.h"
#include "utility.h"

#include "boost/filesystem.hpp"

// Printout colors
#define KNRM "\x1B[0m"
#define KRED "\x1B[31m"
#define KGRN "\x1B[32m"
#define KYEL "\x1B[33m"
#define KBLU "\x1B[34m"
#define KMAG "\x1B[35m"
#define KCYN "\x1B[36m"
#define KWHT "\x1B[37m"
#define RESET "\033[0m"

using namespace std;
using namespace Eigen;
using namespace message_filters;

string log_dir;
double mission_duration = 180;
bool mission_started = false;
ros::Time mission_start_time;

// Score sub and pub
ros::Subscriber scoreTextSub;
ros::Publisher  scoreTextPub;

ros::Subscriber scoreSub;
std::mutex score_mtx;
sensor_msgs::PointCloud scoreMsg;

// Extrinsic of the lidar
// TODO: Make these into rosparams
myTf<double> tf_B_S(Quaternd::Identity(), Vector3d(0, 0, 0.0));
myTf<double> tf_S_L(Quaternd::Identity(), Vector3d(0, 0, 0.0));
int    kf_knn_num = 05.0;
double kf_min_dis = 02.0;
double kf_min_ang = 10.0;
double kf_voxsize = 00.5;

typedef nav_msgs::Odometry OdomMsg;
typedef nav_msgs::Odometry::ConstPtr OdomMsgPtr;
typedef sensor_msgs::PointCloud2 CloudMsg;
typedef sensor_msgs::PointCloud2::ConstPtr CloudMsgPtr;
typedef sync_policies::ApproximateTime<OdomMsg, CloudMsg> MySyncPolicy;

// Handles to ROS and Gazebo object
gazebo::physics::WorldPtr world = NULL;
ros::NodeHandlePtr nh_ptr;

// Topology and status checks
std::mutex topo_mtx;

int Nnodes = 0;
vector<string>  nodeName;
vector<string>  nodeRole;
vector<OdomMsg> nodeOdom;
vector<string>  nodeStatus;
vector<bool>    nodeAlive;
MatrixXd        linkMat;

deque<message_filters::Subscriber<OdomMsg>> odomSub;
deque<message_filters::Subscriber<OdomMsg>> servoSub;
deque<message_filters::Subscriber<CloudMsg>> cloudSub;
deque<Synchronizer<MySyncPolicy>> msgSync;

// Local SLAM data
deque<CloudPosePtr> kfPose;
deque<deque<CloudXYZIPtr>> kfCloud;
deque<ros::Publisher> kfPosePub;
deque<ros::Publisher> slfKfCloudPub;
deque<ros::Publisher> cloudInWPub;

// Mission information
deque<ros::Publisher> missionDurRemained;

// Cooperative SLAM
deque<std::mutex>     nbr_kf_pub_mtx;
deque<ros::Publisher> nbrKfCloudPub;
deque<std::mutex>     nbr_odom_pub_mtx;
deque<ros::Publisher> nbrOdomPub;

// Visualization
typedef visualization_msgs::Marker RosVizMarker;
typedef std_msgs::ColorRGBA RosVizColor;
typedef ros::Publisher RosPub;

bool mission_finished = false;

struct VizAid
{
    RosVizColor    color  = RosVizColor();
    RosVizMarker   marker = RosVizMarker();
    ros::Publisher rosPub = RosPub();
};

// Predefined colors
VizAid vizAid;
RosVizColor los_color;
RosVizColor nlos_color;
RosVizColor dead_color;

// Dead of alive
ros::Publisher ppcomDoAPub;

/////////////////////////////////////////////////
// Function is called everytime a message is received.
void ContactCallback(ConstContactsPtr &_msg)
{
    if (Nnodes == 0)
        return;

    topo_mtx.lock();

    vector<bool> deadNode(Nnodes, false);
    for (int i = 0; i < _msg->contact_size(); ++i)
    {
        string col1 = _msg->contact(i).collision1();
        string col2 = _msg->contact(i).collision2();

        // Check if the node name shows up in collision, if it does, skip it
        for(int node_idx = 0; node_idx < Nnodes; node_idx++)
        {
            if (nodeRole[node_idx] == "manager")
                continue;

            bool colide_case1 = col1.find(nodeName[node_idx]) != std::string::npos
                                && col2.find("ground_plane") == std::string::npos;
            bool colide_case2 = col2.find(nodeName[node_idx]) != std::string::npos
                                && col1.find("ground_plane") == std::string::npos;
            bool on_air = nodeStatus[node_idx] == "on_air";

            // if (col1.find("gcs") != std::string::npos || col2.find("gcs") != std::string::npos)
            //     continue;

            // printf("Collision %s <-> %s. case1 %d. case2: %d. status: %s. %d\n",
            //         col1.c_str(), col2.c_str(), colide_case1, colide_case2, nodeStatus[node_idx].c_str(), on_air);

            if((colide_case1 || colide_case2) && on_air && nodeAlive[node_idx])
            {
                deadNode[node_idx] = true;
                printf("Node %d, %s (role %s) collides with %s.\n",
                        node_idx, nodeName[node_idx].c_str(), nodeRole[node_idx].c_str(),
                        colide_case1 ? col2.c_str() : col1.c_str());
                
                for(int k = 0; k < 1; k++)
                {
                    // auto force = _msg->contact(i).wrench(k).body_1_wrench().force();
                    // printf("wr %d: %6.3f. %6.3f. %6.3f. %s\n", k, force.x(), force.y(), force.z(), _msg->contact(i).world().c_str());
                    // force = _msg->contact(i).wrench(k).body_2_wrench().force();
                    // printf("wr %d: %6.3f. %6.3f. %6.3f. %s\n", k, force.x(), force.y(), force.z(), _msg->contact(i).world().c_str());
                    
                    // Command the drones to fall
                    unicon::Stop stop;
                    stop.request.message = KRED "Collision happens over " + nodeName[node_idx] + ". Control Stopped!" RESET;
                    ros::service::call("/" + nodeName[node_idx] + "/stop", stop);
                }

                // Set the state of the drone as dead
                nodeAlive[node_idx] = false;
            }
        }
    }

    topo_mtx.unlock();
}

void UpdateSLAMDatabase(int slfIdx, PointPose pose, CloudXYZIPtr &cloud)
{
    pcl::UniformSampling<PointXYZI> downsampler;
    downsampler.setRadiusSearch(kf_voxsize);
    downsampler.setInputCloud(cloud);
    downsampler.filter(*cloud);

    kfPose[slfIdx]->push_back(pose);
    kfCloud[slfIdx].push_back(cloud);
    Util::publishCloud(kfPosePub[slfIdx], *kfPose[slfIdx], ros::Time(pose.t), string("world"));
    Util::publishCloud(slfKfCloudPub[slfIdx], *cloud, ros::Time(pose.t), string("world"));

    // Check if there is line of sight to other nodes and publish this kf cloud
    for (int nbrIdx = 0; nbrIdx < Nnodes; nbrIdx++)
    {
        // Skip if nbr is self, or nbr is dead, or there is no los
        if (nbrIdx == slfIdx || nodeAlive[nbrIdx] == false || linkMat(slfIdx, nbrIdx) <= 0.0)
            continue;

        nbr_kf_pub_mtx[nbrIdx].lock();
        Util::publishCloud(nbrKfCloudPub[nbrIdx], *cloud, ros::Time(pose.t), string("world"));
        nbr_kf_pub_mtx[nbrIdx].unlock();
    }
}

void ServoCloudCallback(const OdomMsgPtr &servoMsg, const CloudMsgPtr &cloudMsg, int idx)
{
    // Do nothing if node is dead
    if (!nodeAlive[idx])
        return;

    // Ignore the data if time sync is larger than 1ms
    double time_diff_servo_cloud = fabs((cloudMsg->header.stamp - servoMsg->header.stamp).toSec());
    if(time_diff_servo_cloud > 0.001 )
    {
        printf(KRED "Node %d. Tservo: %.3f. Tcloud %.3f.\n" RESET,
                idx, servoMsg->header.stamp.toSec(), cloudMsg->header.stamp.toSec());
        return;
    }
    // else
    //     printf(KGRN "Node %d. Tcloud %.3f, Todom: %.3f\n" RESET,
    //             idx, cloudMsg->header.stamp.toSec(), odomMsg->header.stamp.toSec());

    myTf<double> tf_W_S(*servoMsg);
    Quaternd q_W_S0 = Util::YPR2Quat(tf_W_S.yaw(), 0, 0);
    myTf<double> tf_W_S0(q_W_S0, tf_W_S.pos);
    myTf<double> tf_W_B = tf_W_S0 * tf_B_S.inverse();
    PointPose pose_W_B = tf_W_B.Pose6D(servoMsg->header.stamp.toSec());

    // Transform lidar into the world frame and publish it for visualization
    CloudXYZIPtr cloud(new CloudXYZI());
    pcl::fromROSMsg(*cloudMsg, *cloud);
    pcl::transformPointCloud(*cloud, *cloud, (myTf<double>(*servoMsg)*tf_S_L).cast<float>().tfMat());
    Util::publishCloud(cloudInWPub[idx], *cloud, cloudMsg->header.stamp, string("world"));

    // Save the key frame and key cloud
    if (kfPose[idx]->size() == 0)
    {
        UpdateSLAMDatabase(idx, tf_W_B.Pose6D(servoMsg->header.stamp.toSec()), cloud);
    }
    else if(servoMsg->header.stamp.toSec() - kfPose[idx]->back().t > 1.0)
    {
        // Check the distance to register a new keyframe
        pcl::KdTreeFLANN<PointPose> kdTreeKeyFrames;
        kdTreeKeyFrames.setInputCloud(kfPose[idx]);

        vector<int> knn_idx(kf_knn_num, 0);
        vector<float> kk_sq_dis(kf_knn_num, 0);
        kdTreeKeyFrames.nearestKSearch(pose_W_B, kf_knn_num, knn_idx, kk_sq_dis);

        // Check for far distance and far angle
        bool far_distance = kk_sq_dis.front() > kf_min_dis*kf_min_dis;
        bool far_angle = true;
        for(int i = 0; i < knn_idx.size(); i++)
        {
            int kf_idx = knn_idx[i];

            // Collect the angle difference
            Quaternd Qa(kfPose[idx]->points[kf_idx].qw,
                        kfPose[idx]->points[kf_idx].qx,
                        kfPose[idx]->points[kf_idx].qy,
                        kfPose[idx]->points[kf_idx].qz);

            Quaternd &Qb = tf_W_B.rot;

            // If the angle is more than 10 degrees, add this to the key pose
            if (fabs(Util::angleDiff(Qa, Qb)) < kf_min_ang)
            {
                far_angle = false;
                break;
            }
        }

        // Admit the key frame if sufficiently spaced and publish it to neigbours
        if(far_distance || far_angle)
            UpdateSLAMDatabase(idx, tf_W_B.Pose6D(servoMsg->header.stamp.toSec()), cloud);
    }
}

void PPComCallback(const rotors_comm::PPComTopology::ConstPtr &msg)
{
    // TicToc tt_ppcom;

    static bool firstshot = true;

    topo_mtx.lock();

    // printf(KGRN "PPComCallback\n" RESET);

    if (firstshot)
    {
        Nnodes      = msg->node_id.size();
        nodeName    = msg->node_id;
        nodeRole    = msg->node_role;
        nodeOdom    = msg->node_odom;
        nodeStatus  = vector<string>(Nnodes, "on_ground"); // Assuming that all drones are static on the ground
        nodeAlive   = vector<bool>(Nnodes, true); // Assuming that all nodes are initially alive, one only dies when colliding stuff
        linkMat     = -Eigen::MatrixXd::Ones(Nnodes, Nnodes);
        firstshot   = false;
        
        // Create the subscribers with synchronization
        for(int i = 0; i < Nnodes; i++)
        {
            // if (nodeRole[i] == "manager")
            //     continue;

            string gndtr_topic = "/" + nodeName[i] + "/ground_truth/odometry";
            string servo_topic = "/" + nodeName[i] + "/servo/odometry";
            string cloud_topic = "/" + nodeName[i] + "/velodyne_points";
            odomSub.emplace_back(*nh_ptr, gndtr_topic, 100);
            servoSub.emplace_back(*nh_ptr, servo_topic, 100);
            cloudSub.emplace_back(*nh_ptr, cloud_topic, 100);
            msgSync.emplace_back(MySyncPolicy(10), servoSub[i], cloudSub[i]);
            msgSync.back().registerCallback(boost::bind(&ServoCloudCallback, _1, _2, i));

            // Local SLAM database
            kfPose.push_back(CloudPosePtr(new CloudPose()));
            kfCloud.push_back(deque<CloudXYZIPtr>());

            // Publisher for local SLAM
            kfPosePub.push_back(nh_ptr->advertise<sensor_msgs::PointCloud2>("/" + nodeName[i] + "/kf_pose", 1));
            slfKfCloudPub.push_back(nh_ptr->advertise<sensor_msgs::PointCloud2>("/" + nodeName[i] + "/slf_kf_cloud", 1));
            cloudInWPub.push_back(nh_ptr->advertise<sensor_msgs::PointCloud2>("/" + nodeName[i] + "/cloud_inW", 1));

            // Publisher for cooperative SLAM
            nbr_kf_pub_mtx.emplace_back();
            nbrKfCloudPub.push_back(nh_ptr->advertise<sensor_msgs::PointCloud2>("/" + nodeName[i] + "/nbr_kf_cloud", 1));
            nbr_odom_pub_mtx.emplace_back();
            nbrOdomPub.push_back(nh_ptr->advertise<sensor_msgs::PointCloud2>("/" + nodeName[i] + "/nbr_odom_cloud", 1));

            // Set the mission time to all the units (in practice this param can be broadcast over the network at the beginning)
            nh_ptr->setParam("/" + nodeName[i] + "/mission_duration", mission_duration);
            missionDurRemained.push_back(nh_ptr->advertise<std_msgs::Duration>("/" + nodeName[i] + "/mission_duration_remained", 1));
        }
    }

    // Update the states
    nodeOdom = msg->node_odom;
    assert(nodeOdom.size() == Nnodes);

    // Update the distance
    int range_idx = 0;
    for(int i = 0; i < Nnodes; i++)
    {
        for(int j = i+1; j < Nnodes; j++)
        {   
            linkMat(i,j) = msg->range[range_idx];
            linkMat(j,i) = linkMat(i,j);
            range_idx++;
        }
    }

    // Update the status
    for(int i = 0; i < Nnodes; i++)
    {
        Vector3d vel(nodeOdom[i].twist.twist.linear.x,
                     nodeOdom[i].twist.twist.linear.y,
                     nodeOdom[i].twist.twist.linear.z);

        if (nodeOdom[i].pose.pose.position.z > 0.5 && vel.norm() > 0.1)
            nodeStatus[i] = "on_air";

        if (nodeOdom[i].pose.pose.position.z < 0.1 && vel.norm() < 0.1)
            nodeStatus[i] = "on_ground";

        // Start counting mission if any velocity exceeds 0.05 m/s
        if (!mission_started && nodeRole[i] != "manager" && nodeStatus[i] == "on_air")
        {
            mission_started = true;
            mission_start_time = ros::Time::now();
            printf(KYEL "MISSION STARTED\n" RESET);
        }    
    }

    // Publish the message with dead or alive check
    rotors_comm::PPComTopology msg_ = *msg;
    msg_.node_alive.clear();
    for(bool doa : nodeAlive)
        msg_.node_alive.push_back(doa);
    ppcomDoAPub.publish(msg_);

    topo_mtx.unlock();

    // Publish the neighbour odom under line of sight
    for(int i = 0; i < Nnodes; i++)
    {
        CloudOdomPtr nbrOdom(new CloudOdom());
        for(int j = 0; j < Nnodes; j++)
        {
            // Skip if node is not alive or there is no line of sight
            if (!nodeAlive[j] || linkMat(i, j) <= 0.0)
                continue;

            PointOdom odom;
            
            // Node id
            odom.intensity = j;

            // Time stamp
            odom.t  = nodeOdom[j].header.stamp.toSec();
            
            // Position
            odom.x  = nodeOdom[j].pose.pose.position.x;
            odom.y  = nodeOdom[j].pose.pose.position.y;
            odom.z  = nodeOdom[j].pose.pose.position.z;

            // Quaternion
            odom.qx = nodeOdom[j].pose.pose.orientation.x;
            odom.qy = nodeOdom[j].pose.pose.orientation.y;
            odom.qz = nodeOdom[j].pose.pose.orientation.z;
            odom.qw = nodeOdom[j].pose.pose.orientation.w;

            // Linear Velocity
            odom.vx = nodeOdom[j].twist.twist.linear.x;
            odom.vy = nodeOdom[j].twist.twist.linear.y;
            odom.vz = nodeOdom[j].twist.twist.linear.z;

            // Angular Velocity
            odom.wx = nodeOdom[j].twist.twist.angular.x;
            odom.wy = nodeOdom[j].twist.twist.angular.y;
            odom.wz = nodeOdom[j].twist.twist.angular.z;

            nbrOdom->push_back(odom);
        }
        
        Util::publishCloud(nbrOdomPub[i], *nbrOdom, msg->header.stamp, string("world"));
    }

    // Update the link visualization
    vizAid.marker.points.clear();
    vizAid.marker.colors.clear();
    for(int i = 0; i < Nnodes; i++)
    {
        if (nodeOdom[i].pose.covariance[0] < 0.0)
            continue;

        for(int j = i+1; j < Nnodes; j++)
        {
            if (nodeOdom[j].pose.covariance[0] < 0.0)
                continue;
            
            // If either node is dead:
            if( !nodeAlive[i] || !nodeAlive[j] )
            {
                // printf("Node %s or %s is dead\n", nodeName[i], nodeName[j]);
                vizAid.marker.points.push_back(nodeOdom[i].pose.pose.position);
                vizAid.marker.colors.push_back(dead_color);
                vizAid.marker.points.push_back(nodeOdom[j].pose.pose.position);
                vizAid.marker.colors.push_back(dead_color);
                continue;
            }
            
            // If there is line of sight
            if (linkMat(i, j) > 0.0)
            {
                // printf("Node %d, %d has line of sight\n", i, j);
                vizAid.marker.points.push_back(nodeOdom[i].pose.pose.position);
                vizAid.marker.colors.push_back(los_color);
                vizAid.marker.points.push_back(nodeOdom[j].pose.pose.position);
                vizAid.marker.colors.push_back(los_color);
            }
            else
            {
                // printf("Node %d, %d lacks line of sight\n", i, j);
                vizAid.marker.points.push_back(nodeOdom[i].pose.pose.position);
                vizAid.marker.colors.push_back(nlos_color);
                vizAid.marker.points.push_back(nodeOdom[j].pose.pose.position);
                vizAid.marker.colors.push_back(nlos_color);
            }
        }
    }

    vizAid.rosPub.publish(vizAid.marker);

    // Publish mission time for each namespace
    if (mission_started)
    {
        for(int node_idx = 0; node_idx < Nnodes; node_idx++)
        {
            std_msgs::Duration msg;
            msg.data = ros::Duration(mission_duration) - (ros::Time::now() - mission_start_time);
            missionDurRemained[node_idx].publish(msg);
        }
    }

    // Shutdown everything when mission time elapses
    if (mission_started && ((ros::Time::now() - mission_start_time).toSec() > mission_duration))
    {
        for(int node_idx = 0; node_idx < Nnodes; node_idx++)
        {
            if (nodeAlive[node_idx])
                printf("Mission time over. Shutting down node %d, %s (role %s).\n",
                        node_idx, nodeName[node_idx].c_str(), nodeRole[node_idx].c_str());
            
            // Command the drones to fall
            unicon::Stop stop;
            stop.request.message = KRED "Collision happens over " + nodeName[node_idx] + ". Control Stopped!" RESET;
            ros::service::call("/" + nodeName[node_idx] + "/stop", stop);

            // Set the state of the drone as dead
            nodeAlive[node_idx] = false;
        }

        mission_finished = true;
    }

    // printf("ppcomcb: %f\n", tt_ppcom.Toc());
}

void ScoreTextCallback(const RosVizMarker::Ptr &msg)
{
    RosVizMarker msg_ = *msg;

    double elapsed_time = mission_started ? (ros::Time::now() - mission_start_time).toSec() : 0;
    string time_report = myprintf("%.3f s / %.3f s", elapsed_time, mission_duration);
    
    msg_.text += ". Time: " + time_report;

    if (elapsed_time / mission_duration < 0.5)
    {
        msg_.color.r = 0;
        msg_.color.g = 1;
        msg_.color.b = 0;
    }
    else if (elapsed_time / mission_duration < 0.8)
    {
        msg_.color.r = 1;
        msg_.color.g = 1;
        msg_.color.b = 0;
    }
    else
    {
        msg_.color.r = 1;
        msg_.color.g = 0;
        msg_.color.b = 1;
    }
    
    scoreTextPub.publish(msg_);
}

void ScoreCallback(const sensor_msgs::PointCloud::Ptr &msg)
{
    std::lock_guard<std::mutex> lock(score_mtx);
    scoreMsg = *msg;
}

void WriteScoreLog()
{
    sensor_msgs::PointCloud scoreReport;
    
    {
        std::lock_guard<std::mutex> lock(score_mtx);
        scoreReport = scoreMsg;
    }

    std::ofstream log_file;
    log_file.open(log_dir + "/score_" + std::to_string(scoreReport.header.stamp.sec) + "_" + std::to_string(scoreReport.header.stamp.nsec) + ".csv");
    log_file.precision(std::numeric_limits<double>::digits10 + 1);
    
    log_file << "x, y, z, nx, ny, nz, score, idx" << endl;

    // printf(KYEL "Logging the score...\n" RESET);

    for(int i = 0; i < scoreReport.points.size(); i++)
    {
        geometry_msgs::Point32 &point = scoreReport.points[i];
        
        log_file << point.x << ", "
                 << point.y << ", "
                 << point.z << ", "
                 << scoreReport.channels[0].values[i] << ", "
                 << scoreReport.channels[1].values[i] << ", "
                 << scoreReport.channels[2].values[i] << ", "
                 << scoreReport.channels[3].values[i] << ", "
                 << scoreReport.channels[4].values[i] << endl;
    }

    // printf(KYEL "Logging the score completed.\n" RESET);

}

void logCallback(const ros::TimerEvent& event)
{
    if (scoreMsg.points.size() > 0)
        WriteScoreLog();

    if (mission_finished)
    {
        printf(KRED "Exitting:\n" RESET);
        ros::shutdown();
    }
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
    // Load gazebo
    gazebo::client::setup(argc, argv);

    // Create a gazebo node to subscribe to gazebo environments
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Listen to Gazebo contact topic
    gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/physics/contacts", ContactCallback);

    // Create a ros node to subscribe to ros environment
    ros::init(argc, argv, "MissionManager");
    ros::NodeHandle nh("~");
    nh_ptr = boost::make_shared<ros::NodeHandle>(nh);

    // Subscribe to the ppcom topology
    printf(KGRN "Subscribing to ppcom_topology\n" RESET);
    ros::Subscriber ppcomSub = nh_ptr->subscribe("/ppcom_topology", 1, PPComCallback);

    // Advertise a ppcom topology with dead or alive status
    printf(KGRN "Subscribing to ppcom_topology\n" RESET);
    ppcomDoAPub = nh_ptr->advertise<rotors_comm::PPComTopology>("/ppcom_topology_doa", 1);

    // Subcribe to the total score status
    scoreTextSub = nh_ptr->subscribe("/viz_score_totalled", 1, ScoreTextCallback);
    scoreTextPub = nh_ptr->advertise<RosVizMarker>("/viz_score_totalled_time", 1);

    // Subscribe to the score topic from the gcs
    scoreSub = nh_ptr->subscribe("/score_eval", 1, ScoreCallback);
    
    // world = gazebo::physics::get_world("default");

    // Get the parameters

    // Mission time
    nh_ptr->getParam("mission_duration", mission_duration);

    // Log directory
    nh_ptr->getParam("log_dir", log_dir);
    boost::filesystem::create_directories(log_dir);
    boost::filesystem::remove_all(log_dir);
    boost::filesystem::create_directories(log_dir);

    // Transform from body to lidar
    vector<double> T_B_S_ = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    nh_ptr->getParam("T_B_S", T_B_S_);
    tf_B_S = myTf<double>(Quaternd(T_B_S_[3], T_B_S_[4], T_B_S_[5], T_B_S_[6]),
                          Vector3d(T_B_S_[0], T_B_S_[1], T_B_S_[2]));
    printf("tf_B_S:\n");
    cout << tf_B_S.tfMat() << endl;

    vector<double> T_S_L_ = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    nh_ptr->getParam("T_S_L", T_S_L_);
    tf_S_L = myTf<double>(Quaternd(T_S_L_[3], T_S_L_[4], T_S_L_[5], T_S_L_[6]),
                          Vector3d(T_S_L_[0], T_S_L_[1], T_S_L_[2]));
    printf("tf_S_L:\n");
    cout << tf_S_L.tfMat() << endl;

    // SLAM parameters
    nh_ptr->param("kf_knn_num", kf_knn_num);
    nh_ptr->param("kf_min_dis", kf_min_dis);
    nh_ptr->param("kf_min_ang", kf_min_ang);
    nh_ptr->param("kf_voxsize", kf_voxsize);
    printf(KGRN "SLAM params: \n" RESET);
    printf(KGRN "\tkf_knn_num: %d\n" RESET, kf_knn_num);
    printf(KGRN "\tkf_min_dis: %06.3f\n" RESET, kf_min_dis);
    printf(KGRN "\tkf_min_ang: %06.3f\n" RESET, kf_min_ang);
    printf(KGRN "\tkf_voxsize: %06.3f\n" RESET, kf_voxsize);
    

    // Initialize visualization
    los_color.r  = 0.0; los_color.g  = 1.0;  los_color.b  = 0.5; los_color.a  = 1.0;
    nlos_color.r = 1.0; nlos_color.g = 0.65; nlos_color.b = 0.0; nlos_color.a = 0.5;
    dead_color.r = 0.2; dead_color.g = 0.2;  dead_color.b = 0.2; dead_color.r = 1.0;

    vizAid.rosPub = nh_ptr->advertise<RosVizMarker>("/topology_marker", 1);
    vizAid.marker.header.frame_id = "world";
    vizAid.marker.ns       = "loop_marker";
    vizAid.marker.type     = visualization_msgs::Marker::LINE_LIST;
    vizAid.marker.action   = visualization_msgs::Marker::ADD;
    vizAid.marker.pose.orientation.w = 1.0;
    vizAid.marker.lifetime = ros::Duration(0);
    vizAid.marker.id       = 0;

    vizAid.marker.scale.x = 0.15;
    vizAid.marker.scale.y = 0.15;
    vizAid.marker.scale.z = 0.15;

    vizAid.marker.color.r = 0.0;
    vizAid.marker.color.g = 1.0;
    vizAid.marker.color.b = 1.0;
    vizAid.marker.color.a = 1.0;

    vizAid.color = nlos_color;

    vizAid.marker.points.clear();
    vizAid.marker.colors.clear();

    // Create a log timer
    ros::Timer timer = nh.createTimer(ros::Duration(1.0), logCallback);

    ros::MultiThreadedSpinner spinner(0);
    spinner.spin();

    // Write down the score
    WriteScoreLog();

    // Make sure to shut everything down.
    gazebo::client::shutdown();

}