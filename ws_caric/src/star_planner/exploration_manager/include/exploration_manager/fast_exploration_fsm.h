#ifndef _FAST_EXPLORATION_FSM_H_
#define _FAST_EXPLORATION_FSM_H_

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/Marker.h>
#include <rotors_comm/PPComTopology.h>

#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <thread>

using Eigen::Vector3d;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

namespace fast_planner
{
  class FastPlannerManager;
  class FastExplorationManager;
  class PlanningVisualization;
  struct FSMParam;
  struct FSMData;

  enum EXPL_STATE
  {
    INIT,
    WAIT_TRIGGER,
    INSPECT_WAIT_FRONTIER,
    PLAN_TRAJ,
    PUB_TRAJ,
    EXEC_TRAJ,
    FINISH,
    FIND_NEXT_BOX
  };

  class FastExplorationFSM
  {
  private:
    /* planning utils */
    shared_ptr<FastPlannerManager> planner_manager_;
    shared_ptr<FastExplorationManager> expl_manager_;
    shared_ptr<PlanningVisualization> visualization_;

    shared_ptr<FSMParam> fp_;
    shared_ptr<FSMData> fd_;
    EXPL_STATE state_;

    bool classic_;

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_, frontier_timer_, inspect_frontier_update_timer_;
    ros::Subscriber trigger_sub_, odom_sub_, gimbal_sub_, all_start_pos_sub_;
    ros::Publisher replan_pub_, new_pub_, bspline_pub_, take_off_pub_;

    double frontier_vis_size_;
    string drone_name_;
    bool all_start_flag_;
    bool init_box_cvrp_ = false;
    bool team_allocate_ = false;
    bool find_box_flag_ = false;
    bool find_gcs_ = false;
    bool rescue_ = false;
    Vector3d gcs_pos_, inspect_pos_;
    bool is_rescue[10];
    int drone_num_;

    /* helper functions */
    int callExplorationPlanner();
    int callFindBoxPlanner();
    int callFindGcsPlanner();
    int callFindInspectPlanner();
    void transitState(EXPL_STATE new_state, string pos_call);

    /* ROS functions */
    void FSMCallback(const ros::TimerEvent &e);
    void safetyCallback(const ros::TimerEvent &e);
    void frontierCallback(const ros::TimerEvent &e);
    void inspectFrontierUpdateCallback(const ros::TimerEvent &e);
    void triggerCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void gimbalCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void allStartPosCallback(const nav_msgs::Odometry::ConstPtr &msg);
    bool isInspectDrone();
    void visualize();
    void clearVisMarker();

  public:
    FastExplorationFSM(/* args */)
    {
    }
    ~FastExplorationFSM()
    {
    }

    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace fast_planner

#endif