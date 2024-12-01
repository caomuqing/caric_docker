
#include <plan_manage/planner_manager.h>
#include <exploration_manager/fast_exploration_manager.h>
#include <traj_utils/planning_visualization.h>

#include <exploration_manager/fast_exploration_fsm.h>
#include <exploration_manager/expl_data.h>
#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>
#include <plan_env/bounding_box.h>

#include <std_msgs/String.h>

using Eigen::Vector4d;

namespace fast_planner
{
  void FastExplorationFSM::init(ros::NodeHandle &nh)
  {
    fp_.reset(new FSMParam);
    fd_.reset(new FSMData);

    /*  Fsm param  */
    nh.param("fsm/thresh_replan1", fp_->replan_thresh1_, -1.0);
    nh.param("fsm/thresh_replan2", fp_->replan_thresh2_, -1.0);
    nh.param("fsm/thresh_replan3", fp_->replan_thresh3_, -1.0);
    nh.param("fsm/replan_time", fp_->replan_time_, -1.0);

    nh.param("drone_num", drone_num_, 5);

    /* Initialize main modules */
    expl_manager_.reset(new FastExplorationManager);
    expl_manager_->initialize(nh);
    visualization_.reset(new PlanningVisualization(nh));

    frontier_vis_size_ = expl_manager_->sdf_map_->getResolution();
    drone_name_ = expl_manager_->ep_->drone_name_;

    planner_manager_ = expl_manager_->planner_manager_;
    state_ = EXPL_STATE::INIT;
    fd_->have_odom_ = false;
    fd_->state_str_ = {"INIT", "WAIT_TRIGGER", "INSPECT_WAIT_FRONTIER", "PLAN_TRAJ", "PUB_TRAJ", "EXEC_TRAJ", "FINISH", "FIND_NEXT_BOX"};
    fd_->static_state_ = true;
    fd_->trigger_ = false;

    /* Ros sub, pub and timer */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &FastExplorationFSM::FSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &FastExplorationFSM::safetyCallback, this);
    frontier_timer_ = nh.createTimer(ros::Duration(0.5), &FastExplorationFSM::frontierCallback, this);
    inspect_frontier_update_timer_ = nh.createTimer(ros::Duration(0.05), &FastExplorationFSM::inspectFrontierUpdateCallback, this);

    trigger_sub_ = nh.subscribe("/move_base_simple/goal", 1, &FastExplorationFSM::triggerCallback, this);
    odom_sub_ = nh.subscribe("/odom_world", 1, &FastExplorationFSM::odometryCallback, this);
    gimbal_sub_ = nh.subscribe("/gimbal", 1, &FastExplorationFSM::gimbalCallback, this);
    all_start_pos_sub_ = nh.subscribe("/odometry/" + drone_name_, 1, &FastExplorationFSM::allStartPosCallback, this);
    all_start_flag_ = false;

    replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
    new_pub_ = nh.advertise<std_msgs::Empty>("/planning/new", 10);
    bspline_pub_ = nh.advertise<bspline::Bspline>("/planning/bspline", 10);
    take_off_pub_ = nh.advertise<std_msgs::Empty>("/take_off", 10);

    for (size_t i = 0; i < 10; i++)
    {
      is_rescue[i] = false;
    }
    rescue_ = true;
  }

  void FastExplorationFSM::FSMCallback(const ros::TimerEvent &e)
  {
    ROS_INFO_STREAM_THROTTLE(1.0, drone_name_.c_str() << "[FSM]: state: " << fd_->state_str_[int(state_)]);
    if (!isInspectDrone())
    {
      switch (state_)
      {
      case INIT:
      {
        // Wait for odometry ready
        if (!fd_->have_odom_)
        {
          ROS_WARN_THROTTLE(1.0, "%s no odom.", drone_name_.c_str());
          return;
        }
        // Wait for all start pos
        if (!all_start_flag_)
        {
          ROS_WARN_THROTTLE(1.0, "%s no all start pos.", drone_name_.c_str());
          return;
        }
        if (!team_allocate_)
        {
          ROS_ERROR("START TEAM ALLOCATE!!!!!!!!!!!!!");
          expl_manager_->sdf_map_->bbx_->teamAllocate();
          team_allocate_ = true;
        }
        // Box allocate for explorers
        if (!init_box_cvrp_)
        {
          expl_manager_->findGlobalTourofBox();
          init_box_cvrp_ = true;
        }
        take_off_pub_.publish(std_msgs::Empty());
        // Go to wait trigger when odom is ok
        transitState(WAIT_TRIGGER, "FSM");
        break;
      }

      case WAIT_TRIGGER:
      {
        // Do nothing but wait for trigger
        ROS_WARN_THROTTLE(1.0, "%s wait for trigger.", drone_name_.c_str());
        break;
      }

      case FINISH:
      {
        ROS_INFO_THROTTLE(1.0, "%s finish exploration.", drone_name_.c_str());
        break;
      }

      case PLAN_TRAJ:
      {
        if (fd_->static_state_)
        {
          // Plan from static state (hover)
          fd_->start_pt_ = fd_->odom_pos_;
          fd_->start_vel_ = fd_->odom_vel_;
          fd_->start_acc_.setZero();

          fd_->start_yaw_(0) = fd_->odom_yaw_;
          fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;
          fd_->start_pitch_ = Vector3d(fd_->gimbal_pitch_(0), 0.0, 0.0);
        }
        else
        {
          // Replan from non-static state, starting from 'replan_time' seconds later
          LocalTrajData *info = &planner_manager_->local_data_;
          double t_r = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;

          fd_->start_pt_ = info->position_traj_.evaluateDeBoorT(t_r);
          fd_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
          fd_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_r);
          fd_->start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
          fd_->start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_r)[0];
          fd_->start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
          if (expl_manager_->ep_->need_pitch_)
          {
            fd_->start_pitch_(0) = info->pitch_traj_.evaluateDeBoorT(t_r)[0];
            fd_->start_pitch_(1) = info->pitchdot_traj_.evaluateDeBoorT(t_r)[0];
            fd_->start_pitch_(2) = info->pitchdotdot_traj_.evaluateDeBoorT(t_r)[0];
          }
        }

        // Inform traj_server the replanning
        replan_pub_.publish(std_msgs::Empty());
        if (rescue_)
        {
          int res = callFindInspectPlanner();
          if (res == 1)
          {
            // success
            transitState(PUB_TRAJ, "FSM");
          }
          else if (res == 2)
          {
            rescue_ = false;
            ROS_ERROR("%s rescue finish ##############!!!!!!!!!!!!!!!", drone_name_.c_str());
          }
          else
          {
            ROS_ERROR("%s can't find inspect!!!!!!!!", drone_name_.c_str());
          }
        }
        else
        {
          if (find_gcs_)
          {
            int res = callFindGcsPlanner();
            if (res == 1)
            {
              // success
              transitState(PUB_TRAJ, "FSM");
            }
            else
            {
              ROS_ERROR("%s can't find gcs!!!!!!!!", drone_name_.c_str());
            }
          }
          else
          {
            if (!find_box_flag_)
            {
              // have frontier so exploration
              int res = callExplorationPlanner();
              if (res == SUCCEED)
              {
                transitState(PUB_TRAJ, "FSM");
              }
              else if (res == NO_FRONTIER)
              {
                // transitState(FINISH, "FSM");
                // fd_->static_state_ = true;
                // clearVisMarker();
                find_box_flag_ = true;
              }
              else if (res == FAIL)
              {
                // Still in PLAN_TRAJ state, keep replanning
                ROS_WARN("%s plan fail", drone_name_.c_str());
                fd_->static_state_ = true;
              }
            }
            else
            {
              // No frontier so find box
              int res = callFindBoxPlanner();
              if (res == 1)
              {
                // success
                transitState(PUB_TRAJ, "FSM");
              }
              else if (res == 2)
              {
                // find frontier
                find_box_flag_ = false;
              }
              else if (res == 3)
              {
                // all boxes are visited
                // transitState(FINISH, "FSM");
                find_gcs_ = true;
                fd_->static_state_ = true;
                find_box_flag_ = false;
              }
            }
          }
        }
        break;
      }

      case PUB_TRAJ:
      {
        double dt = (ros::Time::now() - fd_->newest_traj_.start_time).toSec();
        if (dt > 0)
        {
          bspline_pub_.publish(fd_->newest_traj_);
          fd_->static_state_ = false;
          transitState(EXEC_TRAJ, "FSM");

          thread vis_thread(&FastExplorationFSM::visualize, this);
          vis_thread.detach();
        }
        break;
      }

      case EXEC_TRAJ:
      {
        LocalTrajData *info = &planner_manager_->local_data_;
        double t_cur = (ros::Time::now() - info->start_time_).toSec();

        // Replan if traj is almost fully executed
        double time_to_end = info->duration_ - t_cur;
        if (time_to_end < fp_->replan_thresh1_)
        {
          transitState(PLAN_TRAJ, "FSM");
          ROS_WARN("%s Replan: traj fully executed=================================", drone_name_.c_str());
          return;
        }
        // Replan if next frontier to be visited is covered
        if (t_cur > fp_->replan_thresh2_ && expl_manager_->frontier_finder_->isFrontierCovered() && !rescue_)
        {
          transitState(PLAN_TRAJ, "FSM");
          ROS_WARN("%s Replan: cluster covered=====================================", drone_name_.c_str());
          return;
        }
        // Replan after some time
        if (t_cur > fp_->replan_thresh3_ && !classic_)
        {
          transitState(PLAN_TRAJ, "FSM");
          ROS_WARN("%s Replan: periodic call=======================================", drone_name_.c_str());
        }
        break;
      }
      }
    }
    else
    {
      switch (state_)
      {
      case INIT:
      {
        // Wait for odometry ready
        if (!fd_->have_odom_)
        {
          ROS_WARN_THROTTLE(1.0, "%s no odom.", drone_name_.c_str());
          return;
        }
        // Wait for all start pos
        if (!all_start_flag_)
        {
          ROS_WARN_THROTTLE(1.0, "%s no all start pos.", drone_name_.c_str());
          return;
        }
        if (!team_allocate_)
        {
          ROS_ERROR("START TEAM ALLOCATE!!!!!!!!!!!!!");
          expl_manager_->sdf_map_->bbx_->teamAllocate();
          team_allocate_ = true;
        }
        // Box allocate for explorers
        if (!init_box_cvrp_)
        {
          expl_manager_->findGlobalTourofBox();
          init_box_cvrp_ = true;
        }
        take_off_pub_.publish(std_msgs::Empty());
        // Go to wait trigger when odom is ok
        transitState(WAIT_TRIGGER, "FSM");
        break;
      }

      case WAIT_TRIGGER:
      {
        // Do nothing but wait for trigger
        ROS_WARN_THROTTLE(1.0, "%s wait for trigger.", drone_name_.c_str());
        break;
      }

      case INSPECT_WAIT_FRONTIER:
      {
        ROS_INFO_THROTTLE(1.0, "%s wait inspect frontier.", drone_name_.c_str());
        if (expl_manager_->ed_->frontiers_.size())
        {
          transitState(PLAN_TRAJ, "FSM");
        }
        break;
      }

      case PLAN_TRAJ:
      {
        if (fd_->static_state_)
        {
          // Plan from static state (hover)
          fd_->start_pt_ = fd_->odom_pos_;
          fd_->start_vel_ = fd_->odom_vel_;
          fd_->start_acc_.setZero();

          fd_->start_yaw_(0) = fd_->odom_yaw_;
          fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;
          fd_->start_pitch_ = Vector3d(fd_->gimbal_pitch_(0), 0.0, 0.0);
        }
        else
        {
          // Replan from non-static state, starting from 'replan_time' seconds later
          LocalTrajData *info = &planner_manager_->local_data_;
          double t_r = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;

          fd_->start_pt_ = info->position_traj_.evaluateDeBoorT(t_r);
          fd_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
          fd_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_r);
          fd_->start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
          fd_->start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_r)[0];
          fd_->start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
          if (expl_manager_->ep_->need_pitch_)
          {
            fd_->start_pitch_(0) = info->pitch_traj_.evaluateDeBoorT(t_r)[0];
            fd_->start_pitch_(1) = info->pitchdot_traj_.evaluateDeBoorT(t_r)[0];
            fd_->start_pitch_(2) = info->pitchdotdot_traj_.evaluateDeBoorT(t_r)[0];
          }
        }

        // Inform traj_server the replanning
        replan_pub_.publish(std_msgs::Empty());
        int res = callExplorationPlanner();
        if (res == SUCCEED)
        {
          transitState(PUB_TRAJ, "FSM");
        }
        else if (res == NO_FRONTIER)
        {
          transitState(INSPECT_WAIT_FRONTIER, "FSM");
          fd_->static_state_ = true;
        }
        else if (res == FAIL)
        {
          // Still in PLAN_TRAJ state, keep replanning
          ROS_WARN("%s plan fail", drone_name_.c_str());
          fd_->static_state_ = true;
        }
        break;
      }

      case PUB_TRAJ:
      {
        double dt = (ros::Time::now() - fd_->newest_traj_.start_time).toSec();
        if (dt > 0)
        {
          bspline_pub_.publish(fd_->newest_traj_);
          // if (expl_manager_->ep_->need_pitch_)
          //   bspline_pitch_pub_.publish(fd_->newest_traj_pitch_);
          fd_->static_state_ = false;
          transitState(EXEC_TRAJ, "FSM");

          thread vis_thread(&FastExplorationFSM::visualize, this);
          vis_thread.detach();
        }
        break;
      }

      case EXEC_TRAJ:
      {
        LocalTrajData *info = &planner_manager_->local_data_;
        double t_cur = (ros::Time::now() - info->start_time_).toSec();

        // Replan if traj is almost fully executed
        double time_to_end = info->duration_ - t_cur;
        if (time_to_end < fp_->replan_thresh1_)
        {
          transitState(PLAN_TRAJ, "FSM");
          ROS_WARN("%s Replan: traj fully executed=================================", drone_name_.c_str());
          return;
        }
        // Replan if next frontier to be visited is covered
        if (t_cur > fp_->replan_thresh2_ && expl_manager_->frontier_finder_->isFrontierCovered())
        {
          transitState(PLAN_TRAJ, "FSM");
          ROS_WARN("%s Replan: cluster covered=====================================", drone_name_.c_str());
          return;
        }
        // Replan after some time
        if (t_cur > fp_->replan_thresh3_ && !classic_)
        {
          transitState(PLAN_TRAJ, "FSM");
          ROS_WARN("%s Replan: periodic call=======================================", drone_name_.c_str());
        }
        break;
      }
      }
    }
  }

  int FastExplorationFSM::callFindInspectPlanner()
  {
    bool find_all_free = true;
    ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);

    for (int i = 0; i < expl_manager_->sdf_map_->bbx_->getDroneNum(); i++)
    {
      DroneInfo drone = expl_manager_->sdf_map_->bbx_->getDroneInfo(i);
      if (drone.leader == expl_manager_->sdf_map_->bbx_->getSlfDroneId())
      {
        if (expl_manager_->isLos(drone.start_pos, fd_->odom_pos_) && (drone.start_pos - fd_->odom_pos_).norm() <= 12.0)
        {
          is_rescue[i] = true;
        }
        if (!is_rescue[i])
        {
          inspect_pos_ = drone.start_pos;
          find_all_free = false;
          break;
        }
      }
    }

    if (find_all_free)
    {
      return 2;
    }
    ROS_ERROR("%s want to rescue !!!!!!!!!!!!!!!", drone_name_.c_str());
    double time_lb = min(8.0, (inspect_pos_ - fd_->odom_pos_).norm() * 1.2);
    int res = planner_manager_->kinodynamicReplan(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_, inspect_pos_, Vector3d(0, 0, 0), time_lb);

    if (res)
    {
      planner_manager_->planYaw(fd_->start_yaw_);

      auto info = &planner_manager_->local_data_;
      info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;

      bspline::Bspline bspline;
      bspline.order = planner_manager_->pp_.bspline_degree_;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;
      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      for (int i = 0; i < pos_pts.rows(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = pos_pts(i, 0);
        pt.y = pos_pts(i, 1);
        pt.z = pos_pts(i, 2);
        bspline.pos_pts.push_back(pt);
      }
      Eigen::VectorXd knots = info->position_traj_.getKnot();
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }
      Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
      for (int i = 0; i < yaw_pts.rows(); ++i)
      {
        double yaw = yaw_pts(i, 0);
        bspline.yaw_pts.push_back(yaw);
      }
      bspline.yaw_dt = info->yaw_traj_.getKnotSpan();

      if (expl_manager_->ep_->need_pitch_)
      {
        Eigen::MatrixXd pitch_pts = info->pitch_traj_.getControlPoint();
        for (int i = 0; i < pitch_pts.rows(); ++i)
        {
          double pitch = pitch_pts(i, 0);
          bspline.pitch_pts.push_back(pitch);
        }
        bspline.pitch_dt = info->pitch_traj_.getKnotSpan();
      }
      fd_->newest_traj_ = bspline;
    }
    return res;
  }

  int FastExplorationFSM::callFindGcsPlanner()
  {
    ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);

    double time_lb = min(8.0, (gcs_pos_ - fd_->odom_pos_).norm() * 1.2);
    int res = planner_manager_->kinodynamicReplan(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_, gcs_pos_, Vector3d(0, 0, 0), time_lb);

    if (res)
    {
      planner_manager_->planYaw(fd_->start_yaw_);

      auto info = &planner_manager_->local_data_;
      info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;

      bspline::Bspline bspline;
      bspline.order = planner_manager_->pp_.bspline_degree_;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;
      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      for (int i = 0; i < pos_pts.rows(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = pos_pts(i, 0);
        pt.y = pos_pts(i, 1);
        pt.z = pos_pts(i, 2);
        bspline.pos_pts.push_back(pt);
      }
      Eigen::VectorXd knots = info->position_traj_.getKnot();
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }
      Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
      for (int i = 0; i < yaw_pts.rows(); ++i)
      {
        double yaw = yaw_pts(i, 0);
        bspline.yaw_pts.push_back(yaw);
      }
      bspline.yaw_dt = info->yaw_traj_.getKnotSpan();

      if (expl_manager_->ep_->need_pitch_)
      {
        Eigen::MatrixXd pitch_pts = info->pitch_traj_.getControlPoint();
        for (int i = 0; i < pitch_pts.rows(); ++i)
        {
          double pitch = pitch_pts(i, 0);
          bspline.pitch_pts.push_back(pitch);
        }
        bspline.pitch_dt = info->pitch_traj_.getKnotSpan();
      }
      fd_->newest_traj_ = bspline;
    }
    return res;
  }

  int FastExplorationFSM::callFindBoxPlanner()
  {
    auto ft = expl_manager_->frontier_finder_;
    auto ed = expl_manager_->ed_;
    ft->searchFrontiers();
    ft->computeFrontiersToVisit();
    ft->getFrontiers(ed->frontiers_);
    if (ed->frontiers_.size() > 0)
      return 2; // find frontier then don't find box

    ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);
    vector<int> box_tour = expl_manager_->sdf_map_->bbx_->getBoxTour();
    bool have_visit = false;
    Vector3d box_center;
    for (auto box_id : box_tour)
    {
      if (expl_manager_->sdf_map_->bbx_->getBoxVisit(box_id))
        continue;
      have_visit = true;
      box_center = expl_manager_->sdf_map_->bbx_->getBoxCenter(box_id);
      break;
    }
    if (!have_visit)
      return 3; // all boxes are visited

    double time_lb = min(8.0, (gcs_pos_ - fd_->odom_pos_).norm() * 1.2);
    int res = planner_manager_->kinodynamicReplan(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_, box_center, Vector3d(0, 0, 0), time_lb);

    if (res)
    {
      planner_manager_->planYaw(fd_->start_yaw_);

      auto info = &planner_manager_->local_data_;
      info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;

      bspline::Bspline bspline;
      bspline.order = planner_manager_->pp_.bspline_degree_;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;
      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      for (int i = 0; i < pos_pts.rows(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = pos_pts(i, 0);
        pt.y = pos_pts(i, 1);
        pt.z = pos_pts(i, 2);
        bspline.pos_pts.push_back(pt);
      }
      Eigen::VectorXd knots = info->position_traj_.getKnot();
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }
      Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
      for (int i = 0; i < yaw_pts.rows(); ++i)
      {
        double yaw = yaw_pts(i, 0);
        bspline.yaw_pts.push_back(yaw);
      }
      bspline.yaw_dt = info->yaw_traj_.getKnotSpan();

      if (expl_manager_->ep_->need_pitch_)
      {
        Eigen::MatrixXd pitch_pts = info->pitch_traj_.getControlPoint();
        for (int i = 0; i < pitch_pts.rows(); ++i)
        {
          double pitch = pitch_pts(i, 0);
          bspline.pitch_pts.push_back(pitch);
        }
        bspline.pitch_dt = info->pitch_traj_.getKnotSpan();
      }
      fd_->newest_traj_ = bspline;
      // ROS_ERROR("hhhhhhwwww %.2f %.2f %.2f", box_center(0), box_center(1), box_center(2));
    }
    return res;
  }

  int FastExplorationFSM::callExplorationPlanner()
  {
    ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);

    int res = expl_manager_->planExploreMotion(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_,
                                               fd_->start_yaw_, fd_->start_pitch_);
    classic_ = false;

    if (res == SUCCEED)
    {
      auto info = &planner_manager_->local_data_;
      info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;

      bspline::Bspline bspline;
      bspline.order = planner_manager_->pp_.bspline_degree_;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;
      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      for (int i = 0; i < pos_pts.rows(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = pos_pts(i, 0);
        pt.y = pos_pts(i, 1);
        pt.z = pos_pts(i, 2);
        bspline.pos_pts.push_back(pt);
      }
      Eigen::VectorXd knots = info->position_traj_.getKnot();
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }
      Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
      for (int i = 0; i < yaw_pts.rows(); ++i)
      {
        double yaw = yaw_pts(i, 0);
        bspline.yaw_pts.push_back(yaw);
      }
      bspline.yaw_dt = info->yaw_traj_.getKnotSpan();

      if (expl_manager_->ep_->need_pitch_)
      {
        Eigen::MatrixXd pitch_pts = info->pitch_traj_.getControlPoint();
        for (int i = 0; i < pitch_pts.rows(); ++i)
        {
          double pitch = pitch_pts(i, 0);
          bspline.pitch_pts.push_back(pitch);
        }
        bspline.pitch_dt = info->pitch_traj_.getKnotSpan();
      }
      fd_->newest_traj_ = bspline;
    }
    return res;
  }

  void FastExplorationFSM::visualize()
  {
    // auto info = &planner_manager_->local_data_;
    // auto plan_data = &planner_manager_->plan_data_;
    // auto ed_ptr = expl_manager_->ed_;

    // // Draw updated box
    // // Vector3d bmin, bmax;
    // // planner_manager_->edt_environment_->sdf_map_->getUpdatedBox(bmin, bmax);
    // // visualization_->drawBox((bmin + bmax) / 2.0, bmax - bmin, Vector4d(0, 1, 0, 0.3), "updated_box", 0,
    // // 4);

    // // Draw frontier
    // static int last_ftr_num = 0;
    // for (int i = 0; i < ed_ptr->frontiers_.size(); ++i)
    // {
    //   visualization_->drawCubes(ed_ptr->frontiers_[i], frontier_vis_size_,
    //                             visualization_->getColor(double(i) / ed_ptr->frontiers_.size(), 0.4),
    //                             "frontier", i, 4);
    //   // visualization_->drawBox(ed_ptr->frontier_boxes_[i].first, ed_ptr->frontier_boxes_[i].second,
    //   //                         Vector4d(0.5, 0, 1, 0.3), "frontier_boxes", i, 4);
    // }

    // for (int i = ed_ptr->frontiers_.size(); i < last_ftr_num; ++i)
    // {
    //   visualization_->drawCubes({}, frontier_vis_size_, Vector4d(0, 0, 0, 1), "frontier", i, 4);
    //   // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
    //   // "frontier_boxes", i, 4);
    // }

    // last_ftr_num = ed_ptr->frontiers_.size();

    // // for (int i = 0; i < ed_ptr->dead_frontiers_.size(); ++i)
    // //   visualization_->drawCubes(ed_ptr->dead_frontiers_[i], 0.1, Vector4d(0, 0, 0, 0.5), "dead_frontier",
    // //                             i, 4);
    // // for (int i = ed_ptr->dead_frontiers_.size(); i < 5; ++i)
    // //   visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 0.5), "dead_frontier", i, 4);

    // // Draw global top viewpoints info
    // visualization_->drawSpheres(ed_ptr->points_, 1.0, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
    // // visualization_->drawLines(ed_ptr->global_tour_, 0.07, Vector4d(0, 0.5, 0, 1), "global_tour", 0, 6);
    // visualization_->drawLines(ed_ptr->points_, ed_ptr->views_, 0.05, Vector4d(0, 1, 0.5, 1), "view", 0, 6);
    // visualization_->drawLines(ed_ptr->points_, ed_ptr->averages_, 0.03, Vector4d(1, 0, 0, 1),
    //                           "point-average", 0, 6);

    // visualization_->drawLines(ed_ptr->global_cluster_tour_, 0.1, Vector4d(0, 0.5, 0, 1), "global_cluster_tour", 0, 6);

    // /* Draw local refined viewpoints info */

    // // visualization_->drawSpheres(ed_ptr->refined_points_, 1.0, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
    // // visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->refined_views_, 0.05,
    // //                           Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
    // // visualization_->drawLines(ed_ptr->refined_tour_, 0.07, Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
    // // visualization_->drawLines(ed_ptr->refined_views1_, ed_ptr->refined_views2_, 0.04, Vector4d(0, 0, 0, 1),
    // //                           "refined_view", 0, 6);
    // // visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->unrefined_points_, 0.05, Vector4d(1, 1, 0, 1),
    // //                           "refine_pair", 0, 6);
    // // for (int i = 0; i < ed_ptr->n_points_.size(); ++i)
    // //   visualization_->drawSpheres(ed_ptr->n_points_[i], 0.1,
    // //                               visualization_->getColor(double(ed_ptr->refined_ids_[i]) /
    // //                                                        ed_ptr->frontiers_.size()),
    // //                               "n_points", i, 6);
    // // for (int i = ed_ptr->n_points_.size(); i < 15; ++i)
    // //   visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 0, 1), "n_points", i, 6);

    // visualization_->drawSpheres(ed_ptr->local_tour_vis_, 1.0, Vector4d(1, 0, 0, 1), "local_tour_pts", 0, 6);
    // visualization_->drawLines(ed_ptr->local_tour_vis_, 0.1, Vector4d(0.7, 0, 0, 1), "local_tour_paths", 0, 6);

    // // Draw trajectory
    // // visualization_->drawSpheres({ ed_ptr->next_goal_ }, 0.3, Vector4d(0, 1, 1, 1), "next_goal", 0, 6);
    // visualization_->drawBspline(info->position_traj_, 0.1, Vector4d(1.0, 0.0, 0.0, 1), false, 0.15,
    //                             Vector4d(1, 1, 0, 1));
    // // visualization_->drawSpheres(plan_data->kino_path_, 0.1, Vector4d(1, 0, 1, 1), "kino_path", 0, 0);
    // // visualization_->drawLines(ed_ptr->path_next_goal_, 0.05, Vector4d(0, 1, 1, 1), "next_goal", 1, 6);
  }

  void FastExplorationFSM::clearVisMarker()
  {
    // visualization_->drawSpheres({}, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
    // visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "global_tour", 0, 6);
    // visualization_->drawSpheres({}, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
    // visualization_->drawLines({}, {}, 0.05, Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
    // visualization_->drawLines({}, 0.07, Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
    // visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 1, 1), "B-Spline", 0, 0);

    // visualization_->drawLines({}, {}, 0.03, Vector4d(1, 0, 0, 1), "current_pose", 0, 6);
  }

  void FastExplorationFSM::inspectFrontierUpdateCallback(const ros::TimerEvent &e)
  {
    if (!expl_manager_->ep_->is_inspect_)
      return;

    inspect_frontier_update_timer_.stop();
    // always update inspect frontier
    if (state_ != WAIT_TRIGGER && state_ != FINISH && state_ != INIT)
    {
      auto ft = expl_manager_->frontier_finder_;
      auto ed = expl_manager_->ed_;
      ft->updateVisitCells(fd_->odom_pos_, fd_->gimbal_pitch_(0), fd_->odom_yaw_, ed->frontiers_, ed->frontier_boxes_);
      // ROS_ERROR("%s update inspect cells",drone_name_.c_str());
    }
    inspect_frontier_update_timer_.start();
  }

  void FastExplorationFSM::frontierCallback(const ros::TimerEvent &e)
  {
    static int delay = 0;
    if (++delay < 5)
      return;

    if (state_ == WAIT_TRIGGER || state_ == FINISH || state_ == INSPECT_WAIT_FRONTIER)
    {
      auto ft = expl_manager_->frontier_finder_;
      auto ed = expl_manager_->ed_;
      if (expl_manager_->ep_->is_inspect_)
      {
        ft->searchInspectFrontiers();
        ft->computeInspectFrontiersToVisit();
        // bool neighbor;
        // ft->clusterFrontiers(fd_->odom_pos_, neighbor);
      }
      else
      {
        ft->searchFrontiers();
        ft->computeFrontiersToVisit();
      }
      ft->getFrontiers(ed->frontiers_);
      ft->getFrontierBoxes(ed->frontier_boxes_);
      ft->updateFrontierCostMatrix();

      // Draw box tour info
      // for (int i = 0; i < ed->box_tour.size(); i++)
      // {
      //   visualization_->drawLines(ed->box_tour[i], 0.5, Vector4d(0.5, 0, 0, 1), "box_tour_" + to_string(i), 0, 6);
      // }

      // // Draw frontier and bounding box
      // for (int i = 0; i < ed->frontiers_.size(); ++i)
      // {
      //   visualization_->drawCubes(ed->frontiers_[i], frontier_vis_size_,
      //                             visualization_->getColor(double(i) / ed->frontiers_.size(), 0.4),
      //                             "frontier", i, 4);
      //   // visualization_->drawBox(ed->frontier_boxes_[i].first, ed->frontier_boxes_[i].second,
      //   // Vector4d(0.5, 0, 1, 0.3),
      //   //                         "frontier_boxes", i, 4);
      // }

      // for (int i = ed->frontiers_.size(); i < 50; ++i)
      // {
      //   visualization_->drawCubes({}, frontier_vis_size_, Vector4d(0, 0, 0, 1), "frontier", i, 4);
      //   // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
      //   // "frontier_boxes", i, 4);
      // }
    }

    // if (!fd_->static_state_)
    // {
    //   static double astar_time = 0.0;
    //   static int astar_num = 0;
    //   auto t1 = ros::Time::now();

    //   planner_manager_->path_finder_->reset();
    //   planner_manager_->path_finder_->setResolution(0.4);
    //   if (planner_manager_->path_finder_->search(fd_->odom_pos_, Vector3d(-5, 0, 1)))
    //   {
    //     auto path = planner_manager_->path_finder_->getPath();
    //     visualization_->drawLines(path, 0.05, Vector4d(1, 0, 0, 1), "astar", 0, 6);
    //     auto visit = planner_manager_->path_finder_->getVisited();
    //     visualization_->drawCubes(visit, 0.3, Vector4d(0, 0, 1, 0.4), "astar-visit", 0, 6);
    //   }
    //   astar_num += 1;
    //   astar_time = (ros::Time::now() - t1).toSec();
    //   ROS_WARN("Average astar time: %lf", astar_time);
    // }
  }
  void FastExplorationFSM::allStartPosCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    if (msg->child_frame_id == "gcs/base_link")
    {
      gcs_pos_(0) = msg->pose.pose.position.x;
      gcs_pos_(1) = msg->pose.pose.position.y;
      gcs_pos_(2) = msg->pose.pose.position.z;
      // ROS_ERROR("GCS pos: %f %f %f", gcs_pos_(0), gcs_pos_(1), gcs_pos_(2));
    }

    if (all_start_flag_)
      return;

    vector<string> node_id = {"jurong", "raffles", "sentosa", "changi", "nanyang"};
    static vector<Vector3d> node_pos;
    static int drone_id = 0;
    if (drone_num_ == 5)
    {
      if (msg->child_frame_id == "jurong/base_link" && drone_id == 0)
      {
        drone_id++;
        node_pos.push_back(Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0));
      }
      else if (msg->child_frame_id == "raffles/base_link" && drone_id == 1)
      {
        drone_id++;
        node_pos.push_back(Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0));
      }
      else if (msg->child_frame_id == "sentosa/base_link" && drone_id == 2)
      {
        drone_id++;
        node_pos.push_back(Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0));
      }
      else if (msg->child_frame_id == "changi/base_link" && drone_id == 3)
      {
        drone_id++;
        node_pos.push_back(Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0));
      }
      else if (msg->child_frame_id == "nanyang/base_link" && drone_id == 4)
      {
        drone_id++;
        node_pos.push_back(Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0));
        expl_manager_->sdf_map_->bbx_->setDroneStartPos(node_id, node_pos);
        all_start_flag_ = true;
      }
    }

    if (drone_num_ == 3)
    {
      if (msg->child_frame_id == "jurong/base_link" && drone_id == 0)
      {
        drone_id = 2;
        node_pos.push_back(Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0));
        node_pos.push_back(Vector3d(-100.0, -100.0, -100.0));
      }
      else if (msg->child_frame_id == "sentosa/base_link" && drone_id == 2)
      {
        drone_id = 3;
        node_pos.push_back(Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0));
      }
      else if (msg->child_frame_id == "changi/base_link" && drone_id == 3)
      {
        drone_id++;
        node_pos.push_back(Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0));
        node_pos.push_back(Vector3d(-100.0, -100.0, -100.0));
        expl_manager_->sdf_map_->bbx_->setDroneStartPos(node_id, node_pos);
        all_start_flag_ = true;
      }
    }
  }

  void FastExplorationFSM::triggerCallback(const geometry_msgs::PoseStampedConstPtr &msg)
  {
    if (state_ != WAIT_TRIGGER)
      return;
    fd_->trigger_ = true;
    cout << "Triggered!" << endl;
    transitState(PLAN_TRAJ, "triggerCallback");
  }

  void FastExplorationFSM::safetyCallback(const ros::TimerEvent &e)
  {
    if (state_ == EXPL_STATE::EXEC_TRAJ)
    {
      // Check safety and trigger replan if necessary
      double dist;
      bool safe = planner_manager_->checkTrajCollision(dist);
      if (!safe)
      {
        ROS_WARN("Replan: collision detected==================================");
        transitState(PLAN_TRAJ, "safetyCallback");
      }
    }
  }

  void FastExplorationFSM::gimbalCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
  {
    fd_->gimbal_pitch_(0) = msg->twist.linear.y;
    fd_->gimbal_yaw_(0) = msg->twist.linear.z;
    fd_->gimbal_pitch_(1) = msg->twist.angular.y;
    fd_->gimbal_yaw_(1) = msg->twist.angular.z;
    // ROS_WARN("gimbal pitch: %f yaw: %f", fd_->gimbal_pitch_(0), fd_->gimbal_yaw_(0));
  }

  void FastExplorationFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    fd_->odom_pos_(0) = msg->pose.pose.position.x;
    fd_->odom_pos_(1) = msg->pose.pose.position.y;
    fd_->odom_pos_(2) = msg->pose.pose.position.z;

    fd_->odom_vel_(0) = msg->twist.twist.linear.x;
    fd_->odom_vel_(1) = msg->twist.twist.linear.y;
    fd_->odom_vel_(2) = msg->twist.twist.linear.z;

    fd_->odom_orient_.w() = msg->pose.pose.orientation.w;
    fd_->odom_orient_.x() = msg->pose.pose.orientation.x;
    fd_->odom_orient_.y() = msg->pose.pose.orientation.y;
    fd_->odom_orient_.z() = msg->pose.pose.orientation.z;

    Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
    fd_->odom_yaw_ = atan2(rot_x(1), rot_x(0));

    fd_->have_odom_ = true;
  }

  bool FastExplorationFSM::isInspectDrone()
  {
    return expl_manager_->ep_->is_inspect_;
  }

  void FastExplorationFSM::transitState(EXPL_STATE new_state, string pos_call)
  {
    int pre_s = int(state_);
    state_ = new_state;
    cout << "[" + pos_call + "]: from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)]
         << endl;
  }
} // namespace fast_planner
