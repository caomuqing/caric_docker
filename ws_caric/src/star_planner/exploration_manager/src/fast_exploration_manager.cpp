// #include <fstream>
#include <exploration_manager/fast_exploration_manager.h>
#include <thread>
#include <iostream>
#include <fstream>
#include <lkh_tsp_solver/lkh_interface.h>
#include <lkh_mtsp_solver/SolveMTSP.h>
#include <active_perception/graph_node.h>
#include <active_perception/graph_search.h>
#include <active_perception/perception_utils.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>
#include <plan_env/bounding_box.h>
#include <active_perception/frontier_finder.h>
#include <plan_manage/planner_manager.h>

#include <exploration_manager/expl_data.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

using namespace Eigen;

namespace fast_planner
{
  // SECTION interfaces for setup and query

  FastExplorationManager::FastExplorationManager()
  {
  }

  FastExplorationManager::~FastExplorationManager()
  {
    ViewNode::astar_.reset();
    ViewNode::caster_.reset();
    ViewNode::map_.reset();
  }

  void FastExplorationManager::initialize(ros::NodeHandle &nh)
  {
    planner_manager_.reset(new FastPlannerManager);
    planner_manager_->initPlanModules(nh);
    edt_environment_ = planner_manager_->edt_environment_;
    sdf_map_ = edt_environment_->sdf_map_;
    frontier_finder_.reset(new FrontierFinder(edt_environment_, nh));
    // view_finder_.reset(new ViewFinder(edt_environment_, nh));

    ed_.reset(new ExplorationData);
    ep_.reset(new ExplorationParam);

    nh.param("exploration/refine_local", ep_->refine_local_, true);
    nh.param("exploration/refined_num", ep_->refined_num_, -1);
    nh.param("exploration/refined_radius", ep_->refined_radius_, -1.0);
    nh.param("exploration/top_view_num", ep_->top_view_num_, -1);
    nh.param("exploration/max_decay", ep_->max_decay_, -1.0);
    nh.param("exploration/tsp_dir", ep_->tsp_dir_, string("null"));
    nh.param("exploration/mtsp_dir", ep_->mtsp_dir_, string("null"));
    nh.param("exploration/relax_time", ep_->relax_time_, 1.0);
    nh.param("exploration/drone_name", ep_->drone_name_, string("null"));
    nh.param("exploration/is_inspect", ep_->is_inspect_, false);

    nh.param("exploration/use_history", ep_->use_history_, false);
    nh.param("exploration/feature_max_dist", ep_->feature_max_dist_, -1.0);
    nh.param("exploration/inertial_cost_offset", ep_->inertial_cost_offset_, -1.0);
    nh.param("exploration/need_pitch", ep_->need_pitch_, false);

    nh.param("exploration/vm", ViewNode::vm_, -1.0);
    nh.param("exploration/am", ViewNode::am_, -1.0);
    nh.param("exploration/yd", ViewNode::yd_, -1.0);
    nh.param("exploration/ydd", ViewNode::ydd_, -1.0);
    nh.param("exploration/w_dir", ViewNode::w_dir_, -1.0);

    ViewNode::astar_.reset(new Astar);
    ViewNode::astar_->init(nh, edt_environment_);
    ViewNode::map_ = sdf_map_;

    double resolution_ = sdf_map_->getResolution();
    Eigen::Vector3d origin, size;
    sdf_map_->getRegion(origin, size);
    ViewNode::caster_.reset(new RayCaster);
    ViewNode::caster_->setParams(resolution_, origin);

    planner_manager_->path_finder_->lambda_heu_ = 1.0;
    // planner_manager_->path_finder_->max_search_time_ = 0.05;
    planner_manager_->path_finder_->max_search_time_ = 1.0;

    acvrp_client_ = nh.serviceClient<lkh_mtsp_solver::SolveMTSP>("/solve_acvrp_" + ep_->drone_name_, true);

    // Initialize TSP par file
    ofstream par_file_ordinary(ep_->tsp_dir_ + "/" + ep_->drone_name_ + "_ordinary" + ".par");
    par_file_ordinary << "PROBLEM_FILE = " << ep_->tsp_dir_ << "/" << ep_->drone_name_ << "_ordinary"
                      << ".tsp\n";
    par_file_ordinary << "GAIN23 = NO\n";
    par_file_ordinary << "OUTPUT_TOUR_FILE =" << ep_->tsp_dir_ << "/" << ep_->drone_name_ << "_ordinary"
                      << ".txt\n";
    par_file_ordinary << "RUNS = 1\n";

    ofstream par_file_inertial(ep_->tsp_dir_ + "/" + ep_->drone_name_ + "_inertial" + ".par");
    par_file_inertial << "PROBLEM_FILE = " << ep_->tsp_dir_ << "/" << ep_->drone_name_ << "_inertial"
                      << ".tsp\n";
    par_file_inertial << "GAIN23 = NO\n";
    par_file_inertial << "OUTPUT_TOUR_FILE =" << ep_->tsp_dir_ << "/" << ep_->drone_name_ << "_inertial"
                      << ".txt\n";
    par_file_inertial << "RUNS = 1\n";

    ofstream par_file_cluster(ep_->tsp_dir_ + "/" + ep_->drone_name_ + "_cluster" + ".par");
    par_file_cluster << "PROBLEM_FILE = " << ep_->tsp_dir_ << "/" << ep_->drone_name_ << "_cluster"
                     << ".tsp\n";
    par_file_cluster << "GAIN23 = NO\n";
    par_file_cluster << "OUTPUT_TOUR_FILE =" << ep_->tsp_dir_ << "/" << ep_->drone_name_ << "_cluster"
                     << ".txt\n";
    par_file_cluster << "RUNS = 1\n";

    ofstream par_file_single(ep_->tsp_dir_ + "/" + ep_->drone_name_ + "_single" + ".par");
    par_file_single << "PROBLEM_FILE = " << ep_->tsp_dir_ << "/" << ep_->drone_name_ << "_single"
                    << ".tsp\n";
    par_file_single << "GAIN23 = NO\n";
    par_file_single << "OUTPUT_TOUR_FILE =" << ep_->tsp_dir_ << "/" << ep_->drone_name_ << "_single"
                    << ".txt\n";
    par_file_single << "RUNS = 1\n";
    // Analysis
    // ofstream fout;
    // fout.open("/home/boboyu/Desktop/RAL_Time/frontier.txt");
    // fout.close();
  }

  int FastExplorationManager::planExploreMotion(
      const Vector3d &pos, const Vector3d &vel, const Vector3d &acc,
      const Vector3d &yaw, const Vector3d &pitch)
  {
    ros::Time t1 = ros::Time::now();
    auto t2 = t1;
    ed_->views_.clear();
    ed_->global_tour_.clear();

    std::cout << "start pos: " << pos.transpose() << ", vel: " << vel.transpose()
              << ", acc: " << acc.transpose() << std::endl;

    if (ep_->is_inspect_)
    {
      frontier_finder_->updateVisitCells(pos, pitch(0), yaw(0), ed_->frontiers_, ed_->frontier_boxes_);
      frontier_finder_->searchInspectFrontiers();

      // Find viewpoints (x,y,z,pitch,yaw) for all frontier clusters and get visible ones' info
      frontier_finder_->computeInspectFrontiersToVisit();

      // bool neighbor;
      // frontier_finder_->clusterFrontiers(pos, neighbor);

      // double frontier_time = (ros::Time::now() - t1).toSec();
      // t1 = ros::Time::now();

      // frontier_finder_->getFrontiers(ed_->frontiers_);
      // frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);
      // // frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);

      // if (ed_->frontiers_.empty())
      // {
      //   ROS_WARN("%s No coverable frontier.", ep_->drone_name_.c_str());
      //   return NO_FRONTIER;
      // }

      // // TODO: need to change but not necessary
      // frontier_finder_->getTopViewpointsInfo(pos, ed_->points_, ed_->pitchs_, ed_->yaws_, ed_->averages_);
      // for (int i = 0; i < ed_->points_.size(); ++i)
      //   ed_->views_.push_back(
      //       ed_->points_[i] + 2.0 * Vector3d(cos(ed_->yaws_[i]), sin(ed_->yaws_[i]), 0));

      // vector<vector<Vector3d>> division_clusters;
      // frontier_finder_->getFrontierDivision(division_clusters);
      // Eigen::Vector3d next_cluster_pos;
      // if (division_clusters.size() > 1)
      // {
      //   findNextCluster(pos, vel, yaw, ed_->local_tour_, next_cluster_pos, neighbor);
      //   // ROS_ERROR_STREAM("next_cluster_pos is " << next_cluster_pos);
      // }
      // else
      // {
      //   vector<int> indices;
      //   indices.clear();
      //   indices.push_back(0);
      //   frontier_finder_->getClusterTour(indices, ed_->global_cluster_tour_);
      //   frontier_finder_->getCheckTour(0, ed_->local_tour_);
      //   next_cluster_pos = ed_->global_cluster_tour_[0];
      // }

      // double view_time = (ros::Time::now() - t1).toSec();
      // ROS_WARN("%s Frontier: %d, t: %lf, viewpoint: %d, t: %lf", ep_->drone_name_.c_str(),
      //          ed_->frontiers_.size(), frontier_time, ed_->points_.size(), view_time);

      // Vector3d next_pos;
      // double next_yaw, next_pitch;
      // if (ed_->local_tour_.size() > 1)
      // {
      //   vector<int> indices;
      //   findLocalTour(pos, vel, yaw, next_cluster_pos, indices);
      //   next_pos = ed_->local_tour_[indices[0]].pos_;
      //   next_yaw = ed_->local_tour_[indices[0]].yaws_.front();
      //   next_pitch = ed_->local_tour_[indices[0]].pitchs_.front();
      //   ed_->local_tour_vis_.clear();
      //   ed_->local_tour_vis_.push_back(pos);
      //   for (int i = 0; i < indices.size() - 1; i++)
      //     ed_->local_tour_vis_.push_back(ed_->local_tour_[indices[i]].pos_);
      // }
      // else if (ed_->local_tour_.size() == 1)
      // {
      //   next_pos = ed_->local_tour_[0].pos_;
      //   next_yaw = ed_->local_tour_[0].yaws_.front();
      //   next_pitch = ed_->local_tour_[0].pitchs_.front();
      //   ed_->local_tour_vis_.clear();
      //   ed_->local_tour_vis_.push_back(pos);
      //   ed_->local_tour_vis_.push_back(ed_->local_tour_[0].pos_);
      // }
      // else
      //   ROS_ERROR("Empty destination.");

      double frontier_time = (ros::Time::now() - t1).toSec();
      t1 = ros::Time::now();
      frontier_finder_->getFrontiers(ed_->frontiers_);
      frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);
      frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);

      if (ed_->frontiers_.empty())
      {
        ROS_WARN("%s No coverable frontier.", ep_->drone_name_.c_str());
        return NO_FRONTIER;
      }
      frontier_finder_->getTopViewpointsInfo(pos, ed_->points_, ed_->pitchs_, ed_->yaws_, ed_->averages_);
      // need to change
      for (int i = 0; i < ed_->points_.size(); ++i)
        ed_->views_.push_back(
            ed_->points_[i] + 2.0 * Vector3d(cos(ed_->yaws_[i]), sin(ed_->yaws_[i]), 0));

      double view_time = (ros::Time::now() - t1).toSec();
      ROS_WARN("%s Frontier: %d, t: %lf, viewpoint: %d, t: %lf", ep_->drone_name_.c_str(),
               ed_->frontiers_.size(), frontier_time, ed_->points_.size(), view_time);
      // Do global and local tour planning and retrieve the next viewpoint
      Vector3d next_pos;
      double next_yaw, next_pitch;
      if (ed_->points_.size() > 1)
      {
        // Find the global tour passing through all viewpoints
        // Create TSP and solve by LKH
        // Optimal tour is returned as indices of frontier
        vector<int> indices;
        // ZMJ: Add history aware to findGlobalTour
        findGlobalTour(pos, vel, yaw, indices);

        if (ep_->refine_local_)
        {
          // Do refinement for the next few viewpoints in the global tour
          // Idx of the first K frontier in optimal tour
          t1 = ros::Time::now();

          ed_->refined_ids_.clear();
          ed_->unrefined_points_.clear();
          int knum = min(int(indices.size()), ep_->refined_num_);
          for (int i = 0; i < knum; ++i)
          {
            auto tmp = ed_->points_[indices[i]];
            ed_->unrefined_points_.push_back(tmp);
            ed_->refined_ids_.push_back(indices[i]);
            if ((tmp - pos).norm() > ep_->refined_radius_ && ed_->refined_ids_.size() >= 2)
              break;
          }

          // Get top N viewpoints for the next K frontiers
          ed_->n_points_.clear();
          vector<vector<double>> n_yaws, n_pitchs;
          frontier_finder_->getViewpointsInfo(
              pos, ed_->refined_ids_, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_, n_pitchs, n_yaws);

          ed_->refined_points_.clear();
          ed_->refined_views_.clear();
          vector<double> refined_yaws, refined_pitchs;
          refineLocalTour(pos, vel, pitch, yaw, ed_->n_points_, n_pitchs, n_yaws, ed_->refined_points_, refined_pitchs, refined_yaws);
          next_pos = ed_->refined_points_[0];
          next_pitch = refined_pitchs[0];
          next_yaw = refined_yaws[0];

          // Get marker for view visualization
          for (int i = 0; i < ed_->refined_points_.size(); ++i)
          {
            Vector3d view =
                ed_->refined_points_[i] + 2.0 * Vector3d(cos(refined_yaws[i]), sin(refined_yaws[i]), 0);
            ed_->refined_views_.push_back(view);
          }
          ed_->refined_views1_.clear();
          ed_->refined_views2_.clear();
          for (int i = 0; i < ed_->refined_points_.size(); ++i)
          {
            vector<Vector3d> v1, v2;
            frontier_finder_->percep_utils_->setPose_PY(ed_->refined_points_[i], refined_pitchs[i], refined_yaws[i]);
            frontier_finder_->percep_utils_->getFOV(v1, v2);
            ed_->refined_views1_.insert(ed_->refined_views1_.end(), v1.begin(), v1.end());
            ed_->refined_views2_.insert(ed_->refined_views2_.end(), v2.begin(), v2.end());
          }
          double local_time = (ros::Time::now() - t1).toSec();
          // ROS_WARN("%s Local refine time: %lf", ep_->drone_name_.c_str(), local_time);
        }
        else
        {
          // Choose the next viewpoint from global tour
          next_pos = ed_->points_[indices[0]];
          next_pitch = ed_->pitchs_[indices[0]];
          next_yaw = ed_->yaws_[indices[0]];
        }
      }
      else if (ed_->points_.size() == 1)
      {
        // Only 1 destination, no need to find global tour through TSP
        ed_->global_tour_ = {pos, ed_->points_[0]};
        ed_->refined_tour_.clear();
        ed_->refined_views1_.clear();
        ed_->refined_views2_.clear();

        if (ep_->refine_local_)
        {
          // Find the min cost viewpoint for next frontier
          ed_->refined_ids_ = {0};
          ed_->unrefined_points_ = {ed_->points_[0]};
          ed_->n_points_.clear();
          vector<vector<double>> n_yaws, n_pitchs;
          frontier_finder_->getViewpointsInfo(
              pos, {0}, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_, n_pitchs, n_yaws);

          double min_cost = 100000;
          int min_cost_id = -1;
          vector<Vector3d> tmp_path;
          for (int i = 0; i < ed_->n_points_[0].size(); ++i)
          {
            auto tmp_cost = ViewNode::computeCost(
                pos, ed_->n_points_[0][i], yaw[0], n_yaws[0][i], vel, yaw[1], tmp_path);
            if (tmp_cost < min_cost)
            {
              min_cost = tmp_cost;
              min_cost_id = i;
            }
          }
          next_pos = ed_->n_points_[0][min_cost_id];
          next_pitch = n_pitchs[0][min_cost_id];
          next_yaw = n_yaws[0][min_cost_id];
          ed_->refined_points_ = {next_pos};
          ed_->refined_views_ = {next_pos + 2.0 * Vector3d(cos(next_yaw), sin(next_yaw), 0)};
        }
        else
        {
          next_pos = ed_->points_[0];
          next_pitch = ed_->pitchs_[0];
          next_yaw = ed_->yaws_[0];
        }
      }
      else
        ROS_ERROR("Empty destination.");

      std::cout << "Next view: " << next_pos.transpose() << ", " << next_yaw << std::endl;

      // Plan trajectory (position and yaw) to the next viewpoint
      t1 = ros::Time::now();

      // Compute time lower bound of yaw and use in trajectory generation
      double diff = fabs(next_yaw - yaw[0]);
      double time_lb = min(diff, 2 * M_PI - diff) / ViewNode::yd_;

      // Generate trajectory of x,y,z
      planner_manager_->path_finder_->reset();
      planner_manager_->path_finder_->setMaxSearchTime(0.001);
      if (planner_manager_->path_finder_->search(pos, next_pos) != Astar::REACH_END)
      {
        planner_manager_->path_finder_->setMaxSearchTime(0.008);
        if (planner_manager_->path_finder_->search(pos, next_pos) != Astar::REACH_END)
        {
          ROS_ERROR("%s No path to next viewpoint", ep_->drone_name_.c_str());
          return FAIL;
        }
      }
      ed_->path_next_goal_ = planner_manager_->path_finder_->getPath();
      shortenPath(ed_->path_next_goal_);

      const double radius_far = 5.0;
      const double radius_close = 1.5;
      const double len = Astar::pathLength(ed_->path_next_goal_);
      if (len < radius_close)
      {
        // Next viewpoint is very close, no need to search kinodynamic path, just use waypoints-based
        // optimization
        planner_manager_->planExploreTraj(ed_->path_next_goal_, vel, acc, time_lb);
        ed_->next_goal_ = next_pos;
      }
      else if (len > radius_far)
      {
        // Next viewpoint is far away, select intermediate goal on geometric path (this also deal with
        // dead end)
        std::cout << "Far goal." << std::endl;
        double len2 = 0.0;
        vector<Eigen::Vector3d> truncated_path = {ed_->path_next_goal_.front()};
        for (int i = 1; i < ed_->path_next_goal_.size() && len2 < radius_far; ++i)
        {
          auto cur_pt = ed_->path_next_goal_[i];
          len2 += (cur_pt - truncated_path.back()).norm();
          truncated_path.push_back(cur_pt);
        }
        ed_->next_goal_ = truncated_path.back();
        planner_manager_->planExploreTraj(truncated_path, vel, acc, time_lb);
      }
      else
      {
        // Search kino path to exactly next viewpoint and optimize
        std::cout << "Mid goal" << std::endl;
        ed_->next_goal_ = next_pos;

        if (!planner_manager_->kinodynamicReplan(
                pos, vel, acc, ed_->next_goal_, Vector3d(0, 0, 0), time_lb))
          return FAIL;
      }

      if (planner_manager_->local_data_.position_traj_.getTimeSum() < time_lb - 0.1)
        ROS_ERROR("Lower bound not satified!");

      double traj_plan_time = (ros::Time::now() - t1).toSec();
      t1 = ros::Time::now();

      planner_manager_->planYawExplore(yaw, next_yaw, false, ep_->relax_time_);

      if (ep_->need_pitch_)
      {
        // next_pitch = 0.0;  // ERROR!!!!!!?????
        if (next_pitch > 0.0 && next_pitch < 0.02)
          next_pitch = 0.02;
        else if (next_pitch <= 0.0 && next_pitch > -0.02)
          next_pitch = -0.02;

        planner_manager_->planPitchExplore(pitch, next_pitch);
        ROS_ERROR("%s pitch (%f -> %f)", ep_->drone_name_.c_str(), pitch(0), next_pitch);
      }

      double yaw_time = (ros::Time::now() - t1).toSec();

      // ROS_WARN("%s Traj: %lf, yaw: %lf", ep_->drone_name_.c_str(), traj_plan_time, yaw_time);
      double total = (ros::Time::now() - t2).toSec();
      ROS_WARN("%s Total time: %lf", ep_->drone_name_.c_str(), total);
      ROS_ERROR_COND(total > 0.1, "%s Total time too long!!!", ep_->drone_name_.c_str());

      return SUCCEED;
    }
    else
    {
      // Search frontiers and group them into clusters
      frontier_finder_->searchFrontiers();

      // Find viewpoints (x,y,z,pitch=0,yaw) for all frontier clusters and get visible ones' info
      frontier_finder_->computeFrontiersToVisit();

      // bool neighbor;
      // frontier_finder_->clusterFrontiers(pos, neighbor);

      // double frontier_time = (ros::Time::now() - t1).toSec();
      // t1 = ros::Time::now();

      // frontier_finder_->getFrontiers(ed_->frontiers_);
      // frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);
      // // frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);
      // sdf_map_->bbx_->updateBoxVisit(ed_->frontier_boxes_);

      // if (ed_->frontiers_.empty())
      // {
      //   ROS_WARN("%s No coverable frontier.", ep_->drone_name_.c_str());
      //   return NO_FRONTIER;
      // }

      // // TODO: need to change but not necessary
      // frontier_finder_->getTopViewpointsInfo(pos, ed_->points_, ed_->pitchs_, ed_->yaws_, ed_->averages_);
      // for (size_t i = 0; i < ed_->points_.size(); ++i)
      //   ed_->views_.push_back(
      //       ed_->points_[i] + 2.0 * Vector3d(cos(ed_->yaws_[i]), sin(ed_->yaws_[i]), 0));

      // vector<vector<Vector3d>> division_clusters;
      // frontier_finder_->getFrontierDivision(division_clusters);
      // Eigen::Vector3d next_cluster_pos;
      // if (division_clusters.size() > 1)
      // {
      //   findNextCluster(pos, vel, yaw, ed_->local_tour_, next_cluster_pos, neighbor);
      // }
      // else
      // {
      //   vector<int> indices;
      //   indices.push_back(0);
      //   frontier_finder_->getClusterTour(indices, ed_->global_cluster_tour_);
      //   frontier_finder_->getCheckTour(0, ed_->local_tour_);
      //   next_cluster_pos = ed_->global_cluster_tour_[0];
      // }

      // double view_time = (ros::Time::now() - t1).toSec();
      // t1 = ros::Time::now();
      // ROS_WARN("%s Frontier: %ld, t: %lf, viewpoint: %ld, time: %lf", ep_->drone_name_.c_str(),
      //          ed_->frontiers_.size(), frontier_time, ed_->points_.size(), view_time);

      // Vector3d next_pos;
      // double next_yaw, next_pitch;
      // if (ed_->local_tour_.size() > 1)
      // {
      //   vector<int> indices;
      //   findLocalTour(pos, vel, yaw, next_cluster_pos, indices);
      //   next_pos = ed_->local_tour_[indices[0]].pos_;
      //   next_yaw = ed_->local_tour_[indices[0]].yaws_.front();
      //   ed_->local_tour_vis_.clear();
      //   ed_->local_tour_vis_.push_back(pos);
      //   for (size_t i = 0; i < indices.size() - 1; i++)
      //     ed_->local_tour_vis_.push_back(ed_->local_tour_[indices[i]].pos_);
      // }
      // else if (ed_->local_tour_.size() == 1)
      // {
      //   next_pos = ed_->local_tour_[0].pos_;
      //   next_yaw = ed_->local_tour_[0].yaws_.front();
      //   ed_->local_tour_vis_.clear();
      //   ed_->local_tour_vis_.push_back(pos);
      //   ed_->local_tour_vis_.push_back(ed_->local_tour_[0].pos_);
      // }
      // else
      //   ROS_ERROR("Empty destination.");

      double frontier_time = (ros::Time::now() - t1).toSec();
      t1 = ros::Time::now();
      frontier_finder_->getFrontiers(ed_->frontiers_);
      frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);
      frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);

      sdf_map_->bbx_->updateBoxVisit(ed_->frontier_boxes_);

      if (ed_->frontiers_.empty())
      {
        ROS_WARN("%s No coverable frontier.", ep_->drone_name_.c_str());
        return NO_FRONTIER;
      }
      frontier_finder_->getTopViewpointsInfo(pos, ed_->points_, ed_->pitchs_, ed_->yaws_, ed_->averages_);
      // need to change
      for (int i = 0; i < ed_->points_.size(); ++i)
        ed_->views_.push_back(
            ed_->points_[i] + 2.0 * Vector3d(cos(ed_->yaws_[i]), sin(ed_->yaws_[i]), 0));

      double view_time = (ros::Time::now() - t1).toSec();
      ROS_WARN("%s Frontier: %d, t: %lf, viewpoint: %d, t: %lf", ep_->drone_name_.c_str(),
               ed_->frontiers_.size(), frontier_time, ed_->points_.size(), view_time);

      // Do global and local tour planning and retrieve the next viewpoint
      Vector3d next_pos;
      double next_yaw, next_pitch;
      if (ed_->points_.size() > 1)
      {
        // Find the global tour passing through all viewpoints
        // Create TSP and solve by LKH
        // Optimal tour is returned as indices of frontier
        vector<int> indices;
        // ZMJ: Add history aware to findGlobalTour
        findGlobalTour(pos, vel, yaw, indices);

        if (ep_->refine_local_)
        {
          // Do refinement for the next few viewpoints in the global tour
          // Idx of the first K frontier in optimal tour
          t1 = ros::Time::now();

          ed_->refined_ids_.clear();
          ed_->unrefined_points_.clear();
          int knum = min(int(indices.size()), ep_->refined_num_);
          for (int i = 0; i < knum; ++i)
          {
            auto tmp = ed_->points_[indices[i]];
            ed_->unrefined_points_.push_back(tmp);
            ed_->refined_ids_.push_back(indices[i]);
            if ((tmp - pos).norm() > ep_->refined_radius_ && ed_->refined_ids_.size() >= 2)
              break;
          }

          // Get top N viewpoints for the next K frontiers
          ed_->n_points_.clear();
          vector<vector<double>> n_yaws, n_pitchs;
          frontier_finder_->getViewpointsInfo(
              pos, ed_->refined_ids_, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_, n_pitchs, n_yaws);

          ed_->refined_points_.clear();
          ed_->refined_views_.clear();
          vector<double> refined_yaws, refined_pitchs;
          refineLocalTour(pos, vel, pitch, yaw, ed_->n_points_, n_pitchs, n_yaws, ed_->refined_points_, refined_pitchs, refined_yaws);
          next_pos = ed_->refined_points_[0];
          next_pitch = refined_pitchs[0];
          next_yaw = refined_yaws[0];

          // Get marker for view visualization
          for (int i = 0; i < ed_->refined_points_.size(); ++i)
          {
            Vector3d view =
                ed_->refined_points_[i] + 2.0 * Vector3d(cos(refined_yaws[i]), sin(refined_yaws[i]), 0);
            ed_->refined_views_.push_back(view);
          }
          ed_->refined_views1_.clear();
          ed_->refined_views2_.clear();
          for (int i = 0; i < ed_->refined_points_.size(); ++i)
          {
            vector<Vector3d> v1, v2;
            frontier_finder_->percep_utils_->setPose_PY(ed_->refined_points_[i], refined_pitchs[i], refined_yaws[i]);
            frontier_finder_->percep_utils_->getFOV(v1, v2);
            ed_->refined_views1_.insert(ed_->refined_views1_.end(), v1.begin(), v1.end());
            ed_->refined_views2_.insert(ed_->refined_views2_.end(), v2.begin(), v2.end());
          }
          double local_time = (ros::Time::now() - t1).toSec();
          // ROS_WARN("%s Local refine time: %lf", ep_->drone_name_.c_str(), local_time);
        }
        else
        {
          // Choose the next viewpoint from global tour
          next_pos = ed_->points_[indices[0]];
          next_pitch = ed_->pitchs_[indices[0]];
          next_yaw = ed_->yaws_[indices[0]];
        }
      }
      else if (ed_->points_.size() == 1)
      {
        // Only 1 destination, no need to find global tour through TSP
        ed_->global_tour_ = {pos, ed_->points_[0]};
        ed_->refined_tour_.clear();
        ed_->refined_views1_.clear();
        ed_->refined_views2_.clear();

        if (ep_->refine_local_)
        {
          // Find the min cost viewpoint for next frontier
          ed_->refined_ids_ = {0};
          ed_->unrefined_points_ = {ed_->points_[0]};
          ed_->n_points_.clear();
          vector<vector<double>> n_yaws, n_pitchs;
          frontier_finder_->getViewpointsInfo(
              pos, {0}, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_, n_pitchs, n_yaws);

          double min_cost = 100000;
          int min_cost_id = -1;
          vector<Vector3d> tmp_path;
          for (int i = 0; i < ed_->n_points_[0].size(); ++i)
          {
            auto tmp_cost = ViewNode::computeCost(
                pos, ed_->n_points_[0][i], yaw[0], n_yaws[0][i], vel, yaw[1], tmp_path);
            if (tmp_cost < min_cost)
            {
              min_cost = tmp_cost;
              min_cost_id = i;
            }
          }
          next_pos = ed_->n_points_[0][min_cost_id];
          next_pitch = n_pitchs[0][min_cost_id];
          next_yaw = n_yaws[0][min_cost_id];
          ed_->refined_points_ = {next_pos};
          ed_->refined_views_ = {next_pos + 2.0 * Vector3d(cos(next_yaw), sin(next_yaw), 0)};
        }
        else
        {
          next_pos = ed_->points_[0];
          next_pitch = ed_->pitchs_[0];
          next_yaw = ed_->yaws_[0];
        }
      }
      else
        ROS_ERROR("Empty destination.");

      std::cout << "Next view: " << next_pos.transpose() << ", " << next_yaw << std::endl;

      // Plan trajectory (position and yaw) to the next viewpoint
      t1 = ros::Time::now();

      // Compute time lower bound of yaw and use in trajectory generation
      double diff = fabs(next_yaw - yaw[0]);
      double time_lb = min(diff, 2 * M_PI - diff) / ViewNode::yd_;

      // Generate trajectory of x,y,z
      planner_manager_->path_finder_->reset();
      planner_manager_->path_finder_->setMaxSearchTime(0.001);
      if (planner_manager_->path_finder_->search(pos, next_pos) != Astar::REACH_END)
      {
        planner_manager_->path_finder_->setMaxSearchTime(0.008);
        if (planner_manager_->path_finder_->search(pos, next_pos) != Astar::REACH_END)
        {
          ROS_ERROR("%s No path to next viewpoint", ep_->drone_name_.c_str());
          return FAIL;
        }
      }
      planner_manager_->path_finder_->setMaxSearchTime(0.001);
      ed_->path_next_goal_ = planner_manager_->path_finder_->getPath();
      shortenPath(ed_->path_next_goal_);

      const double radius_far = 5.0;
      const double radius_close = 1.5;
      const double len = Astar::pathLength(ed_->path_next_goal_);
      if (len < radius_close)
      {
        // Next viewpoint is very close, no need to search kinodynamic path, just use waypoints-based
        // optimization
        planner_manager_->planExploreTraj(ed_->path_next_goal_, vel, acc, time_lb);
        ed_->next_goal_ = next_pos;
      }
      else if (len > radius_far)
      {
        // Next viewpoint is far away, select intermediate goal on geometric path (this also deal with
        // dead end)
        std::cout << "Far goal." << std::endl;
        double len2 = 0.0;
        vector<Eigen::Vector3d> truncated_path = {ed_->path_next_goal_.front()};
        for (int i = 1; i < ed_->path_next_goal_.size() && len2 < radius_far; ++i)
        {
          auto cur_pt = ed_->path_next_goal_[i];
          len2 += (cur_pt - truncated_path.back()).norm();
          truncated_path.push_back(cur_pt);
        }
        ed_->next_goal_ = truncated_path.back();
        planner_manager_->planExploreTraj(truncated_path, vel, acc, time_lb);
      }
      else
      {
        // Search kino path to exactly next viewpoint and optimize
        std::cout << "Mid goal" << std::endl;
        ed_->next_goal_ = next_pos;

        if (!planner_manager_->kinodynamicReplan(
                pos, vel, acc, ed_->next_goal_, Vector3d(0, 0, 0), time_lb))
          return FAIL;
      }

      if (planner_manager_->local_data_.position_traj_.getTimeSum() < time_lb - 0.1)
        ROS_ERROR("Lower bound not satified!");

      double traj_plan_time = (ros::Time::now() - t1).toSec();
      t1 = ros::Time::now();

      planner_manager_->planYawExplore(yaw, next_yaw, false, ep_->relax_time_);

      if (ep_->need_pitch_)
      {
        // next_pitch set 0.0  ERROR!!!!!!?????
        planner_manager_->planPitchExplore(pitch, next_pitch);
        ROS_ERROR("%s pitch (%f -> %f)", ep_->drone_name_.c_str(), pitch(0), next_pitch);
      }

      double yaw_time = (ros::Time::now() - t1).toSec();

      // ROS_WARN("%s Traj: %lf, yaw: %lf", ep_->drone_name_.c_str(), traj_plan_time, yaw_time);
      double total = (ros::Time::now() - t2).toSec();
      ROS_WARN("%s Total time: %lf", ep_->drone_name_.c_str(), total);
      ROS_ERROR_COND(total > 0.1, "%s Total time too long!!!", ep_->drone_name_.c_str());

      return SUCCEED;
    }
  }

  void FastExplorationManager::shortenPath(vector<Vector3d> &path)
  {
    if (path.empty())
    {
      ROS_ERROR("Empty path to shorten");
      return;
    }
    // Shorten the tour, only critical intermediate points are reserved.
    const double dist_thresh = 3.0;
    vector<Vector3d> short_tour = {path.front()};
    for (int i = 1; i < path.size() - 1; ++i)
    {
      if ((path[i] - short_tour.back()).norm() > dist_thresh)
        short_tour.push_back(path[i]);
      else
      {
        // Add waypoints to shorten path only to avoid collision
        ViewNode::caster_->input(short_tour.back(), path[i + 1]);
        Eigen::Vector3i idx;
        while (ViewNode::caster_->nextId(idx) && ros::ok())
        {
          if (edt_environment_->sdf_map_->getInflateOccupancy(idx) == 1 ||
              edt_environment_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN)
          {
            short_tour.push_back(path[i]);
            break;
          }
        }
      }
    }
    if ((path.back() - short_tour.back()).norm() > 1e-3)
      short_tour.push_back(path.back());

    // Ensure at least three points in the path
    if (short_tour.size() == 2)
      short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));
    path = short_tour;
  }

  bool FastExplorationManager::findGlobalTourofBox()
  {
    ROS_INFO("Find box tour---------------");

    auto t1 = ros::Time::now();
    auto t2 = t1;

    // string box_tsp_dir_ = ep_->tsp_dir_;

    Eigen::MatrixXd mat;

    sdf_map_->bbx_->getCostMatrix(mat);
    double mat_time = (ros::Time::now() - t1).toSec();

    const int box_num = sdf_map_->bbx_->getBoxNum();
    const int drone_num = sdf_map_->bbx_->getLeaderNum();
    const int dimension = 1 + drone_num + box_num;
    std::cout << " box_num: " << box_num << " drone num: " << drone_num << " dimension: " << dimension << "  " << to_string(dimension) << std::endl;

    std::vector<double> demand = sdf_map_->bbx_->getDemand();
    double task_factor = 0.65;
    double total_demand = std::accumulate(demand.begin(), demand.end(), 0.0);
    double capacity = task_factor * total_demand;

    const int prob_type = 2;
    // Create problem file--------------------------
    ofstream file(ep_->mtsp_dir_ + "/amtsp4_" + ep_->drone_name_ + ".atsp");

    std::cout << ep_->mtsp_dir_ + "/amtsp4_" + ep_->drone_name_ + ".atsp" << std::endl;

    file << "NAME : pairopt\n";

    if (prob_type == 1)
      file << "TYPE : ATSP\n";
    else if (prob_type == 2)
      file << "TYPE : ACVRP\n";

    file << "DIMENSION : " + to_string(dimension) + "\n";
    file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
    file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";

    if (prob_type == 2)
    {
      file << "CAPACITY : " + to_string(capacity) + "\n";  // ACVRP
      file << "VEHICLES : " + to_string(drone_num) + "\n"; // ACVRP
    }

    // Cost matrix
    file << "EDGE_WEIGHT_SECTION\n";
    for (int i = 0; i < dimension; ++i)
    {
      for (int j = 0; j < dimension; ++j)
      {
        int int_cost = 100 * mat(i, j);
        file << int_cost << " ";
      }
      file << "\n";
    }

    if (prob_type == 2)
    { // Demand section, ACVRP only
      file << "DEMAND_SECTION\n";
      file << "1 0\n";
      for (int i = 0; i < drone_num; ++i)
      {
        file << to_string(i + 2) + " 0\n";
      }
      for (int i = 0; i < box_num; ++i)
      {
        file << to_string(i + 2 + drone_num) + " " + to_string(int(demand[i])) + "\n";
      }
      file << "DEPOT_SECTION\n";
      file << "1\n";
      file << "EOF";
    }
    ROS_ERROR("CREATE BOX PROBLEM FILE!!!!!!!!!!!!!!!!!!!!!");
    // ROS_ERROR("PATH: %s", ep_->mtsp_dir_.c_str());
    file.close();

    // Create par file------------------------------------------
    int min_size = int(box_num) / 2;
    int max_size = ceil(int(box_num) / 2.0);
    file.open(ep_->mtsp_dir_ + "/amtsp4_" + ep_->drone_name_ + ".par");
    file << "SPECIAL\n";
    file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp4_" + ep_->drone_name_ + ".atsp\n";
    if (prob_type == 1)
    {
      file << "SALESMEN = " << to_string(drone_num) << "\n";
      file << "MTSP_OBJECTIVE = MINSUM\n";
      // file << "MTSP_OBJECTIVE = MINMAX\n";
      file << "MTSP_MIN_SIZE = " << to_string(min_size) << "\n";
      file << "MTSP_MAX_SIZE = " << to_string(max_size) << "\n";
      file << "TRACE_LEVEL = 0\n";
    }
    else if (prob_type == 2)
    {
      file << "TRACE_LEVEL = 1\n"; // ACVRP
      file << "SEED = 0\n";        // ACVRP
    }
    file << "RUNS = 1\n";
    file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp4_" + ep_->drone_name_ + ".tour\n";

    ROS_ERROR("CREATE BOX PAR FILE!!!!!!!!!!!!!!!!!!!!!");
    file.close();

    lkh_mtsp_solver::SolveMTSP srv;
    srv.request.prob = 4;
    // if (!tsp_client_.call(srv)) {
    if (!acvrp_client_.call(srv))
    {
      ROS_ERROR("Fail to solve BOX ACVRP.");
      return false;
    }

    // Read results
    t1 = ros::Time::now();

    ifstream fin(ep_->mtsp_dir_ + "/amtsp4_" + ep_->drone_name_ + ".tour");
    string res;
    vector<int> ids;
    while (getline(fin, res))
    {
      if (res.compare("TOUR_SECTION") == 0)
        break;
    }
    while (getline(fin, res))
    {
      int id = stoi(res);
      ids.push_back(id - 1);
      if (id == -1)
        break;
    }
    fin.close();

    std::cout << "tour section id is: " << std::endl;
    for (auto id : ids)
    {
      std::cout << id << " ";
    }
    std::cout << std::endl;

    // Parse the m-tour of grid
    vector<vector<int>> tours_id;
    vector<int> tour_id;
    // vector<vector<Eigen::Vector3d>> tours;
    vector<Eigen::Vector3d> tour;
    BoxInfo box;
    DroneInfo drone;
    for (auto id : ids)
    {
      if (id == 0)
        continue;
      if (id > 0 && id <= drone_num)
      {
        tour_id.clear();
        tour.clear();

        drone = sdf_map_->bbx_->getDroneInfo(id - 1);
        tour.push_back(drone.start_pos);
        tour_id.push_back(id);
      }
      else if (id >= dimension || id < 0)
      {
        tours_id.push_back(tour_id);
        ed_->box_tour.push_back(tour);
      }
      else
      {
        tour_id.push_back(id);
        std::cout << "getting box " << id - drone_num - 1 << std::endl;
        box = sdf_map_->bbx_->getBoxInfo(id - drone_num - 1);
        tour.push_back(box.center);
      }
    }
    for (auto tour_ids : tours_id)
    {
      for (size_t i = 1; i < tour_ids.size(); i++)
      {
        int drone_id = tour_ids[0] - 1;
        int box_id = tour_ids[i] - 3;
        sdf_map_->bbx_->addBoxIdForDrone(drone_id, box_id);
        int slf_drone_id = sdf_map_->bbx_->getSlfDroneId();
        DroneInfo slf_drone = sdf_map_->bbx_->getDroneInfo(slf_drone_id);
        int slf_drone_leader = slf_drone.leader;
        if (slf_drone_leader == drone_id)
          sdf_map_->bbx_->addBoxIdForDrone(slf_drone_id, box_id);
      }
    }
    ROS_ERROR("NUM OF TOURS IS %d", ed_->box_tour.size());
    std::cout << "tour section pos is: " << std::endl;
    for (auto tour : ed_->box_tour)
    {
      std::cout << "a tour is: " << std::endl;
      for (auto id : tour)
      {
        std::cout << id << " ";
      }
      std::cout << std::endl;
    }

    ROS_ERROR("SUCCESSFULLY FIND TOUR OF BOX===================");
    return true;
  }

  void FastExplorationManager::findNextCluster(const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d &cur_yaw,
                                               vector<checkPoint> &check_tour, Vector3d &next_cluster_pos, const bool neighbor)
  {
    auto t1 = ros::Time::now();

    Eigen::MatrixXd cost_matrix;
    frontier_finder_->getClusterMatrix(cur_pos, cur_vel, cur_yaw, cost_matrix);
    if (neighbor)
      cost_matrix(0, 1) = 0.0;

    vector<Eigen::Vector3d> centers;
    frontier_finder_->getClusterCenter(centers);
    vector<int> inertial_cluster_indices, TSP_cluster_indices;
    double inertial_cluster_cost = std::numeric_limits<double>::max(),
           TSP_cluster_cost = std::numeric_limits<double>::max();
    TSPConfig cluster_config;
    cluster_config.dimension_ = cost_matrix.rows();
    cluster_config.problem_name_ = "cluster";
    cluster_config.skip_first_ = true;
    cluster_config.skip_last_ = false;
    cluster_config.result_id_offset_ = 2;
    solveTSP(cost_matrix, cluster_config, TSP_cluster_indices, TSP_cluster_cost);

    vector<int> indices;
    indices = TSP_cluster_indices;
    frontier_finder_->getClusterTour(TSP_cluster_indices, ed_->global_cluster_tour_);

    next_cluster_pos = centers[indices[1]];
    frontier_finder_->getCheckTour(indices[0], check_tour);

    double cal_time = (ros::Time::now() - t1).toSec();
    ROS_WARN("%s [findNextCluster] Calculation Time: %f", ep_->drone_name_.c_str(), cal_time);
  }

  void FastExplorationManager::findInertialTour(
      const Vector3d cur_pos, const vector<Vector3d> &last_tour,
      const vector<Vector3d> &now_points, const Eigen::MatrixXd &points_cost_matrix,
      vector<int> &indices, double &inertia_cost)
  {
    auto t1 = ros::Time::now();
    inertia_cost = 0.0;
    if (last_tour.empty() || now_points.empty())
    {
      inertia_cost = 1000000.0;
      return;
    }

    indices.clear();
    for (int i = 0; i < now_points.size(); i++)
    {
      indices.push_back(i);
    }

    vector<double> classified_min_cost;
    vector<int> classified_id;
    vector<int> classified_num(last_tour.size(), 0);

    for (int i = 0; i < now_points.size(); i++)
    {
      double minDistance = std::numeric_limits<double>::max();
      int id = -1;
      for (int j = 0; j < last_tour.size(); j++)
      {
        if ((now_points[i] - last_tour[j]).norm() > 8.0)
          continue;
        vector<Vector3d> path;
        double pos_cost = ViewNode::searchPath(now_points[i], last_tour[j], path);
        if (pos_cost < minDistance)
        {
          minDistance = pos_cost;
          id = j;
        }
      }
      if (id == -1)
      {
        id = last_tour.size() - 1;
        minDistance = 1000000.0;
      }
      classified_min_cost.push_back(minDistance);
      classified_id.push_back(id);
      classified_num[id]++;
    }

    auto compare = [=](int id1, int id2)
    {
      if (classified_id[id1] != classified_id[id2])
        return classified_id[id1] < classified_id[id2];
      else
        return classified_min_cost[id1] < classified_min_cost[id2];
    };
    sort(indices.begin(), indices.end(), compare);

    auto calculateLocalTSP = [&](const int begin_id, const int end_id) -> double
    {
      // consider cur_pos for the first segment
      if (end_id == 0 && begin_id == 0)
        return points_cost_matrix(0, indices[0] + 1);
      // case of two neighbor feature points
      if (end_id - begin_id < 2)
        return points_cost_matrix(indices[begin_id] + 1, indices[end_id] + 1);

      Eigen::MatrixXd local_cost_matrix;
      vector<int> local_indices;
      vector<int> indices_copy;
      for (int i = begin_id; i <= end_id; i++)
      {
        indices_copy.push_back(indices[i]);
      }

      int dimen;
      double local_cost;
      dimen = end_id - begin_id + 1;
      local_cost_matrix = Eigen::MatrixXd::Zero(dimen, dimen);
      // cost between different cluster_centers
      for (int i = 0; i < dimen; i++)
      {
        for (int j = i; j < dimen; j++)
        {
          local_cost_matrix(i, j) = local_cost_matrix(j, i) = points_cost_matrix(
              indices[begin_id + i] + 1, indices[begin_id + j] + 1);
        }
      }

      // set cost to fix start point and end point
      local_cost_matrix.leftCols<1>().setZero();
      for (int i = 1; i < dimen - 1; i++)
      {
        local_cost_matrix(i, 0) = 2000;
        local_cost_matrix(dimen - 1, i) = 2000;
      }

      TSPConfig cluster_config;
      cluster_config.dimension_ = local_cost_matrix.rows();
      cluster_config.problem_name_ = "inertial";
      cluster_config.skip_first_ = true;
      cluster_config.skip_last_ = false;
      cluster_config.result_id_offset_ = 2;
      solveTSP(local_cost_matrix, cluster_config, local_indices, local_cost);

      for (int i = 0; i < local_indices.size(); i++)
      {
        indices[begin_id + i + 1] = indices_copy[local_indices[i]];
      }
      return local_cost;
    };

    int begin_id = 0, end_id = 0;
    while (end_id < now_points.size())
    {
      if (classified_min_cost[indices[end_id]] < ep_->feature_max_dist_)
      {
        double tmp_cost = calculateLocalTSP(begin_id, end_id);
        // ROS_ERROR("begin_id->end_id (%d->%d) indices[begin_id]->indices[end_id] (%d->%d) alculateLocalTSP: %f", begin_id, end_id, indices[begin_id], indices[end_id], tmp_cost);
        inertia_cost += tmp_cost;
        begin_id = end_id;
        end_id += classified_num[classified_id[indices[end_id]]];
      }
      else
      {
        end_id += classified_num[classified_id[indices[end_id]]];
      }
    }
    double tmp_cost = calculateLocalTSP(begin_id, indices.size() - 1);
    // ROS_ERROR("begin_id->end_id (%d->%d) indices[begin_id]->indices[end_id] (%d->%d) alculateLocalTSP: %f", begin_id, indices.size() - 1, indices[begin_id], indices[indices.size() - 1], tmp_cost);
    // ROS_ERROR("(indices.size %d points.size %d)", indices.size(), now_points.size());
    inertia_cost += tmp_cost;

    auto cal_time = (ros::Time::now() - t1).toSec();
    ROS_WARN("%s [updateInertialTour] Calculation time:%f", ep_->drone_name_.c_str(), cal_time);
  }

  void FastExplorationManager::solveTSP(const Eigen::MatrixXd &cost_matrix,
                                        const TSPConfig &config,
                                        vector<int> &result_indices,
                                        double &total_cost)
  {
    // Write params and cost matrix to problem file
    ofstream prob_file(ep_->tsp_dir_ + "/" + ep_->drone_name_ + "_" + config.problem_name_ + ".tsp");

    // Problem specification part, follow the format of TSPLIB
    string prob_spec =
        "NAME : single_frontier\nTYPE : ATSP\nDIMENSION : " +
        to_string(config.dimension_) +
        "\nEDGE_WEIGHT_TYPE : "
        "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";
    prob_file << prob_spec;

    // Use Asymmetric TSP
    const int scale = 100;
    for (int i = 0; i < config.dimension_; ++i)
    {
      for (int j = 0; j < config.dimension_; ++j)
      {
        int int_cost = cost_matrix(i, j) * scale;
        prob_file << int_cost << " ";
      }
      prob_file << "\n";
    }

    prob_file << "EOF";
    prob_file.close();

    // Call LKH TSP solver
    solveTSPLKH((ep_->tsp_dir_ + "/" + ep_->drone_name_ + "_" + config.problem_name_ + ".par").c_str());

    // Read result indices from the tour section of result file
    ifstream fin(ep_->tsp_dir_ + "/" + ep_->drone_name_ + "_" + config.problem_name_ + ".txt");
    string res;
    // Go to tour section
    while (getline(fin, res))
    {
      // Read total cost
      if (res.find("COMMENT : Length") != std::string::npos)
      {
        int cost_res = stoi(res.substr(19));
        total_cost = (double)cost_res / 100.0;
        // ROS_INFO("[ActiveExplorationManager] TSP problem name: %s, total
        // cost:
        // %.2f",
        //          config.problem_name_.c_str(), cost);
        std::cout << "[ActiveExplorationManager] TSP problem name: "
                  << config.problem_name_ << ", total cost: " << total_cost;
      }
      if (res.compare("TOUR_SECTION") == 0)
        break;
    }
    // Read indices
    while (getline(fin, res))
    {
      int id = stoi(res);

      // Ignore the first state (current state)
      if (id == 1 && config.skip_first_)
      {
        continue;
      }

      // Ignore the last state (next grid or virtual depot)
      if (id == config.dimension_ && config.skip_last_)
      {
        break;
      }

      // EOF
      if (id == -1)
        break;

      result_indices.push_back(id - config.result_id_offset_);
    }
    fin.close();
  }

  void FastExplorationManager::findLocalTour(const Vector3d &cur_pos,
                                             const Vector3d &cur_vel,
                                             const Vector3d cur_yaw,
                                             const Vector3d &next_cluster_pos,
                                             vector<int> &indices)
  {
    auto t1 = ros::Time::now();
    indices.clear();

    // Get cost matrix for current state and clusters
    Eigen::MatrixXd cost_mat;
    frontier_finder_->getCheckTourCostMatrix(cur_pos, cur_vel, cur_yaw, next_cluster_pos, cost_mat);
    const int dimension = cost_mat.rows();
    // double mat_time = (ros::Time::now() - t1).toSec();
    // t1 = ros::Time::now();
    ofstream prob_file(ep_->tsp_dir_ + "/" + ep_->drone_name_ + "_single" + ".tsp");
    string prob_spec =
        "NAME : single\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
        "\nEDGE_WEIGHT_TYPE : "
        "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";
    prob_file << prob_spec;
    const int scale = 100;
    // Use Asymmetric TSP
    for (int i = 0; i < dimension; ++i)
    {
      for (int j = 0; j < dimension; ++j)
      {
        int int_cost = cost_mat(i, j) * scale;
        prob_file << int_cost << " ";
      }
      prob_file << "\n";
    }

    prob_file << "EOF";
    prob_file.close();

    // Call LKH TSP solver
    solveTSPLKH((ep_->tsp_dir_ + "/" + ep_->drone_name_ + "_single" + ".par").c_str());

    // Read optimal tour from the tour section of result file
    ifstream res_file(ep_->tsp_dir_ + "/" + ep_->drone_name_ + "_single" + ".txt");
    string res;
    while (getline(res_file, res))
    {
      // Go to tour section
      if (res.compare("TOUR_SECTION") == 0)
        break;
    }

    // Read path for ATSP formulation
    while (getline(res_file, res))
    {
      // Read indices of frontiers in optimal tour
      int id = stoi(res);
      if (id == 1) // Ignore the current state
        continue;
      if (id == -1)
        break;
      indices.push_back(id - 2); // Idx of solver-2 == Idx of frontier
    }

    res_file.close();
    // double tsp_time = (ros::Time::now() - t1).toSec();
    // ROS_WARN("%s [FindLocalTour] Mat_time: %lf, TSP_time: %lf",ep_->drone_name_.c_str(), mat_time, tsp_time);
    double local_tour_time = (ros::Time::now() - t1).toSec();
    ROS_WARN("%s [FindLocalTour] local_tour_time: %lf", ep_->drone_name_.c_str(), local_tour_time);
  }

  void FastExplorationManager::findGlobalTour(
      const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d cur_yaw,
      vector<int> &indices)
  {
    auto t1 = ros::Time::now();

    // Get cost matrix for current state and clusters
    Eigen::MatrixXd cost_mat;
    frontier_finder_->updateFrontierCostMatrix();
    frontier_finder_->getFullCostMatrix(cur_pos, cur_vel, cur_yaw, cost_mat);
    // ROS_ERROR_STREAM("Cost Matrix:\n" << cost_mat);
    double mat_time = (ros::Time::now() - t1).toSec();
    t1 = ros::Time::now();
    vector<int> inertial_indices, TSP_indices;
    double inertial_cost = std::numeric_limits<double>::max(),
           TSP_cost = std::numeric_limits<double>::max();

    TSPConfig cluster_config;
    cluster_config.dimension_ = cost_mat.rows();
    cluster_config.problem_name_ = "ordinary";
    cluster_config.skip_first_ = true;
    cluster_config.skip_last_ = false;
    cluster_config.result_id_offset_ = 2;
    solveTSP(cost_mat, cluster_config, TSP_indices, TSP_cost);

    // zmj: Add history aware
    if (ep_->use_history_)
    {
      findInertialTour(cur_pos, ed_->last_tour_, ed_->points_, cost_mat, inertial_indices, inertial_cost);
      ROS_WARN("%s inertial_cost: %f  TSP_cost: %f", ep_->drone_name_.c_str(), inertial_cost, TSP_cost);
    }

    if (inertial_cost < TSP_cost + ep_->inertial_cost_offset_)
    {
      indices = inertial_indices;
      ROS_WARN("%s [findGlobalTour] Using Inertial tour", ep_->drone_name_.c_str());
    }
    else
    {
      indices = TSP_indices;
      ROS_WARN("%s [findGlobalTour] Using TSP tour", ep_->drone_name_.c_str());
    }

    ed_->last_tour_.clear();
    for (int i = 0; i < indices.size(); i++)
      ed_->last_tour_.push_back(ed_->points_[indices[i]]);

    // Get the path of optimal tour from path matrix
    // ZMJ : why add is bug !!!
    // frontier_finder_->getPathForTour(cur_pos, indices, ed_->global_tour_);

    double tsp_time = (ros::Time::now() - t1).toSec();
    ROS_WARN("%s Cost mat: %lf, TSP: %lf", ep_->drone_name_.c_str(), mat_time, tsp_time);
  }

  void FastExplorationManager::refineLocalTour(
      const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d &cur_pitch, const Vector3d &cur_yaw,
      const vector<vector<Vector3d>> &n_points, const vector<vector<double>> &n_pitchs, const vector<vector<double>> &n_yaws,
      vector<Vector3d> &refined_pts, vector<double> &refined_pitchs, vector<double> &refined_yaws)
  {
    double create_time, search_time, parse_time;
    auto t1 = ros::Time::now();

    // Create graph for viewpoints selection
    GraphSearch<ViewNode> g_search;
    vector<ViewNode::Ptr> last_group, cur_group;

    // Add the current state
    ViewNode::Ptr first(new ViewNode(cur_pos, cur_pitch(0), cur_yaw[0]));
    first->vel_ = cur_vel;
    g_search.addNode(first);
    last_group.push_back(first);
    ViewNode::Ptr final_node;

    // Add viewpoints
    std::cout << "Local tour graph: ";
    for (int i = 0; i < n_points.size(); ++i)
    {
      // Create nodes for viewpoints of one frontier
      for (int j = 0; j < n_points[i].size(); ++j)
      {
        ViewNode::Ptr node(new ViewNode(n_points[i][j], n_pitchs[i][j], n_yaws[i][j]));
        g_search.addNode(node);
        // Connect a node to nodes in last group
        for (auto nd : last_group)
          g_search.addEdge(nd->id_, node->id_);
        cur_group.push_back(node);

        // Only keep the first viewpoint of the last local frontier
        if (i == n_points.size() - 1)
        {
          final_node = node;
          break;
        }
      }
      // Store nodes for this group for connecting edges
      std::cout << cur_group.size() << ", ";
      last_group = cur_group;
      cur_group.clear();
    }
    std::cout << "" << std::endl;
    create_time = (ros::Time::now() - t1).toSec();
    t1 = ros::Time::now();

    // Search optimal sequence
    vector<ViewNode::Ptr> path;
    g_search.DijkstraSearch(first->id_, final_node->id_, path);

    search_time = (ros::Time::now() - t1).toSec();
    t1 = ros::Time::now();

    // Return searched sequence
    for (int i = 1; i < path.size(); ++i)
    {
      refined_pts.push_back(path[i]->pos_);
      refined_pitchs.push_back(path[i]->pitch_);
      refined_yaws.push_back(path[i]->yaw_);
    }

    // Extract optimal local tour (for visualization)
    ed_->refined_tour_.clear();
    ed_->refined_tour_.push_back(cur_pos);
    ViewNode::astar_->lambda_heu_ = 1.0;
    ViewNode::astar_->setResolution(0.5);
    for (auto pt : refined_pts)
    {
      vector<Vector3d> path;
      if (ViewNode::searchPath(ed_->refined_tour_.back(), pt, path))
        ed_->refined_tour_.insert(ed_->refined_tour_.end(), path.begin(), path.end());
      else
        ed_->refined_tour_.push_back(pt);
    }
    ViewNode::astar_->lambda_heu_ = 10000;

    parse_time = (ros::Time::now() - t1).toSec();
    // ROS_WARN("create: %lf, search: %lf, parse: %lf", create_time, search_time, parse_time);
  }

  bool FastExplorationManager::isLos(const Vector3d &pt1, const Vector3d &pt2)
  {
    ViewNode::caster_->input(pt1, pt2);
    Eigen::Vector3i idx;
    while (ViewNode::caster_->nextId(idx) && ros::ok())
    {
      if (edt_environment_->sdf_map_->getOccupancy(idx) == SDFMap::OCCUPIED )
      {
        return false;
      }
    }
    return true;
  }
} // namespace fast_planner
