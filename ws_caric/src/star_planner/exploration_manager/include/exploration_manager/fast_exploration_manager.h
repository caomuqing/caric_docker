#ifndef _EXPLORATION_MANAGER_H_
#define _EXPLORATION_MANAGER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>

using Eigen::Vector3d;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace fast_planner
{
  class EDTEnvironment;
  class SDFMap;
  class FastPlannerManager;
  class FrontierFinder;
  struct ExplorationParam;
  struct ExplorationData;
  struct checkPoint;

  struct TSPConfig
  {
    int dimension_;
    std::string problem_name_;
    bool skip_first_;
    bool skip_last_;
    int result_id_offset_;
  };

  enum EXPL_RESULT
  {
    NO_FRONTIER,
    FAIL,
    SUCCEED
  };

  class FastExplorationManager
  {
  public:
    FastExplorationManager();
    ~FastExplorationManager();

    void initialize(ros::NodeHandle &nh);

    int planExploreMotion(const Vector3d &pos, const Vector3d &vel, const Vector3d &acc,
                          const Vector3d &yaw, const Vector3d &pitch);

    // Benchmark method, classic frontier and rapid frontier
    int classicFrontier(const Vector3d &pos, const double &yaw);
    int rapidFrontier(const Vector3d &pos, const Vector3d &vel, const double &yaw, bool &classic);
    bool findGlobalTourofBox();
    bool isLos(const Vector3d &pt1, const Vector3d &pt2);

    shared_ptr<ExplorationData> ed_;
    shared_ptr<ExplorationParam> ep_;
    shared_ptr<FastPlannerManager> planner_manager_;
    shared_ptr<FrontierFinder> frontier_finder_;
    shared_ptr<SDFMap> sdf_map_;
    // unique_ptr<ViewFinder> view_finder_;

  private:
    shared_ptr<EDTEnvironment> edt_environment_;

    void findInertialTour(
        const Vector3d cur_pos, const vector<Vector3d> &last_tour,
        const vector<Vector3d> &now_points, const Eigen::MatrixXd &points_cost_matrix,
        vector<int> &indices, double &inertia_cost);

    // Find optimal tour for coarse viewpoints of all frontiers
    void findGlobalTour(const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d cur_yaw,
                        vector<int> &indices);

    void findLocalTour(const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d cur_yaw,
                       const Vector3d &next_cluster_pos, vector<int> &indices);

    // Refine local tour for next few frontiers, using more diverse viewpoints
    void refineLocalTour(
        const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d &cur_pitch, const Vector3d &cur_yaw,
        const vector<vector<Vector3d>> &n_points, const vector<vector<double>> &n_pitchs, const vector<vector<double>> &n_yaws,
        vector<Vector3d> &refined_pts, vector<double> &refined_pitchs, vector<double> &refined_yaws);

    void findNextCluster(const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d &cur_yaw,
                         vector<checkPoint> &check_tour, Vector3d &next_cluster_pos, const bool neighbor);

    void shortenPath(vector<Vector3d> &path);

    void solveTSP(const Eigen::MatrixXd &cost_matrix,
                  const TSPConfig &config,
                  vector<int> &result_indices,
                  double &total_cost);

    ros::ServiceClient acvrp_client_;

  public:
    typedef shared_ptr<FastExplorationManager> Ptr;
  };

} // namespace fast_planner

#endif