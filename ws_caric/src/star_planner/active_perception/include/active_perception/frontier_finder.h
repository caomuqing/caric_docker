#ifndef _FRONTIER_FINDER_H_
#define _FRONTIER_FINDER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <list>
#include <utility>

using Eigen::Vector3d;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;
using std::list;
using std::pair;

class RayCaster;

namespace fast_planner {
class EDTEnvironment;
class PerceptionUtils;

// Viewpoint to cover a frontier cluster
struct Viewpoint {
  // Position and heading
  Vector3d pos_;
  double pitch_;
  double yaw_;
  // Fraction of the cluster that can be covered
  // double fraction_;
  int visib_num_;
};

// A frontier cluster, the viewpoints to cover it
struct Frontier {
  // Complete voxels belonging to the cluster
  vector<Vector3d> cells_;
  // down-sampled voxels filtered by voxel grid filter
  vector<Vector3d> filtered_cells_;
  // Average position of all voxels
  Vector3d average_;
  // Idx of cluster
  int id_;
  // Viewpoints that can cover the cluster
  vector<Viewpoint> viewpoints_;
  // Bounding box of cluster, center & 1/2 side length
  Vector3d box_min_, box_max_;
  // Path and cost from this cluster to other clusters
  list<vector<Vector3d>> paths_;
  list<double> costs_;

  bool divided;
};

struct FrontierCluster{

  vector<Frontier> frts_;

  Vector3d center_;

  Vector3d box_min_, box_max_, box_range_;

  vector<double> costs_;
};

struct checkPoint {
  Eigen::Vector3d pos_;
  vector<double> yaws_;
  vector<double> pitchs_;
  vector<double> costs_;
};

class FrontierFinder {
public:
  FrontierFinder(const shared_ptr<EDTEnvironment>& edt, ros::NodeHandle& nh);
  ~FrontierFinder();

  void searchFrontiers();
  void searchInspectFrontiers();
  void computeFrontiersToVisit();
  void computeInspectFrontiersToVisit();

  void clusterFrontiers(const Eigen::Vector3d &cur_pos,bool &neighbor);
  void updateVisitCells(const Vector3d &pos, const double &pitch, const double &yaw,
                        vector<vector<Vector3d>> &clusters,vector<pair<Vector3d, Vector3d>> &cluster_boxes);
                        
  void getFrontiers(vector<vector<Vector3d>>& clusters);
  void getDormantFrontiers(vector<vector<Vector3d>>& clusters);
  void getFrontierBoxes(vector<pair<Vector3d, Vector3d>>& boxes);
  void getFrontierDivision(vector<vector<Eigen::Vector3d>> &frt_division);
  void getClusterCenter(vector<Vector3d> &centers);
  void getClusterTour(const vector<int> indices,vector<Vector3d> &path);
  void getCheckTour(const int cluster_id,vector<checkPoint> &local_tour);

  // Get viewpoint with highest coverage for each frontier
  void getTopViewpointsInfo(const Vector3d& cur_pos, vector<Vector3d>& points, vector<double>& pitchs, 
                            vector<double>& yaws,vector<Vector3d>& averages);
  // Get several viewpoints for a subset of frontiers
  void getViewpointsInfo(const Vector3d& cur_pos, const vector<int>& ids, const int& view_num,
                         const double& max_decay, vector<vector<Vector3d>>& points,
                         vector<vector<double>> &pitchs,vector<vector<double>>& yaws);
  void updateFrontierCostMatrix();
  void getClusterMatrix(const Vector3d &cur_pos,const Vector3d &cur_vel,const Vector3d cur_yaw,
                        Eigen::MatrixXd &cost_mat);
  void getFullCostMatrix(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw,
                         Eigen::MatrixXd& mat);
  void getCheckTourCostMatrix(const Vector3d &cur_pos,const Vector3d &cur_vel,const Vector3d cur_yaw,
                              const Vector3d &next_cluster_pos,Eigen::MatrixXd &mat);
  void getPathForTour(const Vector3d& pos, const vector<int>& frontier_ids, vector<Vector3d>& path);

  void setNextFrontier(const int& id);
  bool isFrontierCovered();
  void wrapYaw(double& yaw);

  bool computeYaw(const Vector3d &cur_pos,double &yaw);

  shared_ptr<PerceptionUtils> percep_utils_;

private:
  void splitLargeFrontiers(list<Frontier>& frontiers);
  bool splitHorizontally(const Frontier& frontier, list<Frontier>& splits);
  //ZMJ ADD:splitIn3D
  bool splitIn3D(const Frontier& frontier, list<Frontier>& splits);
  void mergeFrontiers(Frontier& ftr1, const Frontier& ftr2);
  bool isFrontierChanged(const Frontier& ft);
  bool haveOverlap(const Vector3d& min1, const Vector3d& max1, const Vector3d& min2,
                   const Vector3d& max2);
  void computeFrontierInfo(Frontier& frontier);
  void downsample(const vector<Vector3d>& cluster_in, vector<Vector3d>& cluster_out);
  void sampleViewpoints(Frontier& frontier);
  void sampleInspectViewpoints(Frontier& frontier);

  int countVisibleCells(const Vector3d& pos, const double &pitch, const double& yaw, const vector<Vector3d>& cluster);
  void computeNormal(const vector<Vector3d> &point_cloud,vector<Vector3d> &normals);
  void drawLines(const vector<Vector3d> &points1, const vector<Vector3d> &points2, double scale, Eigen::Vector4d color, const std::string &ns, int id);
  bool isNearUnknown(const Vector3d& pos);
  vector<Eigen::Vector3i> sixNeighbors(const Eigen::Vector3i& voxel);
  vector<Eigen::Vector3i> tenNeighbors(const Eigen::Vector3i& voxel);
  vector<Eigen::Vector3i> allNeighbors(const Eigen::Vector3i& voxel);
  bool isNeighborUnknown(const Eigen::Vector3i& voxel);
  bool isNeighborOccupied(const Eigen::Vector3i &voxel);

  void expandFrontier(const Eigen::Vector3i& first /* , const int& depth, const int& parent_id */);
  void expandInspectFrontier(const Eigen::Vector3i& first /* , const int& depth, const int& parent_id */);

  // Wrapper of sdf map
  int toadr(const Eigen::Vector3i& idx);
  bool knownfree(const Eigen::Vector3i& idx);
  bool inmap(const Eigen::Vector3i& idx);

  // Deprecated
  Eigen::Vector3i searchClearVoxel(const Eigen::Vector3i& pt);
  bool isInBoxes(const vector<pair<Vector3d, Vector3d>>& boxes, const Eigen::Vector3i& idx);
  bool canBeMerged(const Frontier& ftr1, const Frontier& ftr2);
  void findViewpoints(const Vector3d& sample, const Vector3d& ftr_avg, vector<Viewpoint>& vps);

  // Data
  vector<char> frontier_flag_;
  vector<char> visit_flag_;
  list<Frontier> frontiers_, dormant_frontiers_, tmp_frontiers_;
  vector<int> removed_ids_;
  list<Frontier>::iterator first_new_ftr_;
  Frontier next_frontier_;

  vector<FrontierCluster> frontier_clusters_;
  vector<checkPoint> check_tour_;

  // Params
  int cluster_min_;
  double cluster_size_xy_, cluster_size_z_,cluster_size_xyz_;
  double candidate_rmax_, candidate_rmin_, candidate_dphi_, min_candidate_dist_,
      min_candidate_clearance_;
  int down_sample_;
  double min_view_finish_fraction_, resolution_;
  int min_visib_num_, candidate_rnum_;
  double frt_cluster_range_x_,frt_cluster_range_y_,frt_cluster_range_z_;
  bool need_pitch_;

  // Utils
  shared_ptr<EDTEnvironment> edt_env_;
  unique_ptr<RayCaster> raycaster_;
  ros::Publisher marker1_pub_;
};

}  // namespace fast_planner
#endif