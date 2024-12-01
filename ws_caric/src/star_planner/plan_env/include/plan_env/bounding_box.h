#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <sensor_msgs/PointCloud.h>

#include <queue>
#include <ros/ros.h>
#include <tuple>

using namespace std;

namespace fast_planner {

struct BoxInfo;
struct DroneInfo;

class BBox {
public:
  BBox();
  ~BBox();
  void initBBox(ros::NodeHandle& nh);
  int getBoxNum();
  int getDroneNum();
  int getSlfDroneId();
  DroneInfo getDroneInfo(int id);
  int getLeaderNum();
  void setDroneStartPos(std::vector<std::string> node_name,std::vector<Eigen::Vector3d> node_pos);
  // std::vector<int> getLeaderList();
  std::vector<BoxInfo> getBoxesInfo();
  BoxInfo getBoxInfo(int id);
  void cal_global_map(Eigen::Vector3d &box_min,Eigen::Vector3d &box_max);
  bool isInAnyBoundingBox(const Eigen::Vector3d& point);
  bool haveAnyBoxesOverlap(const Eigen::Vector3d& box_min, const Eigen::Vector3d& box_max);
  bool teamAllocate();
  void getCostMatrix(Eigen::MatrixXd& mat);
  std::vector<double> getDemand();
  void addBoxIdForDrone(int drone_id, int box_id);
  void updateBoxVisit(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &boxes);
  Eigen::Vector3d getBoxCenter(int id);
  std::vector<int> getBoxTour();
  bool getBoxVisit(int id);
  void setDroneFree(int id);
  

private:
  std::vector<DroneInfo> drones_;
  std::vector<BoxInfo> boxes_;
  std::vector<int> box_id_;
  double box_inflate_;
  bool use_box_id_;
  // std::vector<int>& visit_id;
  std::vector<geometry_msgs::Point32> get_bounding_box_vertices(BoxInfo box);
  bool isInBoundingBox(const Eigen::Vector3d& point, const BoxInfo& box);
  bool haveBoxesOverlap(
      const Eigen::Vector3d& box1_min, const Eigen::Vector3d& box1_max, const BoxInfo& box2);
  double getCostDroneToBox(const DroneInfo& drone, const BoxInfo& box);
  double getCostBoxToBox(const BoxInfo& box1, const BoxInfo& box2);
  bool isSameTeam(int lead_id);

 int slf_drone_id_;
};

struct BoxInfo {
  int BoxID;
  Eigen::Vector3d center;
  Eigen::Vector3d size;
  Eigen::Matrix4d orientation;
  bool visit;
};

struct DroneInfo {
  int DroneID;
  bool DroneType;
  int leader;
  string DroneName;
  bool is_free;
  Eigen::Vector3d start_pos;
};
}  // namespace fast_planner



#endif