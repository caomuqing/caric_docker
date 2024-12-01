#include <iostream>
#include <fstream>
#include <numeric>
#include <yaml-cpp/yaml.h>
#include <plan_env/bounding_box.h>

namespace fast_planner
{
  BBox::BBox()
  {
  }

  BBox::~BBox()
  {
  }

  void BBox::initBBox(ros::NodeHandle &nh)
  {
    std::string BoxPath;
    int box_id_tmp;
    std::string drone_name_;
    nh.param<std::string>("bounding_box/box_path", BoxPath, "");
    nh.param("bounding_box/box_inflate", box_inflate_, 0.0);
    nh.param("bounding_box/use_box_id", use_box_id_, false);
    nh.param("exploration/drone_name", drone_name_, std::string("null"));
    if (drone_name_ == "jurong")
      slf_drone_id_ = 0;
    else if (drone_name_ == "raffles")
      slf_drone_id_ = 1;
    else if (drone_name_ == "sentosa")
      slf_drone_id_ = 2;
    else if (drone_name_ == "changi")
      slf_drone_id_ = 3;
    else if (drone_name_ == "nanyang")
      slf_drone_id_ = 4;

    YAML::Node BoxesNode = YAML::LoadFile(BoxPath);

    int BoxID = 0;
    for (YAML::const_iterator it = BoxesNode.begin(); it != BoxesNode.end(); ++it)
    {
      std::string boxName = it->first.as<std::string>();
      BoxInfo box;

      box.visit = false;
      box.BoxID = BoxID;
      box.center = Eigen::Vector3d(BoxesNode[boxName]["center"][0].as<double>(),
                                   BoxesNode[boxName]["center"][1].as<double>(), BoxesNode[boxName]["center"][2].as<double>());
      box.size = Eigen::Vector3d(BoxesNode[boxName]["size"][0].as<double>() + box_inflate_,
                                 BoxesNode[boxName]["size"][1].as<double>() + box_inflate_,
                                 BoxesNode[boxName]["size"][2].as<double>() + box_inflate_);

      Eigen::Matrix4d orientation;
      orientation << BoxesNode[boxName]["orientation"][0].as<double>(),
          BoxesNode[boxName]["orientation"][1].as<double>(),
          BoxesNode[boxName]["orientation"][2].as<double>(),
          BoxesNode[boxName]["orientation"][3].as<double>(),
          BoxesNode[boxName]["orientation"][4].as<double>(),
          BoxesNode[boxName]["orientation"][5].as<double>(),
          BoxesNode[boxName]["orientation"][6].as<double>(),
          BoxesNode[boxName]["orientation"][7].as<double>(),
          BoxesNode[boxName]["orientation"][8].as<double>(),
          BoxesNode[boxName]["orientation"][9].as<double>(),
          BoxesNode[boxName]["orientation"][10].as<double>(),
          BoxesNode[boxName]["orientation"][11].as<double>(),
          BoxesNode[boxName]["orientation"][12].as<double>(),
          BoxesNode[boxName]["orientation"][13].as<double>(),
          BoxesNode[boxName]["orientation"][14].as<double>(),
          BoxesNode[boxName]["orientation"][15].as<double>();

      box.orientation = orientation;
      boxes_.push_back(box);
      // 增加 BoxID 计数器
      BoxID++;
      // ROS_INFO("BoxID : %d",BoxID);
      std::cout << "BoxID: " << box.BoxID << " BoxCenter: " << box.center << " BoxSize: "
                << box.size << " BoxOrientation: " << box.orientation << std::endl;
    }
  }

  void BBox::updateBoxVisit(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &frt_boxes)
  {
    for (auto frt_box : frt_boxes)
    {
      Eigen::Vector3d center = frt_box.first;
      for (auto &bbox : boxes_)
      {
        if (bbox.visit)
          continue;
        if (isInBoundingBox(center, bbox))
        {
          bbox.visit = true;
          ROS_ERROR("boundingbox %d is visit!!!!!!!!!!!!!!!!!!!", bbox.BoxID);
        }
      }
    }
  }

  void BBox::setDroneStartPos(std::vector<std::string> node_name, std::vector<Eigen::Vector3d> node_pos)
  {
    for (size_t i = 0; i < node_name.size(); i++)
    {
      DroneInfo drone;
      std::string drone_name = node_name[i];
      if (drone_name == "gcs")
        continue;
      else
      {
        drone.DroneID = i;
        drone.DroneName = drone_name;
        if (drone_name == "jurong" || drone_name == "raffles")
          drone.DroneType = true;
        else
          drone.DroneType = false;
        drone.leader = -1;
        drone.is_free = drone.DroneType;
        drone.start_pos = node_pos[i];
        drone.start_pos(2) = 3.0;
        drones_.push_back(drone);
        ROS_ERROR_STREAM("drone name: " << drone_name << " drone id: " << drone.DroneID << " start pos " << drone.start_pos);
      }
    }
  }

  void BBox::addBoxIdForDrone(int drone_id, int box_id)
  {
    if (drone_id != slf_drone_id_)
      return;
    ROS_ERROR("drone id = %d add box %d", drone_id, box_id);
    box_id_.push_back(box_id);
  }

  std::vector<int> BBox::getBoxTour()
  {
    return box_id_;
  }

  bool BBox::getBoxVisit(int id)
  {
    BoxInfo box = getBoxInfo(id);
    return box.visit;
  }

  int BBox::getSlfDroneId()
  {
    return slf_drone_id_;
  }

  bool BBox::teamAllocate()
  {
    std::vector<DroneInfo> leader_drones_, follower_drones_;
    int last_lead_id = -2, last_follow_id = -2;

    // find the leader drones and follower drones
    for (DroneInfo &drone : this->drones_)
    {
      // if the drone is a leader, it does not need a leader
      if (drone.DroneType == true)
      {
        leader_drones_.push_back(drone);
        drone.leader = -1;
        std::cout << "drone: " << drone.DroneID << " is a leader" << std::endl;
      }
      else
      {
        follower_drones_.push_back(drone);
        std::cout << "drone: " << drone.DroneID << " is a follower" << std::endl;
      }
    }

    // std::vector<int> team_member_num(leader_drones_.size(),0);

    std::cout << "there are " << follower_drones_.size() << " followers" << std::endl;

    // allocate team
    while (!follower_drones_.empty())
    {
      std::cout << "unallocate num: " << follower_drones_.size() << std::endl;
      // ROS_ERROR("ENTER!");
      double min_dist = std::numeric_limits<double>::max();
      int follow_id = -1;
      int lead_id = -1;

      for (DroneInfo &lead_drone : leader_drones_)
      {
        for (DroneInfo &follow_drone : follower_drones_)
        {
          // ROS_ERROR("FIND NEAREST");
          double current_dist = (follow_drone.start_pos - lead_drone.start_pos).norm();
          if (current_dist < min_dist)
          {
            min_dist = current_dist;
            follow_id = follow_drone.DroneID;
            lead_id = lead_drone.DroneID;
          }
        }
      }

      // set the leader for the follower

      this->drones_[follow_id].leader = lead_id;
      std::cout << "drone " << follow_id << " follows " << lead_id << std::endl;

      last_lead_id = lead_id;
      last_follow_id = follow_id;

      for (auto iter = follower_drones_.begin(); iter != follower_drones_.end();)
      {
        if (iter->DroneID == follow_id)
        {
          iter = follower_drones_.erase(iter);
          // ROS_ERROR("ERASE!");
          continue;
        }
        else
        {
          ++iter;
        }
      }
    }

    // refine when all the followers are allocated to the same leader
    ROS_ERROR("NEED REFINE!!!!");
    if (this->isSameTeam(last_lead_id))
    {
      for (DroneInfo &lead_drone : leader_drones_)
      {
        if (lead_drone.DroneID != last_lead_id)
        {
          this->drones_[last_follow_id].leader = lead_drone.DroneID;
          std::cout << "after refine drone " << last_follow_id << " follows " << lead_drone.DroneID << std::endl;
        }
      }
    }

    ROS_ERROR("SUCCESSFULLY ALLOCATE TEAM==============");
    // for (DroneInfo& drone : this->drones_){
    //   std::cout<<"drone "<<drone.DroneID<<" follows "<<drone.leader<<std::endl;
    // }

    return true;
  }

  DroneInfo BBox::getDroneInfo(int id)
  {
    for (DroneInfo &drone : this->drones_)
    {
      if (drone.DroneID == id)
      {
        return drone;
      }
    }
  }

  void BBox::setDroneFree(int id)
  {
    for (DroneInfo &drone : this->drones_)
    {
      if (drone.DroneID == id)
      {
        drone.is_free = true;
      }
    }
  }

  bool BBox::isSameTeam(int lead_id)
  {
    for (DroneInfo &drone : this->drones_)
    {
      if (drone.DroneType == false & drone.leader != lead_id)
      {
        return false;
      }
    }
    return true;
  }

  std::vector<BoxInfo> BBox::getBoxesInfo()
  {
    return this->boxes_;
  }

  BoxInfo BBox::getBoxInfo(int id)
  {
    for (BoxInfo &box : this->boxes_)
    {
      if (box.BoxID == id)
      {
        return box;
      }
    }
    ROS_ERROR("THERE IS NO TARGET BOX============");
  }

  Eigen::Vector3d BBox::getBoxCenter(int id)
  {
    for (BoxInfo &box : this->boxes_)
    {
      if (box.BoxID == id)
      {
        return box.center;
      }
    }
  }

  int BBox::getBoxNum()
  {
    return this->boxes_.size();
  }

  int BBox::getDroneNum()
  {
    return this->drones_.size();
  }

  int BBox::getLeaderNum()
  {
    int leader_num = 0;
    for (DroneInfo &drone : this->drones_)
    {
      // if the drone is a leader, it does not need a leader
      if (drone.DroneType == true)
      {
        leader_num++;
      }
    }
    return leader_num;
  }

  std::vector<double> BBox::getDemand()
  {
    std::vector<double> demand;
    for (const BoxInfo &box : this->boxes_)
    {
      double volume = box.size[0] * box.size[1] * box.size[2];
      demand.push_back(volume);
    }
    return demand;
  }

  void BBox::getCostMatrix(Eigen::MatrixXd &mat)
  {
    ROS_ERROR("START CALCULATE COST=========");
    std::vector<DroneInfo> leader_drones_;
    for (DroneInfo &drone : this->drones_)
    {
      // if the drone is a leader, it does not need a leader
      if (drone.DroneType == true)
      {
        leader_drones_.push_back(drone);
      }
    }
    ROS_ERROR_STREAM("leader_drones_num " << leader_drones_.size());
    const int drone_num = leader_drones_.size();
    const int box_num = this->boxes_.size();
    const int dimen = 1 + drone_num + box_num;

    mat = Eigen::MatrixXd::Zero(dimen, dimen);

    // Virtual depot to drones
    for (int i = 0; i < drone_num; ++i)
    {
      mat(0, 1 + i) = -1000;
      mat(1 + i, 0) = 1000;
    }
    // Virtual depot to grid
    for (int i = 0; i < box_num; ++i)
    {
      mat(0, 1 + drone_num + i) = 1000;
      mat(1 + drone_num + i, 0) = 0;
    }
    // Costs between drones
    for (int i = 0; i < drone_num; ++i)
    {
      for (int j = 0; j < drone_num; ++j)
      {
        mat(1 + i, 1 + j) = 10000;
      }
    }

    // Costs from drones to grid
    for (int i = 0; i < drone_num; ++i)
    {
      for (int j = 0; j < box_num; ++j)
      {
        double cost = this->getCostDroneToBox(leader_drones_[i], boxes_[j]);
        mat(1 + i, 1 + drone_num + j) = cost;
        mat(1 + drone_num + j, 1 + i) = 0;
      }
    }
    // Costs between grid
    for (int i = 0; i < box_num; ++i)
    {
      for (int j = i + 1; j < box_num; ++j)
      {
        double cost = this->getCostBoxToBox(boxes_[i], boxes_[j]);
        mat(1 + drone_num + i, 1 + drone_num + j) = cost;
        mat(1 + drone_num + j, 1 + drone_num + i) = cost;
      }
    }

    // Diag
    for (int i = 0; i < dimen; ++i)
    {
      mat(i, i) = 1000;
    }
    ROS_ERROR("END CALCULATE COST=========");
  }

  double BBox::getCostDroneToBox(const DroneInfo &drone, const BoxInfo &box)
  {
    Eigen::Vector3d edge = drone.start_pos - box.center;
    return edge.norm();
  }

  double BBox::getCostBoxToBox(const BoxInfo &box1, const BoxInfo &box2)
  {
    Eigen::Vector3d edge = box1.center - box2.center;
    return edge.norm();
  }

  bool BBox::isInBoundingBox(const Eigen::Vector3d &point, const BoxInfo &box)
  {
    // Apply the inverse of the box's rotation to align the box with the axes
    Eigen::Matrix3d rotation_matrix = box.orientation.block<3, 3>(0, 0);
    Eigen::Quaterniond orientation_quaternion(rotation_matrix);
    Eigen::Quaterniond rotationInverse = orientation_quaternion.inverse();
    Eigen::Vector3d localPoint = rotationInverse * (point - box.center);

    // Calculate half extents of the aligned box
    Eigen::Vector3d halfExtents = box.size / 2.0;

    // Check if the local point is within the aligned box
    if (localPoint.x() >= -halfExtents.x() && localPoint.x() <= halfExtents.x() &&
        localPoint.y() >= -halfExtents.y() && localPoint.y() <= halfExtents.y() &&
        localPoint.z() >= -halfExtents.z() && localPoint.z() <= halfExtents.z())
    {
      return true;
    }

    return false;
  }

  bool BBox::isInAnyBoundingBox(const Eigen::Vector3d &point)
  {
    for (const BoxInfo &box : this->boxes_)
    {
      if (use_box_id_)
      {
        for (int i = 0; i < box_id_.size(); i++)
        {
          if (box.BoxID == box_id_[i] && isInBoundingBox(point, box))
          {
            return true;
          }
        }
      }
      else
      {
        if (isInBoundingBox(point, box))
        {
          return true;
        }
      }
    }
    return false;
  }

  std::vector<geometry_msgs::Point32> BBox::get_bounding_box_vertices(BoxInfo box)
  {

    // 计算立方体的半尺寸
    Eigen::Vector3d half_size = box.size / 2.0;

    // 定义立方体的八个顶点相对于中心的偏移
    Eigen::Vector3d vertices_offsets[8] = {Eigen::Vector3d(
                                               -half_size.x(), -half_size.y(), -half_size.z()),
                                           Eigen::Vector3d(half_size.x(), -half_size.y(), -half_size.z()),
                                           Eigen::Vector3d(half_size.x(), half_size.y(), -half_size.z()),
                                           Eigen::Vector3d(-half_size.x(), half_size.y(), -half_size.z()),
                                           Eigen::Vector3d(-half_size.x(), -half_size.y(), half_size.z()),
                                           Eigen::Vector3d(half_size.x(), -half_size.y(), half_size.z()),
                                           Eigen::Vector3d(half_size.x(), half_size.y(), half_size.z()),
                                           Eigen::Vector3d(-half_size.x(), half_size.y(), half_size.z())};

    // 创建一个存储顶点位置的容器
    std::vector<geometry_msgs::Point32> vertices;

    // 对每个顶点应用朝向变换并添加到容器中
    for (int i = 0; i < 8; ++i)
    {
      Eigen::Vector4d transformed_vertex =
          box.orientation * Eigen::Vector4d(vertices_offsets[i].x(), vertices_offsets[i].y(),
                                            vertices_offsets[i].z(), 1.0);
      geometry_msgs::Point32 vertex;
      vertex.x = transformed_vertex.x();
      vertex.y = transformed_vertex.y();
      vertex.z = transformed_vertex.z();
      // std::cout << "vertex: " << vertex << std::endl;
      vertices.push_back(vertex);
    }

    return vertices;
  }

  void BBox::cal_global_map(Eigen::Vector3d &box_min, Eigen::Vector3d &box_max)
  {
    float max_x = std::numeric_limits<float>::min();
    float max_y = std::numeric_limits<float>::min();
    float max_z = std::numeric_limits<float>::min();
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    for (const fast_planner::BoxInfo &box : boxes_)
    {
      std::vector<geometry_msgs::Point32> vertices = this->get_bounding_box_vertices(box);
      for (const geometry_msgs::Point32 &vertex : vertices)
      {
        if (vertex.x > max_x)
          max_x = vertex.x;
        if (vertex.y > max_y)
          max_y = vertex.y;
        if (vertex.z > max_z)
          max_z = vertex.z;
        if (vertex.x < min_x)
          min_x = vertex.x;
        if (vertex.y < min_y)
          min_y = vertex.y;
        if (vertex.z < min_z)
          min_z = vertex.z;
      }
    }

    ROS_ERROR("======CALCULATE MAP======");
    std::cout << "Max x: " << max_x << ", Max y: " << max_y << ", Max z: " << max_z << std::endl;
    std::cout << "Min x: " << min_x << ", Min y: " << min_y << ", Min z: " << min_z << std::endl;
    box_min = Eigen::Vector3d(min_x - 3.0, min_y - 3.0, min_z - 3.0);
    box_max = Eigen::Vector3d(max_x + 3.0, max_y + 3.0, max_z + 3.0);
  }

} // namespace fast_planner
  // SDFMap
