cmake_minimum_required(VERSION 2.8.3)
project(test_temporarily)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  trajectory_msgs
  geometry_msgs
  roscpp
  rospy
  std_msgs
  mav_msgs
  quadrotor_msgs
  sensor_msgs
  caric_mission
)

find_package(Eigen3 REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(PCL 1.7 REQUIRED)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

include_directories(
	include
    ${catkin_INCLUDE_DIRS} 
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_LIBRARY_DIRS}
)

add_executable(traj_trans_node
src/traj_trans_node.cpp
)

target_link_libraries(traj_trans_node
${catkin_LIBRARIES}
)

add_executable(take_off_node
  src/take_off.cpp
)

target_link_libraries(take_off_node
  ${catkin_LIBRARIES}
)

add_executable(sensor_pose_trans_node src/pose_trans_node.cpp)

target_link_libraries(sensor_pose_trans_node
  ${catkin_LIBRARIES}
)

add_executable(pcd_reader_node src/pcd_reader.cpp)

target_link_libraries(pcd_reader_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


add_executable(ppcom_talker_node src/ppcom_talker.cpp)

target_link_libraries(ppcom_talker_node
  ${catkin_LIBRARIES}
)

add_executable(gcs_talker_node src/gcs_node.cpp)

target_link_libraries(gcs_talker_node
  ${catkin_LIBRARIES}
)
