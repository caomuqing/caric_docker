cmake_minimum_required(VERSION 3.0.2)
project(kios_solution)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17 -Wfatal-errors")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  octomap_msgs
  octomap_ros
  geometry_msgs
  pcl_conversions
  pcl_ros
  rotors_comm
  unicon
  message_generation
)

# catkin_python_setup()

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

find_package(octomap REQUIRED)

if("$ENV{ROS_DISTRO}" STREQUAL "melodic")
  find_package(gazebo QUIET PATHS /usr/include/gazebo-11)
else()
  find_package(gazebo REQUIRED)
endif()

find_package(PCL REQUIRED)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  OctomapToCoords.srv
)

add_message_files(
  FILES
  area.msg
  norms.msg
  multiPoint.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  octomap_msgs
  geometry_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
 DEPENDS rospy
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
link_directories("/home/dronesteam/ws_caric/kios_solution/scripts")
# list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

## Declare a C++ executable
add_executable(${PROJECT_NAME}_octomap_translator_node src/octomapTranslate.cpp)
target_link_libraries(${PROJECT_NAME}_octomap_translator_node
                      ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})

# Declare the nodes
catkin_install_python(PROGRAMS scripts/odom2tf.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/gcs.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/explorer.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/uav_fov.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/explorer_path.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/photographer.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/photographer_path.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
