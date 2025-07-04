cmake_minimum_required(VERSION 3.8)
project(caric_mission_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rotors_comm_msgs REQUIRED)
find_package(caric_mission REQUIRED)
find_package(domain_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)


# Install Python scripts
install(PROGRAMS
  scripts/ppcom_router.py
  scripts/ppcom_jurong_talker.py
  scripts/ppcom_raffles_talker.py
  scripts/ppcom_sentosa_talker.py
  scripts/ppcom_changi_talker.py
  scripts/ppcom_nanyang_talker.py
  scripts/ppcom_call.py
  scripts/ppcom_sequential_launcher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install Config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
