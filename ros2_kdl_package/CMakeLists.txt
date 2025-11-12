cmake_minimum_required(VERSION 3.8)
project(ros2_kdl_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------------------
# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kdl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# -------------------------------
# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${kdl_parser_INCLUDE_DIRS}
)

# -------------------------------
# Definiamo le sorgenti "helper"
# (Senza nessun file 'main')
set(HELPER_SOURCES
  src/kdl_robot.cpp
  src/kdl_planner.cpp
  src/kdl_control.cpp
)

# -------------------------------
# ros2_kdl_node (Il vecchio file)
add_executable(ros2_kdl_node
  src/ros2_kdl_node.cpp
  ${HELPER_SOURCES}
)
target_compile_features(ros2_kdl_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(ros2_kdl_node
  orocos_kdl
  rclcpp
  kdl_parser
  sensor_msgs
  std_msgs
  geometry_msgs
)

# -------------------------------
# kdl_action_server (Il file corretto)
# *** ASSICURATI CHE IL TUO FILE SI CHIAMI "kdl_action_server.cpp" ***
add_executable(kdl_action_server
  src/kdl_action_server.cpp
  ${HELPER_SOURCES}
)
target_compile_features(kdl_action_server PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(kdl_action_server
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs
  kdl_interfaces
  kdl_parser
)

# -------------------------------
# kdl_action_client
add_executable(kdl_action_client src/kdl_action_client.cpp)
target_compile_features(kdl_action_client PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(kdl_action_client
  rclcpp
  rclcpp_action
  geometry_msgs
  kdl_interfaces
)


add_executable(ros2_kdl_node_vision src/ros2_kdl_node_vision.cpp src/kdl_robot.cpp src/kdl_planner.cpp src/kdl_control.cpp)


# -------------------------------
# ros2_kdl_node_example
add_executable(ros2_kdl_node_example src/ros2_kdl_node_example.cpp)
target_compile_features(ros2_kdl_node_example PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(ros2_kdl_node_example
  orocos_kdl
  rclcpp
)

target_include_directories(ros2_kdl_node_vision PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})
target_compile_features(ros2_kdl_node_vision PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(ros2_kdl_node_vision 
  orocos_kdl
  rclcpp
  kdl_parser
  sensor_msgs
  std_msgs)

# -------------------------------
# Install targets
install(TARGETS
  ros2_kdl_node
  kdl_action_server
  kdl_action_client
  ros2_kdl_node_vision
  ros2_kdl_node_example
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# -------------------------------
# Testing / linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()