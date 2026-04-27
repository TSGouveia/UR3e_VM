cmake_minimum_required(VERSION 3.8)
project(ur_action_server_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ur_action_interface REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ur_msgs REQUIRED)

add_executable(move_piece_server src/move_piece_server.cpp)
ament_target_dependencies(move_piece_server
  rclcpp
  rclcpp_action
  ur_action_interface
  moveit_ros_planning_interface
  geometry_msgs
  moveit_ros_planning
  moveit_core
  ur_msgs
)

add_executable(callibrate src/callibrate.cpp)
ament_target_dependencies(callibrate
  rclcpp
  moveit_ros_planning_interface
)

install(TARGETS 
  move_piece_server
  callibrate
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
