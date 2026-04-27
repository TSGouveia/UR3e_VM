cmake_minimum_required(VERSION 3.8)
project(go_to_pose)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(go_to_pose_adapted src/go_to_target_pose_adapted.cpp)
ament_target_dependencies(go_to_pose_adapted 
          rclcpp
          moveit_ros_planning_interface
          geometry_msgs
          moveit_ros_planning
          moveit_core
          )

add_executable(go_to_all src/go_to_all_poses.cpp)
ament_target_dependencies(go_to_all 
          rclcpp
          moveit_ros_planning_interface
          geometry_msgs
          moveit_ros_planning
          moveit_core
          )

install(TARGETS
          go_to_pose_adapted
          go_to_all 
          DESTINATION lib/${PROJECT_NAME})

ament_package()
