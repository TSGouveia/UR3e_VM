cmake_minimum_required(VERSION 3.8)
project(piece_juggler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ur_msgs REQUIRED)

add_executable(basic_piece_juggler src/basic_piece_juggler.cpp)
ament_target_dependencies(basic_piece_juggler 
          rclcpp
          moveit_ros_planning_interface
          geometry_msgs
          moveit_ros_planning
          moveit_core
          ur_msgs
          )

add_executable(random_piece_juggler src/random_piece_juggler.cpp)
ament_target_dependencies(random_piece_juggler 
          rclcpp
          moveit_ros_planning_interface
          geometry_msgs
          moveit_ros_planning
          moveit_core
          ur_msgs
          )

install(TARGETS
          basic_piece_juggler
          random_piece_juggler 
          DESTINATION lib/${PROJECT_NAME})

ament_package()
