cmake_minimum_required(VERSION 3.8)
project(stl_models)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(enviroment_collision src/enviroment.cpp)
ament_target_dependencies(enviroment_collision rclcpp moveit_ros_planning_interface geometric_shapes)

add_executable(gripper_collision src/gripper.cpp)
ament_target_dependencies(gripper_collision 
          rclcpp
          moveit_ros_planning_interface
          geometric_shapes
          tf2
          tf2_ros
          tf2_geometry_msgs
          geometry_msgs)

install(TARGETS
          enviroment_collision
          gripper_collision  
          DESTINATION lib/${PROJECT_NAME})

ament_package()
