// cartesian_to_hardcoded_pose.cpp
//
// ROS 2 Humble + MoveIt 2 example:
// Plan a Cartesian path to a hard-coded pose and execute it.

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>

// For time parameterization
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "ur3e_cartesian_hardcoded_pose",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Run ROS callbacks in a background thread (MoveGroupInterface needs spinning)
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  // ---- MoveIt setup ----
  static const std::string PLANNING_GROUP = "ur_manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  // Optional but helpfulmove_group.setPoseReferenceFrame("base");
  move_group.setPlanningTime(5.0);
  //move_group.setMaxVelocityScalingFactor(0.1);
  //move_group.setMaxAccelerationScalingFactor(0.1);

  // If your EEF link is different, change this.
  // You can also omit this if your SRDF defines a default.
  move_group.setEndEffectorLink("tool0");
  move_group.setPoseReferenceFrame("base_link");


  // ---- Hard-coded target pose (base frame) ----
  // NOTE: This must be reachable and collision-free.
  geometry_msgs::msg::Pose target;
  target.position.x = -0.067;
  target.position.y = -0.284;
  target.position.z = 0.560;

  // Orientation as quaternion (w,x,y,z).
  // This example keeps a neutral-ish orientation; adjust to what you need.
  target.orientation.x = 0.081;
  target.orientation.y = 0.705;
  target.orientation.z = -0.700;
  target.orientation.w = -0.080;

  // ---- Build Cartesian waypoints ----
  std::vector<geometry_msgs::msg::Pose> waypoints;

  // Start from current pose to avoid jumps
  geometry_msgs::msg::Pose start = move_group.getCurrentPose().pose;

    RCLCPP_INFO(node->get_logger(), "start pose: p=(%.3f, %.3f, %.3f) q=(%.3f, %.3f, %.3f, %.3f)",
              start.position.x, start.position.y, start.position.z,
              start.orientation.x, start.orientation.y,
              start.orientation.z, start.orientation.w);


    target = move_group.getCurrentPose().pose;
    target.position.x -= 0.01;

  waypoints.push_back(start);
  waypoints.push_back(target);

  // Cartesian path settings:
  // eef_step: resolution in meters between interpolated points
  // jump_threshold: 0 disables jump detection; consider 1.0-2.0 for real robots
  const double eef_step = 0.001;      // 5 mm
  const double jump_threshold = 2.0;  // disable jump threshold
  moveit_msgs::msg::RobotTrajectory trajectory_msg;

  RCLCPP_INFO(node->get_logger(), "Computing Cartesian path...");
  double fraction = move_group.computeCartesianPath(
      waypoints, eef_step, jump_threshold, trajectory_msg, /*avoid_collisions=*/true);

  RCLCPP_INFO(node->get_logger(), "Cartesian path fraction: %.3f", fraction);

  if (fraction < 0.95) {
    RCLCPP_ERROR(node->get_logger(),
                 "Cartesian path incomplete (fraction %.3f). Not executing.", fraction);
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  // ---- Time-parameterize the trajectory (VERY important for execution) ----
  robot_trajectory::RobotTrajectory rt(
      move_group.getRobotModel(), move_group.getName());
  rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_msg);

  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  bool time_ok = iptp.computeTimeStamps(rt,
                                        0.1,
                                        0.1);

  if (!time_ok) {
    RCLCPP_ERROR(node->get_logger(), "Time parameterization failed. Not executing.");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  rt.getRobotTrajectoryMsg(trajectory_msg);

  // ---- Execute ----
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory_msg;

  RCLCPP_INFO(node->get_logger(), "Executing Cartesian trajectory...");
  auto result = move_group.execute(plan);

  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Execution failed.");
  } else {
    RCLCPP_INFO(node->get_logger(), "Done.");
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
