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


geometry_msgs::msg::Pose build_pose (double x, double y, double z, double rx, double ry, double rz, double rw) {
  geometry_msgs::msg::Pose target;
  target.position.x = x;
  target.position.y = y;
  target.position.z = z;

  // Orientation as quaternion (w,x,y,z).
  // This example keeps a neutral-ish orientation; adjust to what you need.
  target.orientation.x = rx;
  target.orientation.y = ry;
  target.orientation.z = rz;
  target.orientation.w = rw;

  return target;
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "adapted_hardcoded_aproach",
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
  move_group.setPoseReferenceFrame("base");

  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(50);

  // Define target joint values (radians)
  // UR3e has 6 joints
  std::vector<double> joint_target = {
    1.1001,
    -1.8619,
    1.7854,
    4.7859,
    -1.5715,
    5.6838
  };
  move_group.setJointValueTarget(joint_target);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto const ok = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (ok)
  {
    auto const exec_ok = (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!exec_ok) RCLCPP_ERROR(node->get_logger(), "Execution failed.");
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Planning failed. Aborting.");
    rclcpp::shutdown();
    spinner.join();
    return 0;
  }

  move_group.setPoseReferenceFrame("base_link");

  // ---- Build Cartesian waypoints ----
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.reserve(8 * 3 * 2); // 8 poses × tap(3) × 2 matrices

  geometry_msgs::msg::Pose start = move_group.getCurrentPose().pose;

    RCLCPP_INFO(node->get_logger(), "start pose: p=(%.3f, %.3f, %.3f) q=(%.3f, %.3f, %.3f, %.3f)",
              start.position.x, start.position.y, start.position.z,
              start.orientation.x, start.orientation.y,
              start.orientation.z, start.orientation.w);

  waypoints.push_back(start);

  auto rx = start.orientation.x;
  auto ry = start.orientation.y;
  //auto rz = start.orientation.z;
  //auto rw = start.orientation.w;

  double Z_TBD = 0.2156;
  double z_pickup = 0.1878;

  double z_delta = Z_TBD - z_pickup;

  auto pose_drop_to_Z = build_pose( start.position.x, start.position.y, Z_TBD, start.orientation.x, start.orientation.y, start.orientation.z, start.orientation.w);
  waypoints.push_back(pose_drop_to_Z);

  auto push_tap = [&](const geometry_msgs::msg::Pose& p)
  {
    waypoints.push_back(p);
    auto p_down = p;
    p_down.position.z = p_down.position.z - z_delta - 0.002;
    waypoints.push_back(p_down);
    waypoints.push_back(p);
  };

  // ===== Matrix 1 =====
  auto pose_11_m1 = build_pose(0.2166, 0.1923, Z_TBD, rx, ry, 0.0, 0.0);
  push_tap(pose_11_m1);

  auto pose_12_m1 = build_pose(0.1540, 0.1923, Z_TBD, rx, ry, 0.0, 0.0);
  push_tap(pose_12_m1);

  auto pose_21_m1 = build_pose(0.2166, 0.2546, Z_TBD, rx, ry, 0.0, 0.0);
  push_tap(pose_21_m1);

  auto pose_22_m1 = build_pose(0.1540, 0.2546, Z_TBD, rx, ry, 0.0, 0.0);
  push_tap(pose_22_m1);

  auto pose_31_m1 = build_pose(0.2166, 0.3179, Z_TBD, rx, ry, 0.0, 0.0);
  push_tap(pose_31_m1);

  auto pose_32_m1 = build_pose(0.1540, 0.3179, Z_TBD, rx, ry, 0.0, 0.0);
  push_tap(pose_32_m1);

  auto pose_41_m1 = build_pose(0.2166, 0.3809, Z_TBD, rx, ry, 0.0, 0.0);
  push_tap(pose_41_m1);

  auto pose_42_m1 = build_pose(0.1540, 0.3809, Z_TBD, rx, ry, 0.0, 0.0);
  push_tap(pose_42_m1);


  // ===== Matrix 2 =====
  auto pose_11_m2 = build_pose(-0.1549, 0.1887, Z_TBD, rx, ry, 0.0, 0.0);
  push_tap(pose_11_m2);

  auto pose_12_m2 = build_pose(-0.2180, 0.1887, Z_TBD, rx, ry, 0.0, 0.0);
  push_tap(pose_12_m2);

  auto pose_21_m2 = build_pose(-0.1549, 0.2521, Z_TBD, rx, ry, 0.0, 0.0);
  push_tap(pose_21_m2);

  auto pose_22_m2 = build_pose(-0.2180, 0.2521, Z_TBD, rx, ry, 0.0, 0.0);
  push_tap(pose_22_m2);

  auto pose_31_m2 = build_pose(-0.1549, 0.3167, Z_TBD, rx, ry, 0.0, 0.0);
  push_tap(pose_31_m2);

  auto pose_32_m2 = build_pose(-0.2180, 0.3167, Z_TBD, rx, ry, 0.0, 0.0);
  push_tap(pose_32_m2);

  auto pose_41_m2 = build_pose(-0.1549, 0.3789, Z_TBD, rx, ry, 0.0, 0.0);
  push_tap(pose_41_m2);

  auto pose_42_m2 = build_pose(-0.2180, 0.3789, Z_TBD, rx, ry, 0.0, 0.0);
  push_tap(pose_42_m2);

  waypoints.push_back(pose_drop_to_Z);

  // Cartesian path settings:
  // eef_step: resolution in meters between interpolated points
  // jump_threshold: 0 disables jump detection; consider 1.0-2.0 for real robots
  const double eef_step = 0.005;      // 5 mm
  const double jump_threshold = 0.0;  // disable jump threshold
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
                                        0.2,
                                        0.2);

  if (!time_ok) {
    RCLCPP_ERROR(node->get_logger(), "Time parameterization failed. Not executing.");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  rt.getRobotTrajectoryMsg(trajectory_msg);

  // ---- Execute ----
  moveit::planning_interface::MoveGroupInterface::Plan plan2;
  plan2.trajectory_ = trajectory_msg;

  RCLCPP_INFO(node->get_logger(), "Executing Cartesian trajectory...");
  auto result = move_group.execute(plan2);

  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Execution failed.");
  } else {
    RCLCPP_INFO(node->get_logger(), "Done.");
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}