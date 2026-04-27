#include <memory>
#include <vector>
#include <iostream>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <ur_msgs/srv/set_io.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>

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

std::vector<std::vector<std::vector<geometry_msgs::msg::Pose>>> build_poses_3d_vector (double Z_TBD, double rx, double ry){
    
    std::vector<std::vector<std::vector<geometry_msgs::msg::Pose>>> poses;
    // Two matrices, 4 rows, 2 columns each
    poses.resize(2, std::vector<std::vector<geometry_msgs::msg::Pose>>(
                   4, std::vector<geometry_msgs::msg::Pose>(2)));
    
    // ===== Matrix 1 (index 0) =====
    poses[0][0][0] = build_pose( 0.2166, 0.1923-0.002, Z_TBD, rx, ry, 0.0, 0.0 );
    poses[0][0][1] = build_pose( 0.1540, 0.1923-0.002, Z_TBD, rx, ry, 0.0, 0.0 );

    poses[0][1][0] = build_pose( 0.2166, 0.2546-0.001, Z_TBD, rx, ry, 0.0, 0.0 );
    poses[0][1][1] = build_pose( 0.1540, 0.2546-0.001, Z_TBD, rx, ry, 0.0, 0.0 );

    poses[0][2][0] = build_pose( 0.2166, 0.3179-0.001, Z_TBD, rx, ry, 0.0, 0.0 );
    poses[0][2][1] = build_pose( 0.1540, 0.3179-0.001, Z_TBD, rx, ry, 0.0, 0.0 );

    poses[0][3][0] = build_pose( 0.2166, 0.3809-0.001, Z_TBD, rx, ry, 0.0, 0.0 );
    poses[0][3][1] = build_pose( 0.1540, 0.3809-0.001, Z_TBD, rx, ry, 0.0, 0.0 );

    // ===== Matrix 2 (index 1) =====
    poses[1][0][0] = build_pose( -0.1549, 0.1887, Z_TBD, rx, ry, 0.0, 0.0 );
    poses[1][0][1] = build_pose( -0.2180, 0.1887, Z_TBD, rx, ry, 0.0, 0.0 );

    poses[1][1][0] = build_pose( -0.1549, 0.2521, Z_TBD, rx, ry, 0.0, 0.0 );
    poses[1][1][1] = build_pose( -0.2180, 0.2521, Z_TBD, rx, ry, 0.0, 0.0 );

    poses[1][2][0] = build_pose( -0.1549, 0.3167, Z_TBD, rx, ry, 0.0, 0.0 );
    poses[1][2][1] = build_pose( -0.2180, 0.3167, Z_TBD, rx, ry, 0.0, 0.0 );

    poses[1][3][0] = build_pose( -0.1549, 0.3789, Z_TBD, rx, ry, 0.0, 0.0 );
    poses[1][3][1] = build_pose( -0.2180, 0.3789, Z_TBD, rx, ry, 0.0, 0.0 );

    return poses;
}

bool go_to_pose_and_aproach(moveit::planning_interface::MoveGroupInterface &move_group, geometry_msgs::msg::Pose &pose,
                            double z_delta, bool aproach, std::shared_ptr<rclcpp::Node> &node){
  std::vector<geometry_msgs::msg::Pose> waypoints;

  geometry_msgs::msg::Pose start = move_group.getCurrentPose().pose;
  waypoints.push_back(start);

  waypoints.push_back(pose);

  if (aproach) {
    auto p_down = pose;
    p_down.position.z = p_down.position.z - z_delta - 0.002;
    waypoints.push_back(p_down);
  }

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
    return false;
  }

  // ---- Time-parameterize the trajectory ----
  robot_trajectory::RobotTrajectory rt(
      move_group.getRobotModel(), move_group.getName());
  rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_msg);

  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  bool time_ok = iptp.computeTimeStamps(rt,
                                        0.2,
                                        0.2);

  if (!time_ok) {
    RCLCPP_ERROR(node->get_logger(), "Time parameterization failed. Not executing.");
    return false;
  }

  rt.getRobotTrajectoryMsg(trajectory_msg);

  // ---- Execute ----
  moveit::planning_interface::MoveGroupInterface::Plan plan2;
  plan2.trajectory_ = trajectory_msg;

  RCLCPP_INFO(node->get_logger(), "Executing Cartesian trajectory...");
  auto result = move_group.execute(plan2);

  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Execution failed.");
    return false;
  } else {
    RCLCPP_INFO(node->get_logger(), "Done.");
  }
  return true;
}

bool gripper_toggle_suction(rclcpp::Node::SharedPtr &node, rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr &client, int pin, bool &current_state) {
  current_state = !current_state;

  auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
  request->fun   = ur_msgs::srv::SetIO::Request::FUN_SET_DIGITAL_OUT;
  request->pin   = pin;
  request->state = current_state ? 1.0 : 0.0;

  auto future = client->async_send_request(request);

  // Executor is already spinning in your background thread.
  if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
    RCLCPP_ERROR(node->get_logger(), "set_io call timed out");
    return false;
  }

  auto resp = future.get();
  if (!resp->success) {
    RCLCPP_ERROR(node->get_logger(), "set_io responded success=false");
    return false;
  }

  return true;
}


bool retreat_from_aproach(moveit::planning_interface::MoveGroupInterface &move_group, double z_delta, std::shared_ptr<rclcpp::Node> &node){
  std::vector<geometry_msgs::msg::Pose> waypoints;

  geometry_msgs::msg::Pose start = move_group.getCurrentPose().pose;
  waypoints.push_back(start);

  auto p_up = start;
  p_up.position.z = p_up.position.z + z_delta + 0.002;
  waypoints.push_back(p_up);

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
    return false;
  }

  // ---- Time-parameterize the trajectory ----
  robot_trajectory::RobotTrajectory rt(
      move_group.getRobotModel(), move_group.getName());
  rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_msg);

  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  bool time_ok = iptp.computeTimeStamps(rt,
                                        0.2,
                                        0.2);

  if (!time_ok) {
    RCLCPP_ERROR(node->get_logger(), "Time parameterization failed. Not executing.");
    return false;
  }

  rt.getRobotTrajectoryMsg(trajectory_msg);

  // ---- Execute ----
  moveit::planning_interface::MoveGroupInterface::Plan plan2;
  plan2.trajectory_ = trajectory_msg;

  RCLCPP_INFO(node->get_logger(), "Executing Cartesian trajectory...");
  auto result = move_group.execute(plan2);

  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Execution failed.");
    return false;
  } else {
    RCLCPP_INFO(node->get_logger(), "Done.");
  }
  return true;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "basic_piece_juggler_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto tool_io_client =
  node->create_client<ur_msgs::srv::SetIO>("/io_and_status_controller/set_io");

  if (!tool_io_client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_FATAL(node->get_logger(), "set_io service not available");
    rclcpp::shutdown();
    return 1;
  }

  bool tool_on = false;
  int tool_pin = 17;

  // Run ROS callbacks in a background thread
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  // ---- MoveIt setup ----
  static const std::string PLANNING_GROUP = "ur_manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  move_group.setEndEffectorLink("tool0");
  move_group.setPoseReferenceFrame("base");

  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(50);

  // Define target joint values
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

  geometry_msgs::msg::Pose start = move_group.getCurrentPose().pose;

    RCLCPP_INFO(node->get_logger(), "start pose: p=(%.3f, %.3f, %.3f) q=(%.3f, %.3f, %.3f, %.3f)",
              start.position.x, start.position.y, start.position.z,
              start.orientation.x, start.orientation.y,
              start.orientation.z, start.orientation.w);

  auto rx = start.orientation.x;
  auto ry = start.orientation.y;
  auto rz = start.orientation.z;
  auto rw = start.orientation.w;

  double Z_TBD = 0.2156;
  double z_pickup = 0.1878;

  double z_delta = Z_TBD - z_pickup;

  auto pose_drop_to_Z = build_pose( start.position.x, start.position.y, Z_TBD, rx, ry, rz, rw );

  std::vector<std::vector<std::vector<geometry_msgs::msg::Pose>>> poses = build_poses_3d_vector(Z_TBD, rx, ry);

  if (go_to_pose_and_aproach(move_group, pose_drop_to_Z, z_delta, false, node) == false) {
    rclcpp::shutdown();
    spinner.join();
    return 1;  
  }

  std::string line;

  while (rclcpp::ok())
  {
    //            Matrix Line Column
    std::cout << "Enter M L C (or 's' to stop): ";
    std::getline(std::cin, line);

    // Exit condition
    if (line == "s")
    {
      RCLCPP_INFO(node->get_logger(), "Stopping input loop.");
      break;
    }

    double m, l, c;
    std::stringstream ss(line);

    if (!(ss >> m >> l >> c))
    {
      RCLCPP_WARN(node->get_logger(), "Invalid input. Expected 3 numbers or 's'.");
      continue;
    }

    RCLCPP_INFO(
      node->get_logger(),
      "Received values: M=%.3f L=%.3f C=%.3f",
      m, l, c
    );

    if (go_to_pose_and_aproach(move_group, poses[m-1][l-1][c-1], z_delta, true, node) == false) {
      rclcpp::shutdown();
      spinner.join();
      return 1;  
    }

    if (gripper_toggle_suction(node, tool_io_client, tool_pin, tool_on) == false) {
      rclcpp::shutdown();
      spinner.join();
      return 1; 
    }

    rclcpp::sleep_for(std::chrono::milliseconds(400));
    
    if (retreat_from_aproach(move_group, z_delta, node) == false) {
      rclcpp::shutdown();
      spinner.join();
      return 1;  
    }
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}