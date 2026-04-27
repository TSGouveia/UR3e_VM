#include <memory>
#include <vector>
#include <iostream>
#include <random>
#include <array>
#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <ur_msgs/srv/set_io.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>

static constexpr int M = 2, R = 4, C = 2;
using Board = std::array<std::array<std::array<bool, C>, R>, M>; // true = occupied

struct Slot {
  int m; // 0 or 1
  int r; // 0..3
  int c; // 0..1
};

class Picker {
public:
  explicit Picker(uint32_t seed = std::random_device{}()) : rng_(seed) {}

  template <class T>
  const T& pick_one(const std::vector<T>& v) {
    std::uniform_int_distribution<size_t> dist(0, v.size()-1);
    return v[dist(rng_)];
  }

private:
  std::mt19937 rng_;
};

std::vector<int> non_empty_matrices(const Board& occ) {
  std::vector<int> mats;
  for (int m=0; m<M; ++m) {
    bool any=false;
    for (int r=0; r<R; ++r)
      for (int c=0; c<C; ++c)
        any |= occ[m][r][c];
    if (any) mats.push_back(m);
  }
  return mats;
}

std::vector<Slot> occupied_slots_in_matrix(const Board& occ, int m) {
  std::vector<Slot> v;
  for (int r=0; r<R; ++r)
    for (int c=0; c<C; ++c)
      if (occ[m][r][c]) v.push_back({m,r,c});
  return v;
}

std::vector<Slot> empty_slots_in_matrix(const Board& occ, int m) {
  std::vector<Slot> v;
  for (int r=0; r<R; ++r)
    for (int c=0; c<C; ++c)
      if (!occ[m][r][c]) v.push_back({m,r,c});
  return v;
}

geometry_msgs::msg::Pose build_pose (double x, double y, double z, double rx, double ry, double rz, double rw) {
  geometry_msgs::msg::Pose target;
  target.position.x = x;
  target.position.y = y;
  target.position.z = z;

  // Orientation as quaternion (w,x,y,z).
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
  RCLCPP_INFO(node->get_logger(), "pushed start pose: p=(%.3f, %.3f, %.3f) q=(%.3f, %.3f, %.3f, %.3f)",
              start.position.x, start.position.y, start.position.z,
              start.orientation.x, start.orientation.y,
              start.orientation.z, start.orientation.w);

  waypoints.push_back(pose);
  RCLCPP_INFO(node->get_logger(), "pushed target pose: p=(%.3f, %.3f, %.3f) q=(%.3f, %.3f, %.3f, %.3f)",
              pose.position.x, pose.position.y, pose.position.z,
              pose.orientation.x, pose.orientation.y,
              pose.orientation.z, pose.orientation.w);

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

  // ---- Time-parameterize the trajector ----
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

bool gripper_toggle_suction(
  const rclcpp::Node::SharedPtr &node,
  const rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr &client,
  int pin,
  bool &current_state) {
  current_state = !current_state;

  auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
  request->fun   = ur_msgs::srv::SetIO::Request::FUN_SET_DIGITAL_OUT;
  request->pin   = pin;
  request->state = current_state ? 1.0 : 0.0;

  auto future = client->async_send_request(request);

  if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
    RCLCPP_ERROR(node->get_logger(), "set_io call timed out");
    return false;
  }

  auto resp = future.get();
  if (!resp->success) {
    RCLCPP_ERROR(node->get_logger(), "set_io responded success=false");
    return false;
  }

  rclcpp::sleep_for(std::chrono::milliseconds(300)); // vacuum build

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

bool do_pick_or_drop(moveit::planning_interface::MoveGroupInterface& mg,
             const geometry_msgs::msg::Pose& target,
             double z_delta,
             const rclcpp::Node::SharedPtr& node,
             const rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr& io,
             int tool_pin,
             bool &tool_on)
{
  auto pose = target;
  if (!go_to_pose_and_aproach(mg, pose, z_delta, true, const_cast<rclcpp::Node::SharedPtr&>(node)))
    return false;

  // Turn vacuum ON (or toggle logic – better: explicit ON)
  // Prefer explicit set rather than toggle to avoid desync:
  // set_tool(io, pin, 1)
  if (!gripper_toggle_suction(const_cast<rclcpp::Node::SharedPtr&>(node),
                              const_cast<rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr&>(io),
                              tool_pin, tool_on))
    return false;

  if (!retreat_from_aproach(mg, z_delta, const_cast<rclcpp::Node::SharedPtr&>(node)))
    return false;

  return true;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "random_piece_juggler_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::on_shutdown([&]() {
    RCLCPP_INFO(rclcpp::get_logger("shutdown"), "ROS shutdown requested");
  });

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
  rclcpp::executors::MultiThreadedExecutor executor;
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
  {rclcpp::sleep_for(std::chrono::milliseconds(300)); // vacuum build
    auto const exec_ok = (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!exec_ok) RCLCPP_ERROR(node->get_logger(), "Execution failed.");
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Planning failed. Aborting.");
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
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
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;  
  }

  Board occupied{};
  for (int r = 0; r < 2; ++r)
  for (int c = 0; c < 2; ++c)
    occupied[0][r][c] = true;

  Picker picker;

  Slot prev_from;
  prev_from.m = M+1;
  prev_from.r = R+1;
  prev_from.c = C+1;

  while (rclcpp::ok())
  {
    auto mats = non_empty_matrices(occupied);
    if (mats.empty()) {
        RCLCPP_WARN(node->get_logger(), "Nothing to move. Stopping.");
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1; 
    }

    int src_m = picker.pick_one(mats);
    int dst_m = 1 - src_m; // “the other matrix”

    auto src_occ = occupied_slots_in_matrix(occupied, src_m);
    auto dst_empty = empty_slots_in_matrix(occupied, dst_m);

    if (dst_empty.empty()) {
        // Destination full: either choose another src matrix, or stop.
        RCLCPP_WARN(node->get_logger(), "Destination matrix %d full. Stopping.", dst_m);
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1; 
    }

    Slot from = picker.pick_one(src_occ);
    if ((prev_from.m == from.m) && (prev_from.r == from.r) && (prev_from.c == from.c)) continue;

    Slot to = picker.pick_one(dst_empty);

    auto& pick_pose  = poses[from.m][from.r][from.c];
    auto& place_pose = poses[to.m][to.r][to.c];

    // 1) PICK
    if (!do_pick_or_drop(move_group, pick_pose, z_delta, node, tool_io_client, tool_pin, tool_on)) {
        RCLCPP_ERROR(node->get_logger(), "Pick failed. Not updating board.");
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1; 
    }

    // 2) PLACE
    if (!do_pick_or_drop(move_group, place_pose, z_delta, node, tool_io_client, tool_pin, tool_on)) {
        RCLCPP_ERROR(node->get_logger(), "Place failed. Recovery needed; board not updated.");
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;  
    }

    // 3) UPDATE BOARD (only after both succeeded)
    occupied[from.m][from.r][from.c] = false;
    occupied[to.m][to.r][to.c] = true;

    prev_from = to;
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}