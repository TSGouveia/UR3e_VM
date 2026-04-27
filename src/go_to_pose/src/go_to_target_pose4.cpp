#include <memory>
#include <thread>
#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

static void normalizeQuat(geometry_msgs::msg::Quaternion& q)
{
  const double n = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
  if (n > 1e-12) { q.x/=n; q.y/=n; q.z/=n; q.w/=n; }
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
      "target_pose_pub",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger = node->get_logger();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });

  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface move_group(node, "ur3e_manipulator");

  move_group.setPoseReferenceFrame("base");
  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(20);

  // Give CurrentStateMonitor time to receive joint_states
  rclcpp::sleep_for(std::chrono::milliseconds(800));
  move_group.setStartStateToCurrentState();

  RCLCPP_INFO(logger, "Planning frame: %s", move_group.getPoseReferenceFrame().c_str());
  RCLCPP_INFO(logger, "EE link: %s", move_group.getEndEffectorLink().c_str());

  // -----------------------------
  // Your target pose (final pose)
  // -----------------------------
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = -0.179;
  target_pose.position.y = -0.320;
  target_pose.position.z = 0.291;

  target_pose.orientation.x = 0.429;
  target_pose.orientation.y = 0.902;
  target_pose.orientation.z = 0.0;
  target_pose.orientation.w = 0.023;
  normalizeQuat(target_pose.orientation);

  // -----------------------------
  // Rethink: define a PRE-POSE
  // Approach along +Z in base frame by some distance (e.g. 10 cm)
  // -----------------------------
  const double approach_dist = 0.10; // 10 cm
  geometry_msgs::msg::Pose pre_pose = target_pose;
  pre_pose.position.z += approach_dist;

  // IMPORTANT: For Cartesian approach, lock orientation to something feasible.
  // Easiest: use the pre_pose orientation (same as target here).
  // If your target orientation is causing trouble, lock to current orientation:
  // auto cur = move_group.getCurrentPose("tool0").pose;
  // pre_pose.orientation = cur.orientation;
  // target_pose.orientation = cur.orientation;

  RCLCPP_INFO(logger,
              "Target pose: p=(%.3f, %.3f, %.3f) q=(%.3f, %.3f, %.3f, %.3f)",
              target_pose.position.x, target_pose.position.y, target_pose.position.z,
              target_pose.orientation.x, target_pose.orientation.y,
              target_pose.orientation.z, target_pose.orientation.w);

  // =========================================================
  // STAGE 1: Plan to PRE-POSE using normal planning (robust)
  // =========================================================
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(pre_pose, "tool0");

  MoveGroupInterface::Plan to_pre;
  bool plan_ok = (move_group.plan(to_pre) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!plan_ok)
  {
    RCLCPP_ERROR(logger, "Stage 1 failed: couldn't plan to pre-pose.");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  bool exec_ok = (move_group.execute(to_pre) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!exec_ok)
  {
    RCLCPP_ERROR(logger, "Stage 1 failed: execution to pre-pose failed.");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  // Small settle
  rclcpp::sleep_for(std::chrono::milliseconds(200));

  // =========================================================
  // STAGE 2: Short Cartesian approach (pre_pose -> target_pose)
  // =========================================================
  move_group.setStartStateToCurrentState();

  std::vector<geometry_msgs::msg::Pose> waypoints;

  // Start waypoint should match current pose (important for Cartesian)
  geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose("tool0").pose;
  normalizeQuat(start_pose.orientation);

  // Lock orientations exactly (avoid orientation interpolation surprises)
  pre_pose.orientation    = start_pose.orientation;
  target_pose.orientation = start_pose.orientation;

  waypoints.push_back(start_pose);
  waypoints.push_back(target_pose);

  moveit_msgs::msg::RobotTrajectory cart_traj;
  const double eef_step = 0.002;     // 2 mm (more reliable)
  const double jump_threshold = 0.0;

  double fraction = move_group.computeCartesianPath(
      waypoints,
      eef_step,
      jump_threshold,
      cart_traj,
      false // avoid_collisions=false (i.e., DO collision checking)
  );

  RCLCPP_INFO(logger, "Cartesian approach fraction: %.2f%%", fraction * 100.0);

  if (fraction < 0.95)
  {
    RCLCPP_ERROR(logger, "Stage 2 failed: Cartesian approach incomplete.");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  MoveGroupInterface::Plan cart_plan;
  cart_plan.trajectory_ = cart_traj;

  exec_ok = (move_group.execute(cart_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!exec_ok)
  {
    RCLCPP_ERROR(logger, "Stage 2 failed: Cartesian execution failed.");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  // =========================================================
  // Optional: Cartesian retreat (target_pose -> pre_pose)
  // =========================================================
  rclcpp::sleep_for(std::chrono::milliseconds(200));
  move_group.setStartStateToCurrentState();

  geometry_msgs::msg::Pose now_pose = move_group.getCurrentPose("tool0").pose;
  normalizeQuat(now_pose.orientation);

  geometry_msgs::msg::Pose retreat = now_pose;
  retreat.position.z += 0.08; // 8 cm up
  retreat.orientation = now_pose.orientation;

  waypoints.clear();
  waypoints.push_back(now_pose);
  waypoints.push_back(retreat);

  moveit_msgs::msg::RobotTrajectory retreat_traj;
  fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, retreat_traj, false);
  RCLCPP_INFO(logger, "Cartesian retreat fraction: %.2f%%", fraction * 100.0);

  if (fraction > 0.95)
  {
    MoveGroupInterface::Plan retreat_plan;
    retreat_plan.trajectory_ = retreat_traj;
    move_group.execute(retreat_plan);
  }
  else
  {
    RCLCPP_WARN(logger, "Retreat incomplete; skipping retreat execution.");
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
