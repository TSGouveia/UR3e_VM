#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
      "target_pose_pub",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto logger = node->get_logger();

  // Spin in background (important!)
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });

  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface move_group(node, "ur_manipulator");

  // Optional but recommended
  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(50);
  //move_group.setMaxVelocityScalingFactor(0.2);
  //move_group.setMaxAccelerationScalingFactor(0.2);

  geometry_msgs::msg::Pose target_pose;
  
  target_pose.position.x = -0.0013;
  target_pose.position.y = 0.2905;
  target_pose.position.z = 0.5602;

  target_pose.orientation.x = -0.7086;
  target_pose.orientation.y = -0.002;
  target_pose.orientation.z = 0.000;
  target_pose.orientation.w = 0.705;

  // Example: point tool down-ish (adjust to your setup)
  //tf2::Quaternion q;
  //q.setRPY(1.638, 2.638, 0.0);  // roll, pitch, yaw
  //q.normalize();
  //target_pose.orientation = tf2::toMsg(q);

  move_group.setPoseReferenceFrame("base");

  RCLCPP_INFO(logger, "Target frame (assumed planning frame): %s",
              move_group.getPoseReferenceFrame().c_str());

  RCLCPP_INFO(logger, "Target pose: p=(%.3f, %.3f, %.3f) q=(%.3f, %.3f, %.3f, %.3f)",
              target_pose.position.x, target_pose.position.y, target_pose.position.z,
              target_pose.orientation.x, target_pose.orientation.y,
              target_pose.orientation.z, target_pose.orientation.w);

  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(target_pose, "tool0");
  //move_group.setApproximateJointValueTarget(target_pose, "tool0");


  RCLCPP_INFO(logger, "EE link: %s", move_group.getEndEffectorLink().c_str());

  MoveGroupInterface::Plan plan;
  auto const ok = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (ok)
  {
    auto const exec_ok = (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!exec_ok) RCLCPP_ERROR(logger, "Execution failed.");
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed.");
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
