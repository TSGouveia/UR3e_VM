#include <memory>
#include <vector>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "calibration_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));


  // Run ROS callbacks in a background thread (MoveGroupInterface needs spinning)
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

  // joint pose. depends on setup
  std::vector<double> joint_target = {
    1.1001,
    -1.8619,
    1.7854,
    4.7859,
    -1.5715,
    5.6838
  };
  move_group.setJointValueTarget(joint_target);

  RCLCPP_INFO(node->get_logger(), "Starting callibration.");

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
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Finished callibration.");

  rclcpp::shutdown();
  spinner.join();
  return 0;
}