#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "target_pose_pub", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("target_pose_pub");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur3e_manipulator");

  // Set a target Pose
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;

    msg.position.x = -0.02141332565002207;
    msg.position.y = -0.15616598191262503;
    msg.position.z = 0.815864175093933;

    //tf2::Quaternion q(0.018797508230006338, 0.7091776010808645, -0.7044195297127017, 0.022514661198190837);
    //q.normalize(); // Ensures magnitude = 1

    //msg.orientation.x = q.x();
    //msg.orientation.y = q.y();
    //msg.orientation.z = q.z();
    //msg.orientation.w = q.w();

    msg.orientation.x = 0.0;
    msg.orientation.y = 1.0;
    msg.orientation.z = 0.0;
    msg.orientation.w = 0.0;

    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);
  //move_group_interface.setNamedTarget("home");

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success)
  {
    move_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}