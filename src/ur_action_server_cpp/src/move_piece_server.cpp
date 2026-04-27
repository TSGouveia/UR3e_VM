#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

//action related
#include "rclcpp_action/rclcpp_action.hpp"

#include "ur_action_interface/action/move_piece.hpp"

// moveit related
#include <ur_msgs/srv/set_io.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>

// For time parameterization
#include <moveit/trajectory_processing/iterative_time_parameterization.h>



using namespace std::chrono_literals;

class MovePieceServer : public rclcpp::Node
{
public:
  using MovePiece = ur_action_interface::action::MovePiece;
  using GoalHandleMovePiece = rclcpp_action::ServerGoalHandle<MovePiece>;

  explicit MovePieceServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : rclcpp::Node("move_piece_server", options)
  {
    // io init
    io_client_ = this->create_client<ur_msgs::srv::SetIO>("/io_and_status_controller/set_io");
    tool_pin = 17;

    // action server init
    using namespace std::placeholders;
    action_server_ = rclcpp_action::create_server<MovePiece>(
      this,
      "move_piece",
      std::bind(&MovePieceServer::handle_goal, this, _1, _2),
      std::bind(&MovePieceServer::handle_cancel, this, _1),
      std::bind(&MovePieceServer::handle_accepted, this, _1));

    RCLCPP_INFO(get_logger(), "server node built.");
  }

  void init_moveit_and_poses()
  {
    const std::string group_name = this->declare_parameter<std::string>("move_group", "ur_manipulator");
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), group_name);

    move_group_->setPlanningTime(5.0);
    move_group_->setNumPlanningAttempts(30);

    move_group_->startStateMonitor();
    if (!move_group_->getCurrentState(/*rclcpp::Clock().now(),*/ 2.0)) {
      RCLCPP_ERROR(get_logger(), "No current state received from /joint_states");
    }

    geometry_msgs::msg::Pose start = move_group_->getCurrentPose().pose;

    auto rx = start.orientation.x;
    auto ry = start.orientation.y;
    auto rz = start.orientation.z;
    auto rw = start.orientation.w;

    Z_TBD    = 0.2156;
    z_pickup = 0.1878;

    pose_drop_to_Z = build_pose(start.position.x, start.position.y, Z_TBD, rx, ry, rz, rw);

    poses = build_poses_3d_vector(Z_TBD, rx, ry, rz, rw);
  }

private:
  rclcpp_action::Server<MovePiece>::SharedPtr action_server_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr io_client_;
  int tool_pin;
  std::vector<std::vector<std::vector<geometry_msgs::msg::Pose>>> poses;
  double Z_TBD;
  double z_pickup;
  geometry_msgs::msg::Pose pose_drop_to_Z;

  std::mutex exec_mutex_; // protect MoveIt calls

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const MovePiece::Goal> goal)
  {
    if (goal->speed <= 0.0f || goal->speed > 1.0f) {
      RCLCPP_WARN(get_logger(), "Rejecting goal: speed out of range (0,1]. speed=%.3f", goal->speed);
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(
      get_logger(),
      "Received goal: from=(%d,%d,%d) to=(%d,%d,%d) speed=%.2f",
      goal->from_m, goal->from_r, goal->from_c,
      goal->to_m, goal->to_r, goal->to_c,
      goal->speed);

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMovePiece> /*goal_handle*/)
  {
    RCLCPP_WARN(get_logger(), "Cancel request received");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMovePiece> goal_handle)
  {
    // Run execution in a separate thread so we don't block the executor.
    std::thread{std::bind(&MovePieceServer::execute, this, goal_handle)}.detach();
  }

  // -------- helper functions ------------

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

  // this is the pose 3D vector. depends on setup.
  std::vector<std::vector<std::vector<geometry_msgs::msg::Pose>>> build_poses_3d_vector (double Z_TBD, double rx, double ry, double rz, double rw){     
    std::vector<std::vector<std::vector<geometry_msgs::msg::Pose>>> poses;
    // Two matrices, 4 rows, 2 columns each
    poses.resize(2, std::vector<std::vector<geometry_msgs::msg::Pose>>(
                  4, std::vector<geometry_msgs::msg::Pose>(2)));
    
    // ===== Matrix 1 (index 0) =====
    poses[0][0][0] = build_pose( 0.2166, 0.1923-0.002, Z_TBD, rx, ry, rz, rw );
    poses[0][0][1] = build_pose( 0.1540, 0.1923-0.002, Z_TBD, rx, ry, rz, rw );
    poses[0][1][0] = build_pose( 0.2166, 0.2546-0.001, Z_TBD, rx, ry, rz, rw );
    poses[0][1][1] = build_pose( 0.1540, 0.2546-0.001, Z_TBD, rx, ry, rz, rw );
    poses[0][2][0] = build_pose( 0.2166, 0.3179-0.001, Z_TBD, rx, ry, rz, rw );
    poses[0][2][1] = build_pose( 0.1540, 0.3179-0.001, Z_TBD, rx, ry, rz, rw );
    poses[0][3][0] = build_pose( 0.2166, 0.3809-0.001, Z_TBD, rx, ry, rz, rw );
    poses[0][3][1] = build_pose( 0.1540, 0.3809-0.001, Z_TBD, rx, ry, rz, rw );

    // ===== Matrix 2 (index 1) =====
    poses[1][0][0] = build_pose( -0.1549, 0.1887, Z_TBD, rx, ry, rz, rw );
    poses[1][0][1] = build_pose( -0.2180, 0.1887, Z_TBD, rx, ry, rz, rw );
    poses[1][1][0] = build_pose( -0.1549, 0.2521, Z_TBD, rx, ry, rz, rw );
    poses[1][1][1] = build_pose( -0.2180, 0.2521, Z_TBD, rx, ry, rz, rw );
    poses[1][2][0] = build_pose( -0.1549, 0.3167, Z_TBD, rx, ry, rz, rw );
    poses[1][2][1] = build_pose( -0.2180, 0.3167, Z_TBD, rx, ry, rz, rw );
    poses[1][3][0] = build_pose( -0.1549, 0.3789, Z_TBD, rx, ry, rz, rw );
    poses[1][3][1] = build_pose( -0.2180, 0.3789, Z_TBD, rx, ry, rz, rw );

    return poses;
  }

  void publish_phase(const std::shared_ptr<GoalHandleMovePiece>& gh, const std::string& phase, float progress)
  {
    auto fb = std::make_shared<MovePiece::Feedback>();
    fb->phase = phase;
    fb->progress = progress;
    gh->publish_feedback(fb);
  }

  bool set_suction(bool on)
  {
    auto req = std::make_shared<ur_msgs::srv::SetIO::Request>();
    req->fun   = ur_msgs::srv::SetIO::Request::FUN_SET_DIGITAL_OUT;
    req->pin   = tool_pin;
    req->state = on ? 1.0 : 0.0;

    RCLCPP_INFO(get_logger(), "trying to set tool pin %d to %.1f", tool_pin, req->state);

    if (!io_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(get_logger(), "SetIO service not available");
      return false;
    }

    auto future = io_client_->async_send_request(req);

    if (future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "SetIO call timed out");
      return false;
    }

    auto resp = future.get();
    if (!resp->success) {
      RCLCPP_ERROR(get_logger(), "SetIO responded success=false");
      return false;
    }

    // allow vacuum to build/release
    rclcpp::sleep_for(std::chrono::milliseconds(200));

    return true;
  }

  bool go_to_pose(const geometry_msgs::msg::Pose &pose, double vel_scale, double accel_scale){
    move_group_->setEndEffectorLink("tool0");
    move_group_->setPoseReferenceFrame("base_link");

    std::vector<geometry_msgs::msg::Pose> waypoints;

    geometry_msgs::msg::Pose start = move_group_->getCurrentPose().pose;
    waypoints.push_back(start);

    RCLCPP_INFO(get_logger(), "pushed start pose: p=(%.3f, %.3f, %.3f) q=(%.3f, %.3f, %.3f, %.3f)",
              start.position.x, start.position.y, start.position.z,
              start.orientation.x, start.orientation.y,
              start.orientation.z, start.orientation.w);

    waypoints.push_back(pose);

    RCLCPP_INFO(get_logger(), "pushed target pose: p=(%.3f, %.3f, %.3f) q=(%.3f, %.3f, %.3f, %.3f)",
              pose.position.x, pose.position.y, pose.position.z,
              pose.orientation.x, pose.orientation.y,
              pose.orientation.z, pose.orientation.w);

    const double eef_step = 0.005;      // 5 mm
    const double jump_threshold = 0.0;  // disable jump threshold
    moveit_msgs::msg::RobotTrajectory trajectory_msg;

    RCLCPP_INFO(get_logger(), "Computing Cartesian path...");
    double fraction = move_group_->computeCartesianPath(
        waypoints, eef_step, jump_threshold, trajectory_msg, /*avoid_collisions=*/true);

    RCLCPP_INFO(get_logger(), "Cartesian path fraction: %.3f", fraction);

    if (fraction < 0.95) {
      RCLCPP_ERROR(get_logger(),
                  "Cartesian path incomplete (fraction %.3f). Not executing.", fraction);
      return false;
    }

    // ---- Time-parameterize the trajectory ----
    robot_trajectory::RobotTrajectory rt(
        move_group_->getRobotModel(), move_group_->getName());
    rt.setRobotTrajectoryMsg(*move_group_->getCurrentState(), trajectory_msg);

    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    bool time_ok = iptp.computeTimeStamps(rt,
                                          vel_scale,
                                          accel_scale);

    if (!time_ok) {
      RCLCPP_ERROR(get_logger(), "Time parameterization failed. Not executing.");
      return false;
    }

    rt.getRobotTrajectoryMsg(trajectory_msg);

    // ---- Execute move ----
    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    plan2.trajectory_ = trajectory_msg;

    RCLCPP_INFO(get_logger(), "Executing Cartesian trajectory...");
    auto result = move_group_->execute(plan2);

    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Execution failed.");
      return false;
    } else {
      RCLCPP_INFO(get_logger(), "Done.");
    }
    return true;
  }

  // ------------- EXECUTE ----------------

  void execute(const std::shared_ptr<GoalHandleMovePiece> gh)
  {
    std::lock_guard<std::mutex> lock(exec_mutex_);

    auto result = std::make_shared<MovePiece::Result>();
    
    const auto goal = gh->get_goal();

    const double speed = goal->speed;
    const double vel_scale = std::max(0.05, std::min(1.0, static_cast<double>(speed)));
    const double acc_scale = vel_scale;

    try {
      publish_phase(gh, "planning_pick", 0.05f);

      const auto from_above = poses[goal->from_m][goal->from_r][goal->from_c];
      const auto to_above = poses[goal->to_m][goal->to_r][goal->to_c];

      // Approach poses
      auto from_down = from_above;
      from_down.position.z = from_down.position.z - (Z_TBD - z_pickup) - 0.002;

      auto to_down = to_above;
      to_down.position.z = to_down.position.z - (Z_TBD - z_pickup) - 0.002;

      if (gh->is_canceling()) { throw std::runtime_error("Canceled"); }

      publish_phase(gh, "approach_pick", 0.15f);
      if (!go_to_pose(from_above, vel_scale, acc_scale)) {
        throw std::runtime_error("Failed to move to from_above");
      }

      publish_phase(gh, "pick_descend", 0.25f);
      if (!go_to_pose(from_down, vel_scale * 0.5, acc_scale * 0.5)) {
        throw std::runtime_error("Failed to descend to pick");
      }

      publish_phase(gh, "suction_on", 0.35f);
      if (!set_suction(true)) {
        throw std::runtime_error("Failed to enable suction");
      }

      publish_phase(gh, "lift", 0.45f);
      if (!go_to_pose(from_above, vel_scale * 0.6, acc_scale * 0.6)) {
        throw std::runtime_error("Failed to lift");
      }

      publish_phase(gh, "transit", 0.65f);
      if (!go_to_pose(to_above, vel_scale, acc_scale)) {
        throw std::runtime_error("Failed to transit to place above");
      }

      publish_phase(gh, "place_descend", 0.80f);
      if (!go_to_pose(to_down, vel_scale * 0.5, acc_scale * 0.5)) {
        throw std::runtime_error("Failed to descend to place");
      }

      publish_phase(gh, "suction_off", 0.90f);
      if (!set_suction(false)) {
        throw std::runtime_error("Failed to disable suction");
      }

      publish_phase(gh, "retreat", 0.97f);
      if (!go_to_pose(to_above, vel_scale * 0.6, acc_scale * 0.6)) {
        throw std::runtime_error("Failed to retreat");
      }

      result->success = true;
      result->message = "Pick & place completed";
      gh->succeed(result);
      publish_phase(gh, "done", 1.00f);
    }
    catch (const std::exception& e) {
      // If cancel requested, return canceled
      if (gh->is_canceling()) {
        result->success = false;
        result->message = "Canceled";
        gh->canceled(result);
      } else {
        result->success = false;
        result->message = std::string("Failed: ") + e.what();
        gh->abort(result);
      }
      RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MovePieceServer>(rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);

  // thread the init and wait for the node to spin so that we can init moveit and get starting pose
  std::thread init_thread([&]() {
    rclcpp::sleep_for(std::chrono::milliseconds(300));
    node->init_moveit_and_poses();
    RCLCPP_INFO(node->get_logger(), "S E R V E R    R E A D Y !");
  });

  exec.spin();

  rclcpp::shutdown();
  return 0;
}