#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <Eigen/Core>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("enviroment_colision_mesh_node");

    // Planning scene interface
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Create collision object
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "world";  // change if needed
    collision_object.id = "enviroment_collision_mesh";

    // Load mesh from STL
    std::string mesh_resource = "package://stl_models/meshes/UR3e_enviroment_m_v1.stl";

    Eigen::Vector3d scale(0.001, 0.001, 0.001);
    shapes::Mesh *mesh = shapes::createMeshFromResource(mesh_resource, scale);
    if (!mesh)
    {
      RCLCPP_ERROR(node->get_logger(), "Failed to load mesh");
      rclcpp::shutdown();
      return 1;
    }

    // Convert mesh to ROS message
    shape_msgs::msg::Mesh mesh_msg;
    shapes::ShapeMsg shape_msg;
    shapes::constructMsgFromShape(mesh, shape_msg);
    mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);

    // Pose of the mesh
    geometry_msgs::msg::Pose mesh_pose;
    mesh_pose.orientation.w = 0.0;
    mesh_pose.orientation.x = 0.0;
    mesh_pose.orientation.y = 0.0;
    mesh_pose.orientation.z = 1.0;

    mesh_pose.position.x = 0.0;
    mesh_pose.position.y = 0.0;
    mesh_pose.position.z = -0.001;


    collision_object.meshes.push_back(mesh_msg);
    collision_object.mesh_poses.push_back(mesh_pose);
    collision_object.operation = collision_object.ADD;

    // Apply to planning scene
    planning_scene_interface.applyCollisionObject(collision_object);

    RCLCPP_INFO(node->get_logger(), "Collision mesh added to planning scene");

    rclcpp::spin_some(node);
    rclcpp::shutdown();
    return 0;
}