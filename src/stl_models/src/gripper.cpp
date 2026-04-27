#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <Eigen/Core>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("gripper_colision_mesh_node");

    // Planning scene interface
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Load mesh from STL
    std::string mesh_resource = "package://stl_models/meshes/gripper_oriented_v1.stl";

    Eigen::Vector3d scale(0.0001, 0.0001, 0.0001);
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

    // transforming to tool0 link and set pose
    // TF2 buffer and listener
    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // Wait a little to make sure TF tree is populated
    rclcpp::sleep_for(std::chrono::seconds(1));

    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
        transform_stamped = tf_buffer.lookupTransform(
            "tool0",  // target frame
            "world",  // source frame
            rclcpp::Time(0),  // latest available
            std::chrono::seconds(1)
        );
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(node->get_logger(), "Could not get transform: %s", ex.what());
        return 1;
    }

    // Convert to tf2::Transform
    tf2::Transform world_to_tool0;
    world_to_tool0.setOrigin(tf2::Vector3(
        transform_stamped.transform.translation.x,
        transform_stamped.transform.translation.y,
        transform_stamped.transform.translation.z
    ));
    tf2::Quaternion q(
        transform_stamped.transform.rotation.x,
        transform_stamped.transform.rotation.y,
        transform_stamped.transform.rotation.z,
        transform_stamped.transform.rotation.w
    );
    world_to_tool0.setRotation(q);

    tf2::Transform mesh_in_world;
    mesh_in_world.setOrigin(tf2::Vector3(0,0,0));
    mesh_in_world.setRotation(tf2::Quaternion::getIdentity());

    tf2::Transform mesh_in_tool0 = world_to_tool0.inverse() * mesh_in_world;

    geometry_msgs::msg::Pose gripper_pose;
    tf2::Vector3 origin = mesh_in_tool0.getOrigin();
    tf2::Quaternion rotation = mesh_in_tool0.getRotation();

    gripper_pose.position.x = origin.x(); 
    gripper_pose.position.y = origin.y(); 
    gripper_pose.position.z = origin.z(); 

    gripper_pose.orientation.x = rotation.x();
    gripper_pose.orientation.y = rotation.y();
    gripper_pose.orientation.z = rotation.z();
    gripper_pose.orientation.w = rotation.w();

    moveit_msgs::msg::AttachedCollisionObject attached_object;
    attached_object.object.id = "gripper_collision_mesh";
    attached_object.link_name = "tool0";
    attached_object.object.meshes.push_back(mesh_msg);
    attached_object.object.mesh_poses.push_back(gripper_pose);
    attached_object.object.operation = attached_object.object.ADD;

    // Apply to planning scene
    planning_scene_interface.applyAttachedCollisionObject(attached_object);

    RCLCPP_INFO(node->get_logger(), "Collision mesh added to planning scene");

    rclcpp::spin_some(node);
    rclcpp::shutdown();
    return 0;
}