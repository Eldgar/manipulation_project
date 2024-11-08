#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char **argv)
{
    // Initialize ROS
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("move_to_pose_node");

    // Create the MoveGroupInterface for the UR3e manipulator
    static const std::string PLANNING_GROUP = "ur_manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

    // Optional: Set the planner ID (e.g., RRTConnect)
    // move_group.setPlannerId("RRTConnectkConfigDefault");

    // Set maximum velocity and acceleration scaling factors (optional)
    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);

    // Define the target pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.4; // Adjust these values
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.2;

    // Set orientation (quaternion)
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 1.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.0;

    // Set the target pose
    move_group.setPoseTarget(target_pose);

    // Plan to the target pose
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
        RCLCPP_INFO(node->get_logger(), "Planning successful. Executing...");
        move_group.execute(my_plan);
        RCLCPP_INFO(node->get_logger(), "Motion execution complete.");
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Planning failed.");
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}

