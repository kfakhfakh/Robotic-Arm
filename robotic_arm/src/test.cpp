#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <memory>

void move_robot(const std::shared_ptr<rclcpp::Node> node)
{
    auto arm_move_group = moveit::planning_interface::MoveGroupInterface(node, "arm_group");
    arm_move_group.setGoalPositionTolerance(0.01);
    arm_move_group.setGoalOrientationTolerance(0.01);  // Optional: Set orientation tolerance

    // Define the target pose (position + orientation)
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.130;
    target_pose.position.y = 0;
    target_pose.position.z = 0.2;

    // Orientation in quaternion (Horizental)
    target_pose.orientation.x = -0.707;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = -0.707;

    arm_move_group.setPoseTarget(target_pose, "tip_link");

    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    bool arm_plan_success = arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS;

    if (arm_plan_success)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planner succeeded, executing motion...");
        arm_move_group.move();
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Planning failed!");
        return;
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("simple_moveit_interface");

    move_robot(node);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
