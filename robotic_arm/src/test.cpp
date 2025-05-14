#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>
void move_robot(const std::shared_ptr<rclcpp::Node> node)
{
    auto arm_move_group = moveit::planning_interface::MoveGroupInterface(node, "arm_group");

    /********************      set x,y,z coordinates of the tip_link    **********************/
    double x=0.130;
    double y=0.08;
    double z=0.1;
    bool arm_within_bounds = arm_move_group.setPositionTarget(x,y,z,"tip_link");
    if(!arm_within_bounds )
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Target joint position were outside the limits");
        return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    bool arm_plan_success = arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS;

    if(arm_plan_success)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planner Succeed, moving the arm and the gripper");
        arm_move_group.move();
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "One or more planners failed!");
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