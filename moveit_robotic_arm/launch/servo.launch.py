import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Launch arguments
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )
    is_sim = LaunchConfiguration("is_sim")

    # MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("robotic_arm", package_name="moveit_robotic_arm")
        .robot_description(file_path=os.path.join(
            get_package_share_directory("robotic_arm"), 
            "urdf", 
            "Robot_Arm.xacro"))
        .robot_description_semantic(file_path="config/robotic_arm.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    # Servo configuration
    servo_config = os.path.join(
        get_package_share_directory('moveit_robotic_arm'),
        'config',
        'servo.yaml'
    )

    # ROS2 Control
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_robotic_arm"),
        "config",
        "ros2_controllers.yaml",
    )

    # Nodes
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_group_controller", "-c", "/controller_manager"],
    )

    # Main Servo node (using servo_node_main instead of demo)
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_server",
        parameters=[
            servo_config,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": is_sim}
        ],
        output="screen",
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": is_sim},
            {"publish_robot_description_semantic": True},
            {"publish_robot_state": True},
            {"publish_planning_scene": True}
        ],
    )

    # RViz
    rviz_config_file = os.path.join(
        get_package_share_directory("moveit_robotic_arm"), 
        "config", 
        "moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": is_sim}
        ],
    )

    return LaunchDescription([
        is_sim_arg,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        servo_node,
        move_group_node,
        rviz_node,
    ])