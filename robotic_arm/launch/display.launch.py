from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=PathJoinSubstitution([
            FindPackageShare("robotic_arm"),
            "urdf",
            "Robot_Arm.xacro"
        ]),
        description="Absolute path to the robot URDF file"
    )
    
    world_arg = DeclareLaunchArgument(
        name="world",
        default_value="empty.sdf",
        description="SDF world file"
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation time"
    )

    # Set environment variables for Ignition
    ign_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=os.path.join(get_package_share_directory("robotic_arm"), "models") + ":" + 
              os.path.join(get_package_share_directory("robotic_arm"), "worlds")
    )

    # Robot description parameter
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str
    )

    # Robot state publisher node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ],
        output="screen"
    )

    # Start Ignition Gazebo
    start_ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])
        ]),
        launch_arguments=[
            ("gz_args", ["-r -v4 ", LaunchConfiguration("world")])
        ]
    )

    # Spawn robot in Ignition
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "robotic_arm",
            "-z", "0.1"
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen"
    )

    # Bridge between ROS 2 and Ignition Gazebo
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model",
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen"
    )

    # Joint state publisher (if needed for manual control)
    use_joint_state_publisher_gui_arg = DeclareLaunchArgument(
        name="use_joint_state_publisher_gui",
        default_value="false",
        description="Start joint state publisher GUI"
    )
    
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("use_joint_state_publisher_gui"))
    )

    return LaunchDescription([
        model_arg,
        world_arg,
        use_sim_time_arg,
        use_joint_state_publisher_gui_arg,
        ign_resource_path,
        robot_state_publisher,
        start_ignition_gazebo,
        spawn_robot,
        bridge,
        joint_state_publisher_gui
    ])