from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Paths
    urdf_path = PathJoinSubstitution([
        FindPackageShare("robotic_arm"),
        "urdf",
        "Robot_Arm.xacro"
    ])

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare("robotic_arm"),
        "rviz",
        "rviz_config.rviz"
    ])

    # Launch Ignition Gazebo with an empty world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])
        ]),
        launch_arguments={"gz_args": "-r empty.sdf"}.items()
    )

    return LaunchDescription([
        # Publish robot description to /robot_description
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": Command(["xacro ", urdf_path])
            }]
        ),

        # Spawn robot in Ignition using /robot_description topic
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name", "robotic_arm",
                "-topic", "robot_description",
                "-x", "0", "-y", "0", "-z", "0.5"
            ]
        ),

        # GUI for manually setting joint states (useful for testing)
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui"
        ),
    

        # Launch RViz2 with predefined config
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", rviz_config_path]
        ),

        # Launch Ignition Gazebo
        gz_sim
    ])
