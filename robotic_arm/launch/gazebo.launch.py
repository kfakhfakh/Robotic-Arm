from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
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

    # Launch Gazebo Classic with a default world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"),
                "launch",
                "gazebo.launch.py"  # Default Gazebo launch file
            ])
        ])
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

        # Delay spawning robot to ensure Gazebo is ready
        TimerAction(
            period=5.0,  # Wait 5 seconds before spawning
            actions=[
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    arguments=[
                        "-file", Command(["xacro ", urdf_path]),  # Convert XACRO to XML
                        "-entity", "robotic_arm",  # Specify the entity name
                        "-x", "0", "-y", "0", "-z", "0.5"  # Position
                    ]
                )
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

        # Launch Gazebo Classic
        gz_sim
    ])
