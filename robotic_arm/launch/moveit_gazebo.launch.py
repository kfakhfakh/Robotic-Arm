from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Paths
    description_pkg = FindPackageShare('robotic_arm').find('robotic_arm')
    moveit_config_pkg = FindPackageShare('moveit_robotic_arm').find('moveit_robotic_arm')

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([description_pkg, 'urdf', 'Robot_Arm.xacro'])
    ])
    robot_description = {'robot_description': robot_description_content}

    # Controller parameters
    ros2_control_params = PathJoinSubstitution([moveit_config_pkg, 'config', 'ros2_controllers.yaml'])

    # Nodes
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'robotic_arm',
            '-x', '0', '-y', '0', '-z', '0',
            '-topic', '/robot_description'
        ],
        output='screen'
    )
    
    joint_state_publisher_gui=Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[robot_description]
    )   

    # Controller Manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_control_params],
        output='screen'
    )

    # Delayed controller spawners
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    spawn_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_group_controller'],
        output='screen'
    )

    # Joint state bridge
    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/joint_states@sensor_msgs/msg/JointState[ignition.msgs.JointState'],
        output='screen'
    )

    # MoveIt
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([moveit_config_pkg, 'launch', 'move_group.launch.py'])
        )
    )

    # RViz
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([moveit_config_pkg, 'launch', 'moveit_rviz.launch.py'])
        ),
        launch_arguments={
            'config': PathJoinSubstitution([moveit_config_pkg, 'config', 'moveit.rviz'])
        }.items()
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
        controller_manager,

        
        
        # Delay controller spawn after controller manager starts
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager,
                on_start=[
                    TimerAction(
                        period=7.0,
                        actions=[spawn_joint_state_broadcaster]
                    ),
                    TimerAction(
                        period=10.0,
                        actions=[spawn_arm_controller]
                    )
                ]
            )
        ),
        gz_sim,
        spawn_robot,
        joint_state_bridge,
        moveit,
        
        
    ])