#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    
    # Package directories
    pkg_husky_minimal = get_package_share_directory('husky_minimal')
    pkg_tb3_multi = get_package_share_directory('tb3_multi_robot')
    
    # Paths
    world_file = os.path.join(pkg_tb3_multi, 'worlds', 'tb3_world.world')
    urdf_file = os.path.join(pkg_husky_minimal, 'urdf', 'husky_ros2.urdf.xacro')
    nav2_params_file = os.path.join(pkg_husky_minimal, 'config', 'nav2_params.yaml')
    custom_nav2_launch = os.path.join(pkg_husky_minimal, 'launch', 'custom_nav2.launch.py')
    
    # Use absolute path to map in source directory
    workspace_dir = os.path.join(os.path.dirname(pkg_husky_minimal), '..', '..', '..')
    map_file = os.path.join(workspace_dir, 'src', 'husky_minimal', 'map', 'map.yaml')
    
    print(f"Using map file: {map_file}")
    print(f"Map file exists: {os.path.exists(map_file)}")
    
    # Process URDF with xacro
    robot_description = xacro.process_file(urdf_file).toxml()
    
    # Launch Gazebo with TurtleBot3 world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': [f'-r -v 4 {world_file}'],
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description
        }]
    )
    
    # Spawn Husky in the world
    spawn_husky = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'husky',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.3'
        ],
        output='screen'
    )
    
    # ROS-Gazebo Bridge for clock
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    
    # Bridge for IMU
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
        remappings=[('/imu', '/imu/data')],
        output='screen'
    )
    
    # Bridge for Lidar
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        output='screen'
    )
    
    # Bridge for cmd_vel
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen'
    )
    
    # Bridge for odometry
    odom_data_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        output='screen'
    )
    
    # Bridge for joint_states
    joint_states_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'],
        output='screen'
    )
    
    # Bridge for TF from Gazebo
    tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        output='screen'
    )
    
    # Custom Nav2 Launch
    custom_nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(custom_nav2_launch),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'true',
            'params_file': nav2_params_file,
            'autostart': 'true',
        }.items()
    )
    
    # Delay Nav2 launch to ensure everything else is ready
    delayed_nav2 = TimerAction(
        period=8.0,
        actions=[custom_nav2_bringup]
    )
    
    # RViz with Nav2 config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'rviz', 'nav2_default_view.rviz'])],
        parameters=[{'use_sim_time': True}]
    )
    
    # Delayed RViz
    delayed_rviz = TimerAction(
        period=10.0,
        actions=[rviz_node]
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_husky,
        clock_bridge,
        imu_bridge,
        lidar_bridge,
        cmd_vel_bridge,
        odom_data_bridge,
        joint_states_bridge,
        tf_bridge,
        # odom_bridge,
        # base_link_bridge,
        delayed_nav2,
        delayed_rviz,
    ])