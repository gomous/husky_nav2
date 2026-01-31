#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
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
            '-z', '0.3',
            '-R', '0.0',
            '-P', '0.0',  
            '-Y', '0.0'
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
    
    # Bridge for Lidar
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        output='screen'
    )
    
    # Bridge for IMU
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
        output='screen'
    )
    
    # Bridge for cmd_vel
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen'
    )
    
    # Bridge for odometry data
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
    
    # SLAM Toolbox Node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'scan_topic': '/scan',
            'mode': 'mapping',
            'solver_plugin': 'solver_plugins::CeresSolver',
            'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
            'ceres_preconditioner': 'SCHUR_JACOBI',
            'ceres_trust_strategy': 'LEVENBERG_MARQUARDT',
            'ceres_dogleg_type': 'TRADITIONAL_DOGLEG',
            'ceres_loss_function': 'None',
            'resolution': 0.05,
            'max_laser_range': 10.0,
            'minimum_time_interval': 0.5,
            'transform_publish_period': 0.02,
            'map_update_interval': 2.0,
            'debug_logging': False,
            'throttle_scans': 1,
            'do_loop_closing': True,
            'loop_search_maximum_distance': 3.0,
            'minimum_travel_distance': 0.1,
            'minimum_travel_heading': 0.1,
            'scan_buffer_size': 10,
            'use_scan_matching': True,
            'use_scan_barycenter': True,
        }]
    )
    
    # Auto-configure SLAM Toolbox after 3 seconds
    configure_slam = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
                output='screen'
            )
        ]
    )
    
    # Auto-activate SLAM Toolbox after 4 seconds
    activate_slam = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
                output='screen'
            )
        ]
    )
    
    # RViz config file
    rviz_config_file = os.path.join(pkg_husky_minimal, 'rviz', 'slam.rviz')
    
    # RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_husky,
        clock_bridge,
        lidar_bridge,
        imu_bridge,
        cmd_vel_bridge,
        odom_data_bridge,
        joint_states_bridge,
        tf_bridge,
        slam_toolbox_node,
        configure_slam,
        activate_slam,
        rviz_node,
    ])