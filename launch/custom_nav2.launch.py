#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Get the launch directory
    pkg_husky_minimal = get_package_share_directory('husky_minimal')
    
    # Paths
    nav2_params_file = os.path.join(pkg_husky_minimal, 'config', 'nav2_params.yaml')
    workspace_dir = os.path.join(os.path.dirname(pkg_husky_minimal), '..', '..', '..')
    map_file = os.path.join(workspace_dir, 'src', 'husky_minimal', 'map', 'map.yaml')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_file,
        description='Full path to the ROS2 parameters file to use')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=map_file,
        description='Full path to map yaml file to load')
    
    # Create our own configured parameters with map file
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={'yaml_filename': map_yaml_file},
        convert_types=True)
    
    # Map Server Node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params,
                   {'use_sim_time': use_sim_time}])
    
    # AMCL Node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[configured_params,
                   {'use_sim_time': use_sim_time}])
    
    # Controller Server Node
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params,
                   {'use_sim_time': use_sim_time}])
    
    # Planner Server Node
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params,
                   {'use_sim_time': use_sim_time}])
    
    # Behavior Server Node
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_params,
                   {'use_sim_time': use_sim_time}])
    
    # BT Navigator Node
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params,
                   {'use_sim_time': use_sim_time}])
    
    # Waypoint Follower Node
    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_params,
                   {'use_sim_time': use_sim_time}])
    
    # Velocity Smoother Node
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[configured_params,
                   {'use_sim_time': use_sim_time}])
    
    # Smoother Server Node
    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[configured_params,
                   {'use_sim_time': use_sim_time}])
    
    # Collision Monitor Node
    collision_monitor_node = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[configured_params,
                   {'use_sim_time': use_sim_time}])
    
    # Lifecycle Manager for Localization - DELAYED
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                   {'autostart': autostart},
                   {'node_names': ['map_server', 'amcl']}])
    
    # Lifecycle Manager for Navigation - DELAYED
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                   {'autostart': autostart},
                   {'node_names': ['controller_server',
                                  'smoother_server',
                                  'planner_server',
                                  'behavior_server',
                                  'bt_navigator',
                                  'waypoint_follower',
                                  'velocity_smoother',
                                  'collision_monitor']}])
    
    # Delay lifecycle managers to avoid race condition
    delayed_lifecycle_localization = TimerAction(
        period=3.0,
        actions=[lifecycle_manager_localization]
    )
    
    delayed_lifecycle_navigation = TimerAction(
        period=3.0,
        actions=[lifecycle_manager_navigation]
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_yaml_cmd)
    
    # Add all nodes
    ld.add_action(map_server_node)
    ld.add_action(amcl_node)
    ld.add_action(controller_server_node)
    ld.add_action(planner_server_node)
    ld.add_action(behavior_server_node)
    ld.add_action(bt_navigator_node)
    ld.add_action(waypoint_follower_node)
    ld.add_action(velocity_smoother_node)
    ld.add_action(smoother_server_node)
    ld.add_action(collision_monitor_node)
    
    # Add DELAYED lifecycle managers
    ld.add_action(delayed_lifecycle_localization)
    ld.add_action(delayed_lifecycle_navigation)
    
    return ld