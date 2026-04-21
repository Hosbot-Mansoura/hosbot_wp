#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch = LaunchDescription()
    hardware_package_path = get_package_share_directory('hardware_pkg')
    params_file = os.path.join(hardware_package_path,'config','nav_params.yaml')

    use_sim_time = False
    map_yaml_file = LaunchConfiguration('map')
    run_rviz = LaunchConfiguration('run_rviz')


    
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    default_rviz_config = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    lifecycle_nodes_localization = ['map_server' , 'amcl']
    lifecycle_nodes_navigation = [
        'controller_server',
        'planner_server',
        'smoother_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower'
    ]

    map_saver_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[ params_file,
            {
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml_file
            }
        ]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file]
    )

    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file]
    )

    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file]
    )

    smoother_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file]
    )

    behavior_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file]
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file]
    )

    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file]
    )

    lifecycle_localization_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': lifecycle_nodes_localization
        }]
    )

    lifecycle_navigation_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': lifecycle_nodes_navigation
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(run_rviz)
    )
    launch.add_action(DeclareLaunchArgument('run_rviz',default_value='true'))
    launch.add_action(DeclareLaunchArgument('map',default_value='/home/hosbot/hosbot_wp/maps/res_map.yaml'))
    launch.add_action(map_saver_node)
    launch.add_action(amcl_node)
    launch.add_action(controller_node)
    launch.add_action(planner_node)
    launch.add_action(smoother_node)
    launch.add_action(behavior_node)
    launch.add_action(bt_navigator_node)
    launch.add_action(waypoint_follower_node)
    launch.add_action(lifecycle_localization_node)
    launch.add_action(lifecycle_navigation_node)
    launch.add_action(rviz_node)
    return launch