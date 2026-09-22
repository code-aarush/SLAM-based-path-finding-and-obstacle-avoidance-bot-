import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    pkg_dir = Path(__file__).parents[2]
    default_params_file = pkg_dir / "navigation" / "config" / "nav2_params.yaml"

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=str(default_params_file),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # NOTE: In Nav2 Jazzy, controller_server and planner_server internally instantiate
    # local_costmap and global_costmap respectively. Launching standalone nav2_costmap_2d
    # nodes with the same namespace/name creates duplicate node names and lifecycle state
    # conflicts. The costmaps are activated as part of their owner server's lifecycle.
    # A single lifecycle_manager_navigation managing the four core servers is sufficient.

    lifecycle_manager_navigation_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator'],
            'bond_timeout': 4.0
        }]
    )

    # Delay lifecycle manager slightly so all Nav2 servers have started first
    delayed_lifecycle = TimerAction(
        period=2.0,
        actions=[lifecycle_manager_navigation_node]
    )

    ld = LaunchDescription()
    ld.add_action(SetParameter('use_sim_time', use_sim_time))
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(controller_server_node)
    ld.add_action(planner_server_node)
    ld.add_action(behavior_server_node)
    ld.add_action(bt_navigator_node)
    ld.add_action(delayed_lifecycle)

    return ld
