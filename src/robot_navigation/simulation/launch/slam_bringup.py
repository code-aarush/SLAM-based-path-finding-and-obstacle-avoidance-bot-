#!/usr/bin/env python3
"""
Combined SLAM bring-up launch for the Tugbot warehouse.

Starts in order:
  1. Gazebo Harmonic (gz sim) with tugbot_warehouse.sdf
  2. ros_gz_bridge (parameter_bridge with YAML config)
  3. slam_toolbox (async_slam_toolbox_node)
  4. RViz2 (optional, controlled by launch_rviz argument)

All components can also be launched individually for debugging:
  ros2 launch robot_navigation gazebo_launch.py
  ros2 launch robot_navigation bridge_launch.py
  ros2 launch robot_navigation slam_launch.py

Usage:
  # Full stack (recommended first run)
  ros2 launch robot_navigation slam_bringup.py

  # Without RViz (headless / SSH)
  ros2 launch robot_navigation slam_bringup.py launch_rviz:=false

  # With custom world
  ros2 launch robot_navigation slam_bringup.py world:=/path/to/world.sdf
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Resolve paths ──────────────────────────────────────────────────────
    this_file = Path(__file__).resolve()
    # Actual path layout (verified):
    # parents[0] = launch/
    # parents[1] = simulation/
    # parents[2] = robot_navigation/  (Python package inside src/)
    # parents[3] = src/
    # parents[4] = robot_navigation/  (repository root)
    # parents[5] = workspaces/        (contains Worlds_models/)
    workspaces_root = this_file.parents[5]  # .../workspaces/
    repo_root = this_file.parents[4]        # .../robot_navigation/ (repo)

    simulation_dir = this_file.parents[1]           # simulation/
    robot_nav_pkg = this_file.parents[2]            # robot_navigation/ (Python pkg)
    slam_dir = robot_nav_pkg / "slam"

    world_file = workspaces_root / "Worlds_models" / "tugbot_warehouse.sdf"
    bridge_config = simulation_dir / "bridge" / "tugbot_bridge.yaml"
    slam_config = slam_dir / "config" / "slam_toolbox_online_async.yaml"
    rviz_config = slam_dir / "rviz" / "slam.rviz"

    for p in [world_file, bridge_config, slam_config, rviz_config]:
        if not p.exists():
            raise FileNotFoundError(f"Required file missing: {p}")

    # ── Launch Arguments ──────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=str(world_file),
        description="Path to Gazebo SDF world file",
    )
    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="true",
        description="Launch RViz2 for visualization",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="All nodes use Gazebo simulation time",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    # ── 1. Gazebo ──────────────────────────────────────────────────────────
    gz_sim = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            LaunchConfiguration("world"),
            # Uncomment for more verbose sensor/plugin output:
            # '-v', '4',
        ],
        output="screen",
        additional_env={"GZ_SIM_RESOURCE_PATH": os.path.expanduser("~/.gz/fuel")},
    )

    # ── 2. Bridge (delayed 3 s to let Gazebo finish loading) ──────────────
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="tugbot_gz_bridge",
        output="screen",
        parameters=[
            {
                "config_file": str(bridge_config),
                "use_sim_time": use_sim_time,
            }
        ],
    )
    bridge_delayed = TimerAction(period=3.0, actions=[bridge_node])

    # ── 3. slam_toolbox (delayed 6 s to let bridge establish topics) ──────
    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            str(slam_config),
            {"use_sim_time": use_sim_time},
        ],
    )
    slam_delayed = TimerAction(period=6.0, actions=[slam_node])

    # ── 4. RViz2 (delayed 7 s) ────────────────────────────────────────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", str(rviz_config)],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )
    rviz_delayed = TimerAction(period=7.0, actions=[rviz_node])

    return LaunchDescription(
        [
            world_arg,
            launch_rviz_arg,
            use_sim_time_arg,
            gz_sim,
            bridge_delayed,
            slam_delayed,
            rviz_delayed,
        ]
    )
