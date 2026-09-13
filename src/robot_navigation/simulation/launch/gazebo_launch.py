#!/usr/bin/env python3
"""
Launch Gazebo Harmonic with the Tugbot warehouse world.

World file: Worlds_models/tugbot_warehouse.sdf
  - Loads Warehouse (OpenRobotics/Warehouse from Fuel)
  - Loads Tugbot (MovAi/Tugbot from Fuel) at pose (13.9, -10.6, 0.1)
  - Starts paused (press Play in Gazebo GUI)

Standalone usage:
  python3 -m robot_navigation.simulation.launch.gazebo_launch
Or via ros2 launch (requires a ROS 2 package setup):
  ros2 launch robot_navigation gazebo_launch.py
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ── Resolve world file path ────────────────────────────────────────────
    # The workspace root is two levels above this file:
    #   src/robot_navigation/simulation/launch/  →  workspace root
    this_file = Path(__file__).resolve()
    workspace_root = this_file.parents[5]  # robot_navigation/ package root
    # Actually navigate from known structure:
    # parents[0] = launch/
    # parents[1] = simulation/
    # parents[2] = robot_navigation/  (Python package inside src/)
    # parents[3] = src/
    # parents[4] = robot_navigation/  (repository root)
    # parents[5] = workspaces/        (contains Worlds_models/ at this level)
    workspaces_root = this_file.parents[5]
    world_file = workspaces_root / "Worlds_models" / "tugbot_warehouse.sdf"

    if not world_file.exists():
        raise FileNotFoundError(
            f"World file not found: {world_file}\n"
            "Expected layout: <repo>/Worlds_models/tugbot_warehouse.sdf"
        )

    # ── Launch Arguments ──────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=str(world_file),
        description="Path to Gazebo SDF world file",
    )
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run Gazebo without GUI (server-only)",
    )

    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")

    # ── Gazebo Harmonic command ───────────────────────────────────────────
    # In Gazebo Harmonic the CLI is 'gz sim'. The --headless-rendering flag
    # suppresses the GUI window when headless=true.
    gz_sim = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            world,
            # --verbose gives useful startup diagnostics
            # Remove -v 4 if too noisy
            # '-v', '4',
        ],
        output="screen",
        additional_env={"GZ_SIM_RESOURCE_PATH": os.path.expanduser("~/.gz/fuel")},
    )

    return LaunchDescription(
        [
            world_arg,
            headless_arg,
            gz_sim,
        ]
    )
