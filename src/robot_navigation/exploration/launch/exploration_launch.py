#!/usr/bin/env python3
"""
Launch Frontier Explorer
"""

from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    this_file = Path(__file__).resolve()
    exploration_dir = this_file.parents[1]
    frontier_explorer_script = exploration_dir / "frontier_explorer.py"

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time",
    )

    # We use ExecuteProcess to run the python script directly since it's not installed via setup.py
    explorer_node = ExecuteProcess(
        cmd=["python3", str(frontier_explorer_script), "--ros-args", "-p", "use_sim_time:=true"],
        output="screen",
    )

    return LaunchDescription([
        use_sim_time_arg,
        explorer_node,
    ])
