#!/usr/bin/env python3
"""
Launch the ros_gz_bridge for the Tugbot warehouse simulation.

Bridges Gazebo Harmonic topics to ROS 2 using the YAML config at:
  simulation/bridge/tugbot_bridge.yaml

Topics bridged (Gazebo → ROS 2):
  /model/tugbot/scan_front  → /scan        (sensor_msgs/msg/LaserScan)
  /model/tugbot/scan_back   → /scan_back   (sensor_msgs/msg/LaserScan)
  /model/tugbot/odometry    → /odom        (nav_msgs/msg/Odometry)
  /model/tugbot/tf          → /tf          (tf2_msgs/msg/TFMessage)
  /clock                    → /clock       (rosgraph_msgs/msg/Clock)

Topics bridged (ROS 2 → Gazebo):
  /cmd_vel → /model/tugbot/cmd_vel         (geometry_msgs/msg/Twist)

Standalone usage (requires sourced ROS 2):
  ros2 launch robot_navigation bridge_launch.py
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Resolve bridge config path ─────────────────────────────────────────
    this_file = Path(__file__).resolve()
    # parents[0]=launch/, parents[1]=simulation/, parents[2]=robot_navigation(pkg)
    simulation_dir = this_file.parents[1]
    bridge_config = simulation_dir / "bridge" / "tugbot_bridge.yaml"

    if not bridge_config.exists():
        raise FileNotFoundError(f"Bridge config not found: {bridge_config}")

    # ── Launch Arguments ──────────────────────────────────────────────────
    config_arg = DeclareLaunchArgument(
        "bridge_config",
        default_value=str(bridge_config),
        description="Path to ros_gz_bridge YAML configuration file",
    )

    # ── Bridge Node ────────────────────────────────────────────────────────
    # ros_gz_bridge parameter_bridge reads the YAML and creates one bridge
    # node for each entry.  --ros-args allows ROS node remapping if needed.
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="tugbot_gz_bridge",
        output="screen",
        parameters=[
            {
                "config_file": LaunchConfiguration("bridge_config"),
                # Use sim time so slam_toolbox and RViz stay in sync with Gazebo
                "use_sim_time": True,
            }
        ],
        # Remap the ROS /clock publisher so downstream nodes receive it
        remappings=[],
    )

    return LaunchDescription(
        [
            config_arg,
            bridge_node,
        ]
    )
