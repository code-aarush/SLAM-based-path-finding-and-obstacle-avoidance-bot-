#!/usr/bin/env python3
"""
Launch slam_toolbox in online-asynchronous mode for the Tugbot.

RUNTIME-VERIFIED TF chain:
  map → odom → base_link → scan_front

  odom → base_link:
    Published by Tugbot DiffDrive plugin, bridged via /model/tugbot/tf → /tf.
    frame_id=odom, child_frame_id=base_link (verified from gz topic echo).

  base_link → scan_front:
    Static transform from fixed joint in model.sdf:
      <link name="scan_front"><pose>0.221 0 0.1404 0 0 0</pose>
    Published by static_transform_publisher included in this launch.
    This is required because:
      - The DiffDrive plugin only publishes odom→base_link
      - The Gazebo PosePublisher uses scoped names (tugbot::scan_front::...)
        which are not compatible with ROS TF frame names
      - The bridge frame_id override sets LaserScan frame_id=scan_front
      - slam_toolbox must resolve scan_front → base_link to transform scans

  map → odom:
    Published by slam_toolbox itself (SLAM output).

Standalone usage (requires sourced ROS 2 + Gazebo running + bridge running):
  ros2 run slam_toolbox async_slam_toolbox_node \\
    --ros-args \\
    --params-file <path>/slam_toolbox_online_async.yaml \\
    -p use_sim_time:=true

  AND separately:
  ros2 run tf2_ros static_transform_publisher \\
    --x 0.221 --y 0 --z 0.1404 --roll 0 --pitch 0 --yaw 0 \\
    --frame-id base_link --child-frame-id scan_front

Optional launch arguments:
  use_sim_time:=true   (default: true)
  slam_config:=<path>  (override config file)
"""

import os
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Resolve config path ────────────────────────────────────────────────
    this_file = Path(__file__).resolve()
    # parents[0]=launch/, parents[1]=slam/
    slam_dir = this_file.parents[1]
    default_config = slam_dir / "config" / "slam_toolbox_online_async.yaml"

    if not default_config.exists():
        raise FileNotFoundError(f"SLAM config not found: {default_config}")

    # ── Launch Arguments ──────────────────────────────────────────────────
    sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use Gazebo simulation time",
    )
    config_arg = DeclareLaunchArgument(
        "slam_config",
        default_value=str(default_config),
        description="Path to slam_toolbox YAML parameter file",
    )

    # ── Static TF: base_link → scan_front ────────────────────────────────
    # Runtime-verified necessity:
    #   - gz topic -e /model/tugbot/tf shows ONLY odom→base_link
    #   - LaserScan frame_id is overridden to 'scan_front' by bridge YAML
    #   - slam_toolbox requires scan_front to be in TF tree
    # Source: model.sdf fixed joint <link name="scan_front"><pose>0.221 0 0.1404 0 0 0</pose>
    static_tf_scan_front = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_link_scan_front",
        arguments=[
            "--x", "0.221",
            "--y", "0.0",
            "--z", "0.1404",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "base_link",
            "--child-frame-id", "scan_front",
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "slam_params_file": LaunchConfiguration("slam_config"),
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }.items()
    )

    return LaunchDescription(
        [
            sim_time_arg,
            config_arg,
            static_tf_scan_front,
            slam_node,
        ]
    )
