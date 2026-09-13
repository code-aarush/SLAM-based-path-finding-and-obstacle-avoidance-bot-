# SLAM Bring-up — Tugbot Warehouse

Milestone 1 of the autonomous navigation pipeline.  
Replaces the SDF-geometry-based `world_analyzer` approach with a proper sensor-driven SLAM stack.

---

## Architecture

```
Gazebo Harmonic (tugbot_warehouse.sdf)
    │
    │  Sensor: /model/tugbot/scan_front  (gz.msgs.LaserScan, 10 Hz)
    │  Odom:   /model/tugbot/odometry   (gz.msgs.Odometry,   20 Hz)
    │  TF:     /model/tugbot/tf         (gz.msgs.Pose_V → odom→base_link)
    │  Clock:  /clock
    │
    ▼
ros_gz_bridge (parameter_bridge)
    │
    │  /scan        sensor_msgs/msg/LaserScan
    │  /odom        nav_msgs/msg/Odometry
    │  /tf          tf2_msgs/msg/TFMessage   (odom → base_link)
    │  /clock       rosgraph_msgs/msg/Clock
    │
    ▼
slam_toolbox (async_slam_toolbox_node)
    │
    │  subscribes: /scan, /tf (odom→base_link)
    │  publishes:  /map (nav_msgs/msg/OccupancyGrid)
    │              /map_updates (map_msgs/msg/OccupancyGridUpdate)
    │              /tf  map → odom  (completes the TF chain)
    │
    ▼
RViz2
    fixed frame: map
    displays: Map, LaserScan, TF, Odometry
```

---

## Required Packages

All packages are already installed on this system:

| Package | Purpose |
|---------|---------|
| `ros_gz_bridge` | Gazebo ↔ ROS 2 topic bridge |
| `ros_gz_sim` | Gazebo simulation launch support |
| `slam_toolbox` | Online SLAM (map → odom TF + /map) |
| `rviz2` | Visualization |
| `tf2_ros` | TF frame management |
| `sensor_msgs` | LaserScan message type |
| `nav_msgs` | Odometry, OccupancyGrid |

Install if missing:
```bash
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-ros-gz ros-jazzy-rviz2
```

---

## Sensor Configuration (from model SDF)

Sensor: **scan_front** (selected for SLAM)
- Type: `gpu_lidar` (2D planar)
- Link: `scan_front`, pose: `0.221 0 0.1404 0 0 0` rel. to `base_link`
- Frequency: 10 Hz
- Rays: 674 samples, horizontal only
- FOV: ±84° (−1.47 to +1.47 rad)
- Range: 0.01 – 5.0 m

Other lidars (NOT used for SLAM):
- `scan_back`: 5 Hz, ±84°, 480 rays — bridged as `/scan_back` (disabled in RViz)
- `scan_omni`: VLP-16 3D lidar, 16 channels — 3D PointCloud, not bridged

Odometry: **DiffDrive plugin**
- Wheel separation: 0.5605 m, wheel radius: 0.195 m
- Publishes at 20 Hz
- `frame_id: odom`, `child_frame_id: base_link`

---

## TF Tree

```
map
 └─ odom          (published by slam_toolbox)
     └─ base_link (published by DiffDrive plugin via /model/tugbot/tf bridge)
         ├─ scan_front   (fixed joint, pose: 0.221 0 0.1404)
         ├─ scan_back    (fixed joint, pose: -0.2075 0 0.205)
         ├─ scan_omni    (fixed joint, pose: -0.1855 0 0.5318)
         ├─ camera_front (fixed joint)
         ├─ camera_back  (fixed joint)
         └─ imu_link     (fixed joint)
```

> **Note:** The DiffDrive plugin in the Tugbot SDF only publishes the `odom → base_link` transform.  
> Sensor link transforms (`base_link → scan_front`, etc.) come from the Gazebo pose publisher plugin  
> (`/model/tugbot/tf` also includes link poses). If these are missing from RViz, see the TF verification  
> section below.

---

## Launch Command

### Option A — Shell script (no colcon build required)

```bash
cd /home/aarush-sivaraman/Robotics/workspaces/robot_navigation
source /opt/ros/jazzy/setup.bash

# With tmux (recommended):
./slam_bringup.sh

# Without tmux (prints commands for manual terminals):
./slam_bringup.sh --no-tmux
```

### Option B — Manual (four separate terminals)

**Terminal 1 — Gazebo:**
```bash
gz sim /home/aarush-sivaraman/Robotics/workspaces/Worlds_models/tugbot_warehouse.sdf
```
Press **▶ Play** in the Gazebo GUI.

**Terminal 2 — Bridge (after Gazebo is running):**
```bash
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  --ros-args \
  -p config_file:=/home/aarush-sivaraman/Robotics/workspaces/robot_navigation/src/robot_navigation/simulation/bridge/tugbot_bridge.yaml \
  -p use_sim_time:=true
```

**Terminal 3 — SLAM Toolbox:**
```bash
source /opt/ros/jazzy/setup.bash
ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args \
  --params-file /home/aarush-sivaraman/Robotics/workspaces/robot_navigation/src/robot_navigation/slam/config/slam_toolbox_online_async.yaml \
  -p use_sim_time:=true
```

**Terminal 4 — RViz2:**
```bash
source /opt/ros/jazzy/setup.bash
ros2 run rviz2 rviz2 \
  -d /home/aarush-sivaraman/Robotics/workspaces/robot_navigation/src/robot_navigation/slam/rviz/slam.rviz \
  --ros-args -p use_sim_time:=true
```

**Terminal 5 — Teleoperation (drive the robot to build the map):**
```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/cmd_vel
```

---

## Expected Topics After Launch

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
```

Should include:

| Topic | Type | Source |
|-------|------|--------|
| `/scan` | `sensor_msgs/msg/LaserScan` | bridge ← Gazebo |
| `/scan_back` | `sensor_msgs/msg/LaserScan` | bridge ← Gazebo |
| `/odom` | `nav_msgs/msg/Odometry` | bridge ← Gazebo |
| `/tf` | `tf2_msgs/msg/TFMessage` | bridge + slam_toolbox |
| `/clock` | `rosgraph_msgs/msg/Clock` | bridge ← Gazebo |
| `/map` | `nav_msgs/msg/OccupancyGrid` | slam_toolbox |
| `/map_updates` | `map_msgs/msg/OccupancyGridUpdate` | slam_toolbox |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | teleop → bridge → Gazebo |

---

## Verification Commands

### 1. Verify `/scan` is publishing

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic hz /scan
# Expected: ~10.0 Hz

ros2 topic echo /scan --once
# Should show: header.frame_id="scan_front", ranges=<674 floats>
```

### 2. Verify TF chain

```bash
source /opt/ros/jazzy/setup.bash
ros2 run tf2_tools view_frames
# Generates frames.pdf — open to see the TF tree

# Or check specific transform:
ros2 run tf2_ros tf2_echo odom base_link
# Should show continuously updating translation/rotation

ros2 run tf2_ros tf2_echo map odom
# Should appear once slam_toolbox starts (map→odom published by SLAM)
```

### 3. Verify `/map` is publishing

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic hz /map
# Expected: ~0.2 Hz (every 3–5 seconds, maps update slowly)

ros2 topic echo /map --once | head -20
# Should show: info.resolution=0.05, data=[...occupancy values...]
```

### 4. Verify bridge is running

```bash
source /opt/ros/jazzy/setup.bash
ros2 node list | grep bridge
# Expected: /tugbot_gz_bridge

ros2 node info /tugbot_gz_bridge
# Should list subscriptions and publishers
```

### 5. Verify slam_toolbox status

```bash
source /opt/ros/jazzy/setup.bash
ros2 node list | grep slam
# Expected: /slam_toolbox

# Check if any TF errors:
ros2 topic echo /rosout 2>/dev/null | grep -i "slam\|tf\|transform"
```

---

## Saving the Map

Once you have built a satisfactory map, save it with:

```bash
source /opt/ros/jazzy/setup.bash

# Method 1: map_saver_cli (nav2_map_server)
ros2 run nav2_map_server map_saver_cli \
  -f /home/aarush-sivaraman/Robotics/workspaces/robot_navigation/outputs/warehouse_map

# This creates:
#   outputs/warehouse_map.pgm   (grayscale occupancy image)
#   outputs/warehouse_map.yaml  (map metadata)

# Method 2: slam_toolbox serialize (preserves pose graph for continued mapping)
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/aarush-sivaraman/Robotics/workspaces/robot_navigation/outputs/warehouse_slam'}"
```

---

## Teleoperation

The Tugbot Teleop GUI plugin is in Gazebo (uses `/model/tugbot/cmd_vel` directly).  
For keyboard teleop via ROS:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/cmd_vel
```

Keys: `i`=forward, `,`=backward, `j`=left, `l`=right, `k`=stop

---

## Known Limitations

1. **scan_front max range is 5 m** — the warehouse (~35 m × 50 m) requires the robot to explore extensively. The map will only fill in visited areas.

2. **No sensor TFs for non-base links** — The Tugbot DiffDrive only publishes `odom→base_link`. The PosePublisher plugin publishes all link poses on `/model/tugbot/tf`. The bridge config routes this to `/tf`, so all link TFs should propagate. If `scan_front` does not appear in RViz TF, verify the bridge is running.

3. **World starts paused** — Remember to press ▶ in Gazebo. The `/clock` will not advance until the simulation is running, and slam_toolbox will appear stuck.

4. **scan_omni (VLP-16) not used** — The 3D lidar publishes `PointCloud2`, not `LaserScan`. Feeding it to slam_toolbox would require a `pointcloud_to_laserscan` conversion node. This is a future enhancement.

5. **No loop closure guarantee** — With 5 m range, the warehouse is too large for reliable loop closure in a single pass. Drive slowly and overlap previously scanned areas.

6. **Legacy world_analyzer is UNTOUCHED** — The `world_analyzer`, `geometry_processor`, and related modules remain intact. They are no longer in the mapping hot path but can still be used for static collision analysis.

---

## Files in This Module

```
simulation/
  bridge/
    tugbot_bridge.yaml          ← ros_gz_bridge topic mappings (source of truth)
  launch/
    gazebo_launch.py            ← launch Gazebo only
    bridge_launch.py            ← launch bridge only
    slam_bringup.py             ← combined Gazebo + bridge + SLAM + RViz

slam/
  config/
    slam_toolbox_online_async.yaml  ← slam_toolbox parameters
  launch/
    slam_launch.py              ← launch slam_toolbox only
  rviz/
    slam.rviz                   ← RViz2 configuration

slam_bringup.sh                 ← shell script (no colcon build required)
slam/README.md                  ← this file
```

---

## Legacy Module Status

| Module | Status | Reason |
|--------|--------|--------|
| `world_analyzer/` | ✅ Retained | Still functional; not in SLAM hot path |
| `geometry_processor/` | ✅ Retained | Stub; no SLAM dependency |
| `outputs/world_analysis.json` | ✅ Retained | Static analysis artifact |
| `world_analyzer tests` | ✅ Retained | Still pass; not blocking |

**Modules that are NOW OBSOLETE** (once SLAM is verified):
- `world_analyzer` — SDF geometry was used as a map source; SLAM replaces this entirely
- `geometry_processor` — was planned as downstream of world_analyzer
- `occupancy_grid` (the Python stub) — replaced by slam_toolbox's native OccupancyGrid
- `planners` (the Python stub) — was planned over the geometry-derived grid, not sensor SLAM

None are deleted yet. Report deletion in the next milestone after SLAM is verified working.
