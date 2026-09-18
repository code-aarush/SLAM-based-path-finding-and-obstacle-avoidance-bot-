# Stage 1 — Navigation Foundation & Costmap Infrastructure
## Implementation Plan

> **Status:** Source implemented. Build verified (`colcon build` passes). Runtime/integration verification pending.
> **Prerequisite:** Stage 0 (SLAM + Bridge + Frontier Detection) verified complete.
> **Author:** Architecture review, September 2026
> **Updated:** Repository baseline stabilization, September 2026

---

## 0. Confirmed Baseline (No Re-inspection Required)

From the prior audit:

- Package: `ament_python`, `robot_navigation`, `colcon list` discovers it correctly
- TF chain: `map → odom → base_link → scan_front` (verified)
- `/scan` at 10 Hz, frame `scan_front`, 674 rays, range 0.01–5.0 m
- `/map` published by `slam_toolbox` at 5 cm/cell resolution with `Transient Local` QoS
- Tugbot: wheel_separation=0.5605 m, body width ≈ 0.57 m
- Nav2 fully installed system-wide (`ros-jazzy-navigation2`)
- `src/robot_navigation/navigation/` exists with empty `__init__.py` only
- `scripts/`, `config/`, `simulation/config/` are all empty directories
- Missing declared deps: `scipy`, `numpy` (used in `frontier_explorer.py` but absent from `package.xml`)

---

## 1. Nav2 Component Selection

| Component | Stage 1 | Reason |
|---|---|---|
| `lifecycle_manager` | ✅ YES | Manages all Nav2 node lifecycles |
| `global_costmap` (nav2_costmap_2d) | ✅ YES | Core deliverable |
| `local_costmap` (nav2_costmap_2d) | ✅ YES | Core deliverable |
| `planner_server` + NavFn plugin | ✅ YES (placeholder) | Needed to complete NavigateToPose pipeline; plugin string is the only thing that changes when A* replaces it |
| `controller_server` + RegulatedPurePursuit | ✅ YES (placeholder) | Needed for full BT navigator; trivially swappable |
| `bt_navigator` | ✅ YES | Provides `navigate_to_pose` action that Stage 2 frontier nav will call |
| `behavior_server` (Spin, BackUp, Wait) | ✅ YES (minimal) | Required by default BT; keeps recovery available |
| `AMCL` | ❌ NO | `slam_toolbox` already publishes `map→odom`; AMCL would conflict |
| `map_server` | ❌ NO | `slam_toolbox` is the live map source; static map_server would override it |
| `velocity_smoother` | ❌ NO | Deferred |
| `collision_monitor` | ❌ NO | Deferred to dynamic obstacle stage |
| `waypoint_follower` | ❌ NO | Deferred |

---

## 2. Robot Footprint Decision

Tugbot geometry from `~/.gz/fuel/.../tugbot/1/model.sdf`:

- Wheel separation: 0.5605 m → half-width ≈ 0.28 m
- Body mesh collision: ~0.7 m long, ~0.57 m wide
- Gripper extends ~0.46 m to the rear

**Decision: `robot_radius: 0.40` m** (circular, conservative).  
Circumscribes the widest body cross-section with margin. Appears in exactly one place per
costmap block in `nav2_params.yaml` — trivially replaced by a polygon in Stage 2 without
structural change. The same value must be used by the future custom A* planner for
clearance validation.

---

## 3. Costmap Architecture

### Global Costmap

| Parameter | Value | Reason |
|---|---|---|
| `global_frame` | `map` | SLAM output frame |
| `robot_base_frame` | `base_link` | Verified TF root |
| `robot_radius` | `0.40` | Conservative circular footprint |
| `resolution` | `0.05` | Must match slam_toolbox grid resolution exactly |
| `track_unknown_space` | `true` | Warehouse has large unexplored regions |
| `update_frequency` | `1.0` Hz | Map updates slowly from SLAM |
| `publish_frequency` | `1.0` Hz | |
| `transform_tolerance` | `0.5` s | Generous — sim time can lag |
| `always_send_full_costmap` | `true` | Required for RViz display |
| **Plugins** | `static_layer`, `obstacle_layer`, `inflation_layer` | Standard pipeline |

**static_layer:** Subscribes `/map`. `map_subscribe_transient_local: true` is **required** —
slam_toolbox publishes with Transient Local durability; without this flag the costmap
never receives the map.

**obstacle_layer:** `nav2_costmap_2d::ObstacleLayer`. Source: `/scan`, type `LaserScan`.
`obstacle_max_range: 4.5` m (sensor max is 5.0 m; leave 0.5 m margin for noise),
`raytrace_max_range: 4.8` m, marking and clearing both enabled.

**inflation_layer:** `inflation_radius: 0.55` m (robot radius + 0.15 m safety buffer),
`cost_scaling_factor: 3.0`.

### Local Costmap

| Parameter | Value | Reason |
|---|---|---|
| `global_frame` | `odom` | Standard for reactive local costmap |
| `robot_base_frame` | `base_link` | |
| `robot_radius` | `0.40` | Same as global |
| `rolling_window` | `true` | Follows the robot |
| `width` / `height` | `4.0` m | Larger than default (3×3) — scan max is 5.0 m |
| `resolution` | `0.05` | |
| `update_frequency` | `5.0` Hz | |
| `publish_frequency` | `2.0` Hz | |
| `transform_tolerance` | `0.5` s | |
| `always_send_full_costmap` | `true` | |
| **Plugins** | `obstacle_layer`, `inflation_layer` | No static layer in local costmap |

**obstacle_layer:** Same `/scan` config as global. `obstacle_max_range: 4.5` m,
`raytrace_max_range: 4.8` m.

**inflation_layer:** `inflation_radius: 0.55` m, `cost_scaling_factor: 3.0`.

### Data Flow

```
/map (slam_toolbox, Transient Local)  →  global_costmap StaticLayer
/scan (ros_gz_bridge, 10 Hz)          →  global_costmap ObstacleLayer
                                       →  local_costmap  ObstacleLayer
TF (map→odom→base_link)               →  both costmaps (robot pose lookup)

global_costmap publishes:
  /global_costmap/costmap             (nav_msgs/OccupancyGrid)
  /global_costmap/costmap_raw
  /global_costmap/published_footprint

local_costmap publishes:
  /local_costmap/costmap              (nav_msgs/OccupancyGrid)
  /local_costmap/costmap_raw
  /local_costmap/published_footprint
```

---

## 4. File-by-File Change Plan

### 4.1 MODIFY — `package.xml`

**Path:** `package.xml`  
**Purpose:** Declare Nav2 and previously missing Python runtime dependencies.  
**Change — add inside `<package>`:**

```xml
<!-- Navigation infrastructure -->
<depend>nav2_costmap_2d</depend>
<depend>nav2_lifecycle_manager</depend>
<depend>nav2_planner</depend>
<depend>nav2_controller</depend>
<depend>nav2_bt_navigator</depend>
<depend>nav2_behaviors</depend>
<depend>nav2_msgs</depend>
<depend>nav2_util</depend>

<!-- Previously missing Python deps (frontier_explorer.py uses both) -->
<depend>python3-numpy</depend>
<depend>python3-scipy</depend>
```

**Interfaces:** None  
**Verification:** `xmllint --noout package.xml && echo OK`  
**Expected:** `OK`  
**Failure:** XML parse error → mismatched or misspelled tags

---

### 4.2 MODIFY — `setup.py`

**Path:** `setup.py`  
**Purpose:** Declare Python runtime deps at pip level; confirm new subdirectory installs.  
**Change — update `install_requires`:**

```python
install_requires=['setuptools', 'numpy', 'scipy'],
```

The existing `generate_data_files()` walker already recursively installs `.py`, `.yaml`,
`.rviz` from all subdirs under `src/robot_navigation/`. The new
`navigation/config/`, `navigation/launch/`, `navigation/rviz/` directories are picked up
automatically — no walker change required.

**Verification:** `python3 -c "import ast; ast.parse(open('setup.py').read()); print('OK')"`  
**Expected:** `OK`  
**Failure:** `SyntaxError` → check indentation around `install_requires`

---

### 4.3 CREATE — `src/robot_navigation/navigation/config/nav2_params.yaml`

**Path:** `src/robot_navigation/navigation/config/nav2_params.yaml`  
**Purpose:** Single authoritative parameter file for all Nav2 lifecycle nodes in Stage 1.  
**Inputs consumed:** `/map`, `/scan`, `/tf`, `/odom`, `/clock`  
**Topics published:** `/global_costmap/costmap`, `/local_costmap/costmap`, `/plan`, `/cmd_vel`,
`/global_costmap/published_footprint`, `/local_costmap/published_footprint`  
**Actions provided:** `navigate_to_pose`, `navigate_through_poses`, `compute_path_to_pose`,
`follow_path`

#### bt_navigator parameters
- `global_frame: map`, `robot_base_frame: base_link`, `odom_topic: /odom`
- `bt_loop_duration: 10`, `default_server_timeout: 20`
- `navigators: ["navigate_to_pose", "navigate_through_poses"]`
- `navigate_to_pose.plugin: "nav2_bt_navigator::NavigateToPoseNavigator"`
- `navigate_through_poses.plugin: "nav2_bt_navigator::NavigateThroughPosesNavigator"`
- `error_code_names: [compute_path_error_code, follow_path_error_code]`

#### planner_server parameters
- `expected_planner_frequency: 20.0`, `costmap_update_timeout: 1.0`
- Single plugin named `GridBased` using `nav2_navfn_planner::NavfnPlanner`
- `tolerance: 0.5`, `use_astar: false`, `allow_unknown: true`

> The key `GridBased` is arbitrary — Stage 2 replaces only the `plugin:` string to swap
> in the custom A* planner, without renaming any keys or restructuring the file.

#### controller_server parameters
- `controller_frequency: 10.0` Hz (matches scan rate)
- `min_x_velocity_threshold: 0.001`, `min_theta_velocity_threshold: 0.001`
- `failure_tolerance: 0.3`
- Single controller `FollowPath` using
  `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`
- `desired_linear_vel: 0.3`, `max_linear_accel: 2.5`, `max_linear_decel: 2.5`
- `max_angular_accel: 3.2`, `transform_tolerance: 0.5`
- `lookahead_dist: 0.6`, `use_velocity_scaled_lookahead: false`
- Goal checker: `xy_goal_tolerance: 0.30`, `yaw_goal_tolerance: 0.35`

#### behavior_server parameters
- `local_frame: odom`, `global_frame: map`, `robot_base_frame: base_link`
- `transform_tolerance: 0.5`, `cycle_frequency: 10.0`
- `behavior_plugins: ["spin", "backup", "wait"]` with their standard plugin strings

#### global_costmap parameters
As specified in Section 3 above.

#### local_costmap parameters
As specified in Section 3 above.

**What must NOT go in this file:** AMCL parameters, map_server parameters, custom A*
plugin configuration, any hardcoded filesystem paths.

**Verification:**
```bash
python3 -c "
import yaml
with open('src/robot_navigation/navigation/config/nav2_params.yaml') as f:
    d = yaml.safe_load(f)
required = ['bt_navigator','planner_server','controller_server',
            'behavior_server','global_costmap','local_costmap']
for k in required:
    assert k in d, f'Missing top-level key: {k}'
print('YAML OK, all required keys present')
"
```
**Expected:** `YAML OK, all required keys present`  
**Failure:** `AssertionError` → missing section; `yaml.YAMLError` → indentation issue

---

### 4.4 CREATE — `src/robot_navigation/navigation/launch/navigation_launch.py`

**Path:** `src/robot_navigation/navigation/launch/navigation_launch.py`  
**Purpose:** Standalone ROS 2 launch file for all Nav2 lifecycle nodes. Usable independently
or included by `slam_bringup.py`.

**Launch arguments:**
- `use_sim_time` (default: `"true"`)
- `nav2_params_file` (default: resolved path to `../config/nav2_params.yaml`)

**Path resolution:** `this_file.parents[2] / "config" / "nav2_params.yaml"` — navigates
`launch/` → `navigation/` → `config/`.

**Nodes to start (all receive `params_file` and `use_sim_time`):**

| Node name | Package | Executable |
|---|---|---|
| `controller_server` | `nav2_controller` | `controller_server` |
| `planner_server` | `nav2_planner` | `planner_server` |
| `behavior_server` | `nav2_behaviors` | `behavior_server` |
| `bt_navigator` | `nav2_bt_navigator` | `bt_navigator` |
| `local_costmap` | `nav2_costmap_2d` | `nav2_costmap_2d_ros` |
| `global_costmap` | `nav2_costmap_2d` | `nav2_costmap_2d_ros` |
| `lifecycle_manager_costmaps` | `nav2_lifecycle_manager` | `lifecycle_manager` |
| `lifecycle_manager_navigation` | `nav2_lifecycle_manager` | `lifecycle_manager` |

**Lifecycle manager configuration:**
- `lifecycle_manager_costmaps` manages: `["local_costmap", "global_costmap"]`
- `lifecycle_manager_navigation` manages: `["controller_server", "planner_server", "behavior_server", "bt_navigator"]`
- Both managers: `autostart: true`, `bond_timeout: 4.0`
- Wrap both lifecycle managers in `TimerAction(period=2.0)` to allow costmap and
  navigation nodes to register before the managers attempt transitions.

**Costmap node namespacing:** The `local_costmap` node must be launched with
`namespace="local_costmap"` and `name="local_costmap"`. The `global_costmap` node
similarly with `namespace="global_costmap"`. This matches Nav2 convention and the
parameter file structure.

**Interfaces:**
- Subscribes (via config): `/map`, `/scan`, `/tf`, `/tf_static`, `/clock`, `/odom`
- Publishes: `/global_costmap/costmap`, `/local_costmap/costmap`, `/plan`, `/cmd_vel`
- Actions: `/navigate_to_pose`, `/navigate_through_poses`, `/compute_path_to_pose`, `/follow_path`

**What must NOT go in this file:** Gazebo launch logic, bridge launch, SLAM launch,
hardcoded absolute paths, AMCL, map_server.

**Verification:**
```bash
source /opt/ros/jazzy/setup.bash
python3 -c "
from launch.launch_description_sources import PythonLaunchDescriptionSource
src = PythonLaunchDescriptionSource(
    'src/robot_navigation/navigation/launch/navigation_launch.py')
print('Parse OK')
"
```
**Expected:** `Parse OK`  
**Failure:** `ImportError` → wrong launch API import; `AttributeError` → missing
`generate_launch_description`

---

### 4.5 MODIFY — `src/robot_navigation/simulation/launch/slam_bringup.py`

**Path:** `src/robot_navigation/simulation/launch/slam_bringup.py`  
**Purpose:** Add optional `launch_navigation` argument to start Nav2 alongside the
existing SLAM stack.

**Changes (additive only):**

1. Add `DeclareLaunchArgument("launch_navigation", default_value="false",
   description="Launch Nav2 navigation stack alongside SLAM")`

2. Resolve the path to `navigation_launch.py`:
   `nav_launch_file = robot_nav_pkg / "navigation" / "launch" / "navigation_launch.py"`

3. Add a `GroupAction` containing an `IncludeLaunchDescription` for
   `navigation_launch.py`, wrapped in `IfCondition(LaunchConfiguration("launch_navigation"))`.
   Wrap the entire GroupAction in `TimerAction(period=15.0)` — Nav2 needs SLAM to
   produce at least one `/map` message before costmaps can initialize.

4. Add the `GroupAction` to the `LaunchDescription` return list.

**What must NOT change:** Gazebo, bridge, SLAM, or RViz logic. All existing arguments
and their defaults remain unchanged. The file must still work correctly with no arguments.

**Verification:**
```bash
python3 -c "
import ast
with open('src/robot_navigation/simulation/launch/slam_bringup.py') as f:
    ast.parse(f.read())
print('Syntax OK')
"
```

---

### 4.6 CREATE — `src/robot_navigation/navigation/rviz/navigation.rviz`

**Path:** `src/robot_navigation/navigation/rviz/navigation.rviz`  
**Purpose:** RViz2 configuration for navigation infrastructure verification. Shows SLAM
map, both costmaps, laser scan, TF frames, and robot footprint.

**Fixed frame:** `map`

**Required display panels:**

| Display type | Topic | Notes |
|---|---|---|
| `Map` | `/map` | SLAM output; Durability: Transient Local |
| `Map` | `/global_costmap/costmap` | Alpha 0.6, color scheme: costmap |
| `Map` | `/local_costmap/costmap` | Alpha 0.5, color scheme: costmap |
| `LaserScan` | `/scan` | Red points, size 3px |
| `TF` | — | Show `map`, `odom`, `base_link` |
| `Polygon` | `/global_costmap/published_footprint` | Robot footprint overlay |
| `Polygon` | `/local_costmap/published_footprint` | |
| `Path` | `/plan` | Green line |

**What must NOT be in this file:** `/exploration/frontiers` MarkerArray, `/exploration/selected_target` — those remain in `slam.rviz` only.

**Verification:**
```bash
python3 -c "
import yaml
with open('src/robot_navigation/navigation/rviz/navigation.rviz') as f:
    yaml.safe_load(f)
print('RViz YAML OK')
"
```

---

## 5. Dependency Changes Summary

| Scope | Dependency | Reason |
|---|---|---|
| `package.xml <depend>` | `nav2_costmap_2d` | Costmap nodes |
| `package.xml <depend>` | `nav2_lifecycle_manager` | Lifecycle management |
| `package.xml <depend>` | `nav2_planner` | Planner server |
| `package.xml <depend>` | `nav2_controller` | Controller server |
| `package.xml <depend>` | `nav2_bt_navigator` | BT navigator |
| `package.xml <depend>` | `nav2_behaviors` | Recovery behaviors |
| `package.xml <depend>` | `nav2_msgs` | Action/service message types |
| `package.xml <depend>` | `nav2_util` | Shared Nav2 utilities |
| `package.xml <depend>` | `python3-numpy` | `frontier_explorer.py` (was missing) |
| `package.xml <depend>` | `python3-scipy` | `frontier_explorer.py` (was missing) |
| `setup.py install_requires` | `numpy`, `scipy` | pip-level declaration |

---

## 6. Execution Order

```
1.  Modify  package.xml                          → add <depend> entries
2.  Modify  setup.py                             → add numpy/scipy to install_requires
3.  Create  navigation/config/nav2_params.yaml   → full Nav2 parameter file
4.  Create  navigation/launch/navigation_launch.py
5.  Create  navigation/rviz/navigation.rviz
6.  Modify  simulation/launch/slam_bringup.py    → add launch_navigation arg
7.  colcon build --packages-select robot_navigation
8.  source install/setup.bash
9.  Run verification procedure (Section 7)
```

---

## 7. Runtime Verification Procedure

### Step 1 — Build

```bash
cd /home/aarush-sivaraman/Robotics/workspaces/robot_navigation
colcon build --packages-select robot_navigation
```

**Expected:** `1 package finished`  
**Failure:** Import error in `setup.py` or missing data file → check walker excludes `__init__.py`

### Step 2 — Launch full stack

```bash
source install/setup.bash
ros2 launch robot_navigation slam_bringup.py launch_navigation:=true
```

In Gazebo GUI: press ▶ **Play**.

### Step 3 — Verify Nav2 nodes

```bash
ros2 node list | grep -E "costmap|controller|planner|bt_navigator|behavior|lifecycle"
```

**Expected:** `/global_costmap`, `/local_costmap`, `/controller_server`, `/planner_server`,
`/bt_navigator`, `/behavior_server`, `/lifecycle_manager_costmaps`,
`/lifecycle_manager_navigation`  
**Failure:** Missing nodes → lifecycle manager not finding them → node names in
`managed_nodes` list must match exactly

### Step 4 — Verify lifecycle active state

```bash
ros2 lifecycle get /global_costmap
ros2 lifecycle get /local_costmap
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
```

**Expected:** All report `active [3]`  
**Failure:** `inactive` or `unconfigured` → lifecycle manager failed to transition →
check `nav2_params.yaml` is found and valid

### Step 5 — Verify costmap publication rates

```bash
ros2 topic hz /global_costmap/costmap   # expect ~1.0 Hz
ros2 topic hz /local_costmap/costmap    # expect ~2.0 Hz
```

**Failure:** No output → costmap node failed to init → most likely `map_subscribe_transient_local`
is missing

### Step 6 — Verify map reaches global costmap

```bash
ros2 topic echo /global_costmap/costmap --once | head -8
```

**Expected:** `header.frame_id: map`, non-zero `data` array  
**Failure:** All zeros → `static_layer` not receiving `/map` → check QoS setting

### Step 7 — Verify scan reaches costmaps

```bash
ros2 topic echo /local_costmap/costmap_raw --once | grep -c "254\|253\|252"
```

**Expected:** Non-zero count (obstacle cells visible)  
**Failure:** All free/unknown → `/scan` topic name mismatch in `obstacle_layer` config

### Step 8 — Verify complete TF chain

```bash
ros2 run tf2_ros tf2_echo map base_link
```

**Expected:** Continuous `Translation`/`Rotation` output  
**Failure:** `Could not find a connection` → SLAM not running or bridge not started

### Step 9 — Verify NavigateToPose action server

```bash
ros2 action list | grep navigate_to_pose
```

**Expected:** `/navigate_to_pose`  
**Failure:** Missing → `bt_navigator` not in active state

---

## 8. RViz Verification

```bash
ros2 run rviz2 rviz2 \
  -d src/robot_navigation/navigation/rviz/navigation.rviz \
  --ros-args -p use_sim_time:=true
```

**Required observations:**

- [ ] Grey/white SLAM map visible (`/map`)
- [ ] Global costmap overlay showing inflated obstacles around walls and shelves
- [ ] Local costmap visible as a rolling rectangle following the robot
- [ ] Red laser scan points visible on `/scan`
- [ ] `base_link` frame visible in TF display
- [ ] Footprint polygon visible at robot position (`/global_costmap/published_footprint`)
- [ ] Path display present (empty until a goal is sent)

---

## 9. Failure Modes and Diagnostics

| Symptom | Root Cause | Diagnostic |
|---|---|---|
| Lifecycle nodes stay `inactive` | Node names in `managed_nodes` don't match actual node names | `ros2 node list` vs `managed_nodes` param list |
| Global costmap all unknown | `map_subscribe_transient_local: false` or SLAM not running | `ros2 topic echo /map --once` |
| No obstacles in costmap | `/scan` topic or frame mismatch | `ros2 topic echo /scan --once` → check `frame_id: scan_front` |
| `Could not look up transform` spam | TF chain broken or `transform_tolerance` too tight | `ros2 run tf2_tools view_frames` |
| `navigate_to_pose` action missing | `bt_navigator` not `active` | `ros2 lifecycle get /bt_navigator` |
| costmap update timeout errors | `controller_frequency` > laser update rate | Reduce `controller_frequency` to ≤ 10 Hz |
| `bond_timeout` disconnects | Nodes not starting in time | Increase `TimerAction` delay or `bond_timeout` param |
| YAML param not found | `params_file` path wrong at runtime | Print resolved path in launch file before node declaration |

---

## 10. Definition of Done (Hard Gate)

Stage 1 is complete **only** when all of the following pass simultaneously in a running session:

- [ ] `colcon build --packages-select robot_navigation` exits `0`
- [ ] `ros2 node list` shows all 8 Nav2 nodes listed in Section 4.4
- [ ] All 5 queried lifecycle nodes report `active [3]`
- [ ] `ros2 topic hz /global_costmap/costmap` shows ≥ 0.8 Hz
- [ ] `ros2 topic hz /local_costmap/costmap` shows ≥ 1.5 Hz
- [ ] `ros2 topic echo /global_costmap/costmap --once` produces non-zero `data`
- [ ] `ros2 action list` includes `/navigate_to_pose`
- [ ] `ros2 run tf2_ros tf2_echo map base_link` produces continuous output
- [ ] RViz shows map + both costmaps + laser + footprint simultaneously
- [ ] No `transform timeout` or `costmap update timeout` errors in `/rosout`

---

## 11. Files Modified / Created Summary

### Files to Modify

| File | Change |
|---|---|
| `package.xml` | Add 10 `<depend>` entries |
| `setup.py` | Add `numpy`, `scipy` to `install_requires` |
| `src/robot_navigation/simulation/launch/slam_bringup.py` | Add `launch_navigation` arg + conditional Nav2 include |

### Files to Create

| File | Purpose |
|---|---|
| `src/robot_navigation/navigation/config/nav2_params.yaml` | All Nav2 parameters |
| `src/robot_navigation/navigation/launch/navigation_launch.py` | Nav2 lifecycle node bringup |
| `src/robot_navigation/navigation/rviz/navigation.rviz` | Navigation visualization config |

### Files with No Change

`tugbot_bridge.yaml`, `slam_toolbox_online_async.yaml`, `slam.rviz`, `slam_launch.py`,
`bridge_launch.py`, `gazebo_launch.py`, `frontier_explorer.py`, `exploration_launch.py`,
all `world_analyzer/` files, all tests.

---

## 12. Handoff to Stage 2

Stage 2 (frontier explorer → `NavigateToPose` integration) may assume:

- `/navigate_to_pose` action server is active and responding
- `/global_costmap/costmap` and `/local_costmap/costmap` are publishing
- `base_link` is localized in `map` frame continuously
- `/plan` topic is available for path visualization
- `robot_radius: 0.40` m is the authoritative footprint value — Stage 2 uses this for
  goal clearance validation in the frontier filter
- Planner is NavFn — Stage 2 swaps only the `plugin:` string under `GridBased` to
  install the custom A* planner
- `ros2 launch robot_navigation slam_bringup.py launch_navigation:=true` starts the
  entire stack in one command

### Stage 2 must NOT touch

`nav2_params.yaml` costmap sections, `navigation_launch.py` node definitions,
`package.xml` (unless adding new deps), `slam_launch.py`, `bridge_launch.py`,
`tugbot_bridge.yaml`.

### Stage 2 will modify

`frontier_explorer.py` — add `NavigateToPose` action client to replace the current
pose-publish-only behavior.  
`nav2_params.yaml` — planner `plugin:` string only, when custom A* is ready.
