#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# slam_bringup.sh  —  SLAM bring-up without a colcon workspace
#
# Launches each component in a separate terminal tab/window.
# Requires: tmux (apt install tmux) OR run manually in separate terminals.
#
# Usage:
#   chmod +x slam_bringup.sh
#   ./slam_bringup.sh                     # uses tmux
#   ./slam_bringup.sh --no-tmux           # prints commands to run manually
#
# Prerequisites:
#   source /opt/ros/jazzy/setup.bash
# ─────────────────────────────────────────────────────────────────────────────

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORLD_FILE="${REPO_ROOT}/Worlds_models/tugbot_warehouse.sdf"
BRIDGE_CONFIG="${SCRIPT_DIR}/src/robot_navigation/simulation/bridge/tugbot_bridge.yaml"
SLAM_CONFIG="${SCRIPT_DIR}/src/robot_navigation/slam/config/slam_toolbox_online_async.yaml"
RVIZ_CONFIG="${SCRIPT_DIR}/src/robot_navigation/slam/rviz/slam.rviz"

# Verify files exist
for f in "${WORLD_FILE}" "${BRIDGE_CONFIG}" "${SLAM_CONFIG}" "${RVIZ_CONFIG}"; do
    if [[ ! -f "${f}" ]]; then
        echo "ERROR: Required file missing: ${f}" >&2
        exit 1
    fi
done

# Source ROS 2 if not already sourced
if [[ -z "${ROS_DISTRO:-}" ]]; then
    echo "Sourcing ROS 2 Jazzy..."
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
fi

echo "═══════════════════════════════════════════════════════════"
echo "  Tugbot Warehouse SLAM Bring-up"
echo "  World:  ${WORLD_FILE}"
echo "  Bridge: ${BRIDGE_CONFIG}"
echo "  SLAM:   ${SLAM_CONFIG}"
echo "  RViz:   ${RVIZ_CONFIG}"
echo "═══════════════════════════════════════════════════════════"

USE_TMUX=true
if [[ "${1:-}" == "--no-tmux" ]]; then
    USE_TMUX=false
fi

# ── Commands ──────────────────────────────────────────────────────────────────
CMD_GAZEBO="gz sim \"${WORLD_FILE}\""

CMD_BRIDGE="ros2 run ros_gz_bridge parameter_bridge \
  --ros-args -p config_file:=\"${BRIDGE_CONFIG}\" -p use_sim_time:=true"

CMD_SLAM="ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args --params-file \"${SLAM_CONFIG}\" -p use_sim_time:=true"

CMD_RVIZ="ros2 run rviz2 rviz2 -d \"${RVIZ_CONFIG}\" \
  --ros-args -p use_sim_time:=true"

if [[ "${USE_TMUX}" == "false" ]]; then
    echo ""
    echo "Run these commands in SEPARATE terminals (source ROS 2 first in each):"
    echo ""
    echo "── Terminal 1: Gazebo ──────────────────────────────────"
    echo "${CMD_GAZEBO}"
    echo ""
    echo "── Terminal 2: Bridge (after Gazebo loads) ─────────────"
    echo "source /opt/ros/jazzy/setup.bash"
    echo "${CMD_BRIDGE}"
    echo ""
    echo "── Terminal 3: SLAM Toolbox ────────────────────────────"
    echo "source /opt/ros/jazzy/setup.bash"
    echo "${CMD_SLAM}"
    echo ""
    echo "── Terminal 4: RViz2 ───────────────────────────────────"
    echo "source /opt/ros/jazzy/setup.bash"
    echo "${CMD_RVIZ}"
    echo ""
    echo "── Terminal 5: Teleoperation (keyboard) ────────────────"
    echo "source /opt/ros/jazzy/setup.bash"
    echo "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel"
    exit 0
fi

# ── tmux session ──────────────────────────────────────────────────────────────
SESSION="slam_bringup"

# Kill existing session if it exists
tmux kill-session -t "${SESSION}" 2>/dev/null || true

echo "Starting tmux session: ${SESSION}"
tmux new-session -d -s "${SESSION}" -x 220 -y 50

# Window 0: Gazebo
tmux rename-window -t "${SESSION}:0" "Gazebo"
tmux send-keys -t "${SESSION}:0" "${CMD_GAZEBO}" Enter

# Window 1: Bridge (auto-delayed)
tmux new-window -t "${SESSION}" -n "Bridge"
tmux send-keys -t "${SESSION}:1" \
    "source /opt/ros/jazzy/setup.bash && sleep 5 && ${CMD_BRIDGE}" Enter

# Window 2: SLAM
tmux new-window -t "${SESSION}" -n "SLAM"
tmux send-keys -t "${SESSION}:2" \
    "source /opt/ros/jazzy/setup.bash && sleep 10 && ${CMD_SLAM}" Enter

# Window 3: RViz
tmux new-window -t "${SESSION}" -n "RViz"
tmux send-keys -t "${SESSION}:3" \
    "source /opt/ros/jazzy/setup.bash && sleep 12 && ${CMD_RVIZ}" Enter

# Window 4: Teleop
tmux new-window -t "${SESSION}" -n "Teleop"
tmux send-keys -t "${SESSION}:4" \
    "source /opt/ros/jazzy/setup.bash && sleep 6 && \
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/cmd_vel" Enter

echo ""
echo "tmux session '${SESSION}' started."
echo "Attach with:  tmux attach -t ${SESSION}"
echo "Kill with:    tmux kill-session -t ${SESSION}"
echo ""
tmux attach -t "${SESSION}"
