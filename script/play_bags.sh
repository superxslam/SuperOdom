#!/usr/bin/env bash
# Play two rosbag2 dirs in parallel on ROS 2 Humble.
# One player publishes /clock; both start paused; then resume both together.
# Usage: ./parallel_play.sh <bag_dir_A> <bag_dir_B>

set -euo pipefail

if [ "$#" -ne 2 ]; then
  echo "Usage: $0 <bag_dir_A> <bag_dir_B>"
  exit 1
fi

BAG_A="$1"
BAG_B="$2"

# Source ROS if needed
if [ -z "${ROS_DISTRO:-}" ]; then
  source /opt/ros/humble/setup.bash
fi

for d in "$BAG_A" "$BAG_B"; do
  if [ ! -d "$d" ]; then
    echo "[error] Not a directory: $d"; exit 2
  fi
done

PLAYER_A="player_a"
PLAYER_B="player_b"

# Helper: wait until a service appears
wait_for_service() {
  local srv="$1"
  echo "[info] Waiting for service $srv ..."
  until ros2 service list | grep -qE "(^| )${srv}($| )"; do
    sleep 0.1
  done
}

echo "[info] Launching players paused…"

# NOTE: give an explicit Hz to --clock to avoid it swallowing the next arg
ros2 bag play "$BAG_A" --start-paused --clock 100 \
  --remap __node:=${PLAYER_A} \
  >/tmp/${PLAYER_A}.log 2>&1 &
PID_A=$!

ros2 bag play "$BAG_B" --start-paused --clock \
  --remap __node:=${PLAYER_B} \
  >/tmp/${PLAYER_B}.log 2>&1 &
PID_B=$!

# Wait for their /resume services to exist
wait_for_service "/${PLAYER_A}/resume"
wait_for_service "/${PLAYER_B}/resume"

# (Optional) tell your consuming nodes to use sim time
# ros2 param set /your_node use_sim_time true || true

echo "[info] Resuming both players…"
# Humble's CLI: simple calls, no `wait` subcommand available
ros2 service call "/${PLAYER_A}/resume" rosbag2_interfaces/srv/Resume "{}" >/dev/null &
ros2 service call "/${PLAYER_B}/resume" rosbag2_interfaces/srv/Resume "{}" >/dev/null &
wait

echo "[info] Both players running. Ctrl+C to stop."
wait $PID_A $PID_B || true
