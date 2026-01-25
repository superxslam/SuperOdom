#!/usr/bin/env bash
# Play two rosbag2 dirs in parallel for humanoid dataset.
# One player publishes /clock; both start paused; then resume together.
# Usage: ./play_humanoid_bags.sh <livox_bag_dir> <other_bag_dir>

set -euo pipefail

if [ "$#" -ne 2 ]; then
  echo "Usage: $0 <livox_bag_dir> <other_bag_dir>"
  echo "Example: $0 ~/humanoid_dataset/12-05-dancing1/imu_foundation_bag_20251205_145009_livox ~/humanoid_dataset/12-05-dancing1/imu_foundation_bag_20251205_145009_other"
  exit 1
fi

LIVOX_BAG="$1"
OTHER_BAG="$2"

# Source ROS if needed
if [ -z "${ROS_DISTRO:-}" ]; then
  source /opt/ros/humble/setup.bash
fi

for d in "$LIVOX_BAG" "$OTHER_BAG"; do
  if [ ! -d "$d" ]; then
    echo "[error] Not a directory: $d"
    exit 2
  fi
done

PLAYER_LIVOX="player_livox"
PLAYER_OTHER="player_other"

# Helper: wait until a service appears
wait_for_service() {
  local srv="$1"
  echo "[info] Waiting for service $srv ..."
  local max_wait=10
  local waited=0
  until ros2 service list | grep -qE "(^| )${srv}($| )"; do
    sleep 0.1
    waited=$((waited + 1))
    if [ $waited -ge $max_wait ]; then
      echo "[warn] Service $srv not found after ${max_wait}s, continuing anyway..."
      break
    fi
  done
}

echo "[info] Launching players paused…"

# Play livox bag with --clock flag (publishes /clock)
ros2 bag play "$LIVOX_BAG" --start-paused --clock 100 \
  --remap __node:=${PLAYER_LIVOX} \
  >/tmp/${PLAYER_LIVOX}.log 2>&1 &
PID_LIVOX=$!

# Play other bag without --clock (uses existing clock from livox bag)
ros2 bag play "$OTHER_BAG" --start-paused \
  --remap __node:=${PLAYER_OTHER} \
  >/tmp/${PLAYER_OTHER}.log 2>&1 &
PID_OTHER=$!

# Wait for their /resume services to exist
wait_for_service "/${PLAYER_LIVOX}/resume"
wait_for_service "/${PLAYER_OTHER}/resume"

# Set use_sim_time for all nodes
echo "[info] Setting use_sim_time=true..."
ros2 param set /use_sim_time true 2>/dev/null || true

echo "[info] Resuming both players…"
# Resume both players
ros2 service call "/${PLAYER_LIVOX}/resume" rosbag2_interfaces/srv/Resume "{}" >/dev/null 2>&1 &
ros2 service call "/${PLAYER_OTHER}/resume" rosbag2_interfaces/srv/Resume "{}" >/dev/null 2>&1 &
sleep 1

echo "[info] Both players running. Ctrl+C to stop."
echo "[info] Livox bag PID: $PID_LIVOX"
echo "[info] Other bag PID: $PID_OTHER"
echo "[info] Logs: /tmp/${PLAYER_LIVOX}.log and /tmp/${PLAYER_OTHER}.log"

# Wait for both processes
wait $PID_LIVOX $PID_OTHER || true

echo "[info] Bag playback completed."
