#!/bin/bash

# Source ROS2 script utilities
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/ros_utils.sh"

# Source the setup files if not in docker container
[ -f ~/unitree_ros2/setup.sh ] && source ~/unitree_ros2/setup.sh
[ -f ~/ws_navigation/install/setup.bash ] && source ~/ws_navigation/install/setup.bash
[ -f ~/g1_navigation_ws/install/setup.bash ] && source ~/g1_navigation_ws/install/setup.bash

# Initialize directory and filename variables
BAG_DIR=$HOME/ros2_bags/navigation_bag
DATE_FOLDER=$(date +%Y%m%d)
FULL_BAG_DIR=$BAG_DIR/$DATE_FOLDER
mkdir -p $FULL_BAG_DIR

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BAG_FILENAME="navigation_bag_${TIMESTAMP}"

# Function to cleanup on exit
# Usage: cleanup <success_flag> <pid1> [pid2] [pid3] ...
# success_flag: "success" to show bag info, "error" to skip bag info
cleanup() {
    local success_flag="$1"
    shift  # Remove first argument, remaining are PIDs
    
    # Use the generic process killer from ros_utils.sh
    kill_processes_with_timeout "$@"
    
    # Show bag info only on successful completion
    if [ "$success_flag" = "success" ]; then
        ros2 bag info $FULL_BAG_DIR/${BAG_FILENAME}_livox
        ros2 bag info $FULL_BAG_DIR/${BAG_FILENAME}_zed
        ros2 bag info $FULL_BAG_DIR/${BAG_FILENAME}_other        
        echo "All bags saved to: $FULL_BAG_DIR/$BAG_FILENAME"
        exit 0
    else
        echo "Process cleanup completed due to error."
        exit 1
    fi
}

# Start Livox MID360 driver
echo "Starting Livox MID360 driver..."
setsid ros2 launch livox_ros_driver2 msg_MID360_launch.py &
LIVOX_PID=$!

# Launch the ZED camera wrapper
echo "Starting ZED camera wrapper..."
setsid ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i &
ZED_PID=$!

# Start web video server for GUI camera streaming
echo "Starting web video server for GUI..."
setsid ros2 run web_video_server web_video_server &
WEB_VIDEO_SERVER_PID=$!

# Define topic groups to be recorded
LIVOX_TOPICS=("/livox/lidar" "/livox/imu")
ZED_TOPICS=("/zed/zed_node/left/image_rect_color/compressed" "/zed/zed_node/right/image_rect_color/compressed" "/zed/zed_node/left/camera_info" "/zed/zed_node/right/camera_info")
OTHER_TOPICS=("/lowstate" "/lowcmd" "/wirelesscontroller" "/odommodestate" "/secondary_imu")
ALL_TOPICS=("${LIVOX_TOPICS[@]}" "${ZED_TOPICS[@]}" "${OTHER_TOPICS[@]}")

# Check all required topics with polling and timeout
echo "Verifying all required topics are available..."
if ! check_topics_exist_with_timeout "${ALL_TOPICS[@]}"; then
    echo "ERROR: Required topics not found after timeout. Stopping process."
    cleanup "error" "LIVOX_PID" "ZED_PID" "WEB_VIDEO_SERVER_PID"
fi
echo "All topics verified and ready for recording."

echo "Starting navigation bag recording..."
echo "Bag will be saved to: $FULL_BAG_DIR/$BAG_FILENAME"

# Start Livox and ZED bag recording in parallel
echo "Starting ROS2 bag recording for Livox and ZED topics..."
setsid ros2 bag record --storage mcap "${LIVOX_TOPICS[@]}" -o $FULL_BAG_DIR/${BAG_FILENAME}_livox & 
ROS2_BAG_LIVOX_PID=$!

setsid ros2 bag record --storage mcap "${ZED_TOPICS[@]}" -o $FULL_BAG_DIR/${BAG_FILENAME}_zed & 
ROS2_BAG_ZED_PID=$!

# Start recording all other necessary topics in the third bag
echo "Starting ROS2 bag recording for other topics..."
setsid ros2 bag record --storage mcap "${OTHER_TOPICS[@]}" -o $FULL_BAG_DIR/${BAG_FILENAME}_other & 
ROS2_BAG_OTHER_PID=$!

echo "All components started. Press Ctrl+C to stop recording."

# Set up signal handlers
trap 'cleanup "success" "LIVOX_PID" "ZED_PID" "WEB_VIDEO_SERVER_PID" "ROS2_BAG_LIVOX_PID" "ROS2_BAG_ZED_PID" "ROS2_BAG_OTHER_PID"' SIGINT SIGTERM

# Wait for all background processes
wait
