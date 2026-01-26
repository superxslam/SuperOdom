#!/bin/bash
set -e

# Source the setup files
echo "Setting up ROS2 environment..."
[ -f "$HOME/superodom_ws/install/setup.bash" ] && source "$HOME/superodom_ws/install/setup.bash"
[ -f "$HOME/unitree_ros2_ws/install/setup.bash" ] && source "$HOME/unitree_ros2_ws/install/setup.bash"

# Setup directory
BAG_DIR="$HOME/ros2_bags/data_collection_bag"
DATE_FOLDER=$(date +%Y%m%d)
FULL_BAG_DIR="$BAG_DIR/$DATE_FOLDER"
mkdir -p "$FULL_BAG_DIR"

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BAG_FILENAME="humanoid_imu_${TIMESTAMP}"

echo "Bag directory: $FULL_BAG_DIR"
echo "Bag filename: $BAG_FILENAME"

# Cleanup function
cleanup() {
    success_flag="$1"
    shift  # Remove first argument, rest are PIDs to kill
    
    echo ""
    echo "================================"
    echo "Stopping all processes..."
    echo "================================"
    
    # Kill all background processes
    for pid_var in "$@"; do
        pid=${!pid_var}
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            echo "Stopping process $pid_var (PID: $pid)..."
            kill "$pid" 2>/dev/null || true
        fi
    done
    
    # Wait a moment for processes to stop
    sleep 2
    
    # Force kill if still running
    for pid_var in "$@"; do
        pid=${!pid_var}
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            echo "Force stopping process $pid_var (PID: $pid)..."
            kill -9 "$pid" 2>/dev/null || true
        fi
    done
    
    # Show bag info only on successful completion
    if [ "$success_flag" = "success" ]; then
        echo ""
        echo "================================"
        echo "RECORDING SUMMARY"
        echo "================================"
        echo ""
        echo "Bag files directory: $FULL_BAG_DIR"
        echo ""
        echo "Saved bag files:"
        echo "  - ${BAG_FILENAME}_livox"
        echo "  - ${BAG_FILENAME}_zed"
        echo "  - ${BAG_FILENAME}_other"
        echo ""
        echo "Recorded topics:"
        echo "  Livox topics:"
        for topic in "${LIVOX_TOPICS[@]}"; do
            echo "    - $topic"
        done
        echo "  ZED topics:"
        for topic in "${ZED_TOPICS[@]}"; do
            echo "    - $topic"
        done
        echo "  Other topics:"
        for topic in "${OTHER_TOPICS[@]}"; do
            echo "    - $topic"
        done
        echo ""
        echo "Bag details:"
        ros2 bag info $FULL_BAG_DIR/${BAG_FILENAME}_livox
        echo ""
        ros2 bag info $FULL_BAG_DIR/${BAG_FILENAME}_zed
        echo ""
        ros2 bag info $FULL_BAG_DIR/${BAG_FILENAME}_other
        echo ""
        echo "================================"
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
# echo "Starting ZED camera wrapper..."
# setsid ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i &
# ZED_PID=$!

# Start web video server for GUI camera streaming
# echo "Starting web video server for GUI..."
# setsid ros2 run web_video_server web_video_server &
# WEB_VIDEO_SERVER_PID=$!

# Define topic groups to be recorded
LIVOX_TOPICS=("/livox/lidar" "/livox/imu")
ZED_TOPICS=("/zed/zed_node/left/image_rect_color/compressed" "/zed/zed_node/right/image_rect_color/compressed" "/zed/zed_node/left/camera_info" "/zed/zed_node/right/camera_info")
OTHER_TOPICS=("/lowstate" "/lowcmd" "/wirelesscontroller" "/odommodestate" "/secondary_imu")
#ALL_TOPICS=("${LIVOX_TOPICS[@]}" "${ZED_TOPICS[@]}" "${OTHER_TOPICS[@]}")
ALL_TOPICS=("${LIVOX_TOPICS[@]}" "${OTHER_TOPICS[@]}")

# Function to check if a topic exists
check_topic_exists() {
    local topic="$1"
    timeout 1 ros2 topic list | grep -q "^${topic}$"
}

# Function to check topic message count (rough estimate)
get_topic_info() {
    local topic="$1"
    if check_topic_exists "$topic"; then
        local pub_count=$(timeout 1 ros2 topic info "$topic" 2>/dev/null | grep -c "Publisher" || echo "0")
        local sub_count=$(timeout 1 ros2 topic info "$topic" 2>/dev/null | grep -c "Subscription" || echo "0")
        echo "  ✓ $topic (Publishers: $pub_count, Subscribers: $sub_count)"
        return 0
    else
        echo "  ✗ $topic (NOT AVAILABLE)"
        return 1
    fi
}

# Wait a bit for topics to become available
echo "Waiting for topics to become available..."
sleep 3

# Check and display topic availability
echo ""
echo "================================"
echo "TOPIC AVAILABILITY CHECK"
echo "================================"
echo ""
echo "Checking topics to be recorded:"
echo ""
echo "Livox topics:"
for topic in "${LIVOX_TOPICS[@]}"; do
    get_topic_info "$topic"
done
echo ""
echo "Other topics:"
for topic in "${OTHER_TOPICS[@]}"; do
    get_topic_info "$topic"
done
echo ""
echo "================================"
echo ""
echo "NOTE: /lowcmd will only have messages if a controller is actively sending commands."
echo "      If no controller is running, this topic will be empty (0 messages)."
echo ""

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

# Give recording processes a moment to initialize
sleep 2

echo "All components started. Press Ctrl+C to stop recording."
echo ""
echo "Recording is active. Make sure your robot controller is running if you want /lowcmd messages."

# Set up signal handlers
trap 'cleanup "success" "LIVOX_PID" "ZED_PID" "WEB_VIDEO_SERVER_PID" "ROS2_BAG_LIVOX_PID" "ROS2_BAG_ZED_PID" "ROS2_BAG_OTHER_PID"' SIGINT SIGTERM

# Wait for all background processes
wait
