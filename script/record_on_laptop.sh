#!/bin/bash
set -e

# Source the setup files
echo "Setting up ROS2 environment..."
[ -f "$HOME/superodom_ws/install/setup.bash" ] && source "$HOME/superodom_ws/install/setup.bash"
[ -f "$HOME/unitree_ros2/install/setup.bash" ] && source "$HOME/unitree_ros2/install/setup.bash"
[ -f "$HOME/unitree_ros2/setup.bash" ] && source "$HOME/unitree_ros2/setup.bash"

# Setup directory
BAG_DIR="$HOME/ros2_bags/data_collection_bag"
DATE_FOLDER=$(date +%Y%m%d)
FULL_BAG_DIR="$BAG_DIR/$DATE_FOLDER"
mkdir -p "$FULL_BAG_DIR"

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BAG_FILENAME="humanoid_imu_${TIMESTAMP}"

echo "Bag directory: $FULL_BAG_DIR"
echo "Bag filename: $BAG_FILENAME"

# Clean up any existing Livox processes that might be holding ports
echo "Checking for existing Livox processes..."
LIVOX_COUNT=$(ps aux | grep -c "[l]ivox_ros_driver2_node" || echo "0")
if [ "$LIVOX_COUNT" -gt 0 ]; then
    echo "Found $LIVOX_COUNT existing Livox driver process(es). Cleaning up..."
    pkill -9 -f "livox_ros_driver2_node" 2>/dev/null || true
    pkill -9 -f "msg_MID360_launch" 2>/dev/null || true
    sleep 2
    echo "Cleanup complete."
fi

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
        if [ "$pub_count" -eq 0 ]; then
            echo "  ⚠ $topic (EXISTS but NO PUBLISHERS - will record 0 messages!)"
            return 2
        else
            echo "  ✓ $topic (Publishers: $pub_count, Subscribers: $sub_count)"
            return 0
        fi
    else
        echo "  ✗ $topic (NOT AVAILABLE)"
        return 1
    fi
}

# Function to verify topic is actually publishing messages
# Note: This may fail if Python bindings are missing, but that's OK - ros2 bag record uses C++
verify_topic_publishing() {
    local topic="$1"
    local timeout_sec=3
    echo -n "    Checking if $topic is publishing messages... "
    
    # Try ros2 topic hz, but handle Python binding errors gracefully
    local hz_output=$(timeout $timeout_sec ros2 topic hz "$topic" 2>&1)
    
    # Check for Python binding errors
    if echo "$hz_output" | grep -q "UnsupportedTypeSupport\|ModuleNotFoundError\|rosidl_typesupport"; then
        echo "⚠ Cannot verify (Python bindings missing, but C++ recording should work)"
        echo "      Note: ros2 bag record uses C++ and should work fine"
        return 0  # Return success since C++ tools will work
    fi
    
    # Check if we got rate information
    if echo "$hz_output" | grep -q "average rate"; then
        local rate=$(echo "$hz_output" | grep "average rate" | head -1 | awk '{print $3}')
        echo "✓ Publishing at ~${rate} Hz"
        return 0
    else
        echo "✗ NOT publishing (no messages received in ${timeout_sec}s)"
        echo "      This topic will record 0 messages!"
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

MISSING_TOPICS=0
NO_PUBLISHER_TOPICS=0

echo "Livox topics:"
for topic in "${LIVOX_TOPICS[@]}"; do
    get_topic_info "$topic"
    result=$?
    if [ $result -eq 1 ]; then
        MISSING_TOPICS=$((MISSING_TOPICS + 1))
    elif [ $result -eq 2 ]; then
        NO_PUBLISHER_TOPICS=$((NO_PUBLISHER_TOPICS + 1))
    fi
done
echo ""
echo "Other topics:"
for topic in "${OTHER_TOPICS[@]}"; do
    get_topic_info "$topic"
    result=$?
    if [ $result -eq 1 ]; then
        MISSING_TOPICS=$((MISSING_TOPICS + 1))
    elif [ $result -eq 2 ]; then
        NO_PUBLISHER_TOPICS=$((NO_PUBLISHER_TOPICS + 1))
    fi
done
echo ""
echo "================================"
echo ""
echo "Verifying topics are actually publishing messages..."
echo ""
for topic in "${OTHER_TOPICS[@]}"; do
    if check_topic_exists "$topic"; then
        verify_topic_publishing "$topic" || NO_PUBLISHER_TOPICS=$((NO_PUBLISHER_TOPICS + 1))
    fi
done
echo ""
echo "================================"
echo ""

# Verify critical topics are actually publishing
if [ $NO_PUBLISHER_TOPICS -gt 0 ] || [ $MISSING_TOPICS -gt 0 ]; then
    echo "⚠ WARNING: Some topics may not record any messages!"
    echo ""
    if [ $NO_PUBLISHER_TOPICS -gt 0 ]; then
        echo "Topics with no publishers (will record 0 messages):"
        for topic in "${ALL_TOPICS[@]}"; do
            if check_topic_exists "$topic"; then
                pub_count=$(timeout 1 ros2 topic info "$topic" 2>/dev/null | grep -c "Publisher" || echo "0")
                if [ "$pub_count" -eq 0 ]; then
                    echo "  - $topic"
                fi
            fi
        done
        echo ""
    fi
    if [ $MISSING_TOPICS -gt 0 ]; then
        echo "Topics not found:"
        for topic in "${ALL_TOPICS[@]}"; do
            if ! check_topic_exists "$topic"; then
                echo "  - $topic"
            fi
        done
        echo ""
    fi
    echo "Waiting 5 more seconds for topics to become available..."
    sleep 5
    echo ""
fi

echo "Starting navigation bag recording..."
echo "Bag will be saved to: $FULL_BAG_DIR/$BAG_FILENAME"
echo ""

# Final verification: Check if topics are publishing before recording
echo "Final verification - checking if topics are publishing messages..."
VERIFY_FAILED=0
for topic in "${ALL_TOPICS[@]}"; do
    if check_topic_exists "$topic"; then
        pub_count=$(timeout 1 ros2 topic info "$topic" 2>/dev/null | grep -c "Publisher" || echo "0")
        if [ "$pub_count" -eq 0 ]; then
            echo "  ⚠ WARNING: $topic has no publishers - will record 0 messages!"
            VERIFY_FAILED=$((VERIFY_FAILED + 1))
        fi
    else
        echo "  ✗ ERROR: $topic does not exist!"
        VERIFY_FAILED=$((VERIFY_FAILED + 1))
    fi
done

if [ $VERIFY_FAILED -gt 0 ]; then
    echo ""
    echo "⚠ WARNING: $VERIFY_FAILED topic(s) may not record any data!"
    echo "   Make sure all required nodes are running before recording."
    echo "   Continuing anyway in 3 seconds..."
    sleep 3
    echo ""
fi

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
