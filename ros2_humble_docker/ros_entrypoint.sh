#!/bin/bash
 
set -e

# Ros build
source "/opt/ros/humble/setup.bash"

echo "==============SuperOdom ROS2 Docker Env Ready================"

cd /root/superodom_ws

exec "$@"
