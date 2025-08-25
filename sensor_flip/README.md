# sensor_flip

A small ROS 2 node to apply a fixed rotation (e.g., 180° roll) to LiDAR PointCloud2 and IMU messages.

- Input topics (defaults):
  - /livox/lidar
  - /livox/imu
- Output topics (defaults):
  - /livox_flipped/lidar
  - /livox_flipped/imu

Parameters:
- roll_deg, pitch_deg, yaw_deg: degrees, default 180, 0, 0
- lidar_in, imu_in, lidar_out, imu_out

Launch:
ros2 launch sensor_flip flip.launch.py

Example (180° roll flip, using Livox driver topics):
ros2 launch sensor_flip flip.launch.py lidar_in:=/livox/lidar imu_in:=/livox/imu lidar_out:=/livox/lidar_flipped imu_out:=/livox/imu_flipped roll_deg:=180.0 pitch_deg:=0.0 yaw_deg:=0.0
