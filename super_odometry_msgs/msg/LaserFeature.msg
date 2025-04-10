# feature Info
std_msgs/Header header 

int64 sensor

int64 imu_available
int64 odom_available

# IMU initial guess for laser mapping
float64 imu_quaternion_x
float64 imu_quaternion_y
float64 imu_quaternion_z
float64 imu_quaternion_w

# Odometry initial guess for laser mapping
float64 initial_pose_x
float64 initial_pose_y
float64 initial_pose_z
float64 initial_quaternion_x
float64 initial_quaternion_y
float64 initial_quaternion_z
float64 initial_quaternion_w

# Preintegration reset ID
int64 imu_preintegration_reset_id

# Point cloud messages
sensor_msgs/PointCloud2 cloud_nodistortion  # original cloud remove distortion
sensor_msgs/PointCloud2 cloud_corner    # extracted corner feature
sensor_msgs/PointCloud2 cloud_surface   # extracted surface feature
sensor_msgs/PointCloud2 cloud_realsense   # extracted surface feature from realsense