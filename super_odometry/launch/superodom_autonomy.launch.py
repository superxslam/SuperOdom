"""
SuperOdom launch file for Base Autonomy and Route Planner integration.

This launch file runs SuperOdom (Livox Mid360).
SuperOdom publishes (with PROJECT_NAME=""):
  - /laser_odometry (10Hz odometry from scan matching)
  - /registered_scan (registered point cloud)

Future: 100 Hz odometry from /state_estimation (when IMU preintegration is enabled)
"""

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros

def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)

def generate_launch_description():
    # Config paths
    config_path = get_share_file(
        package_name="super_odometry",
        file_name="config/livox_mid360.yaml")
    calib_path = get_share_file(
        package_name="super_odometry",
        file_name="config/livox/livox_mid360_calibration.yaml"
    )
    
    # Launch arguments
    config_path_arg = DeclareLaunchArgument(
        "config_file",
        default_value=config_path,
        description="Path to config file for super_odometry"
    )
    calib_path_arg = DeclareLaunchArgument(
        "calibration_file",
        default_value=calib_path,
    )
    use_imu_odom_arg = DeclareLaunchArgument(
        "use_imu_odom",
        default_value="false",
        description="Use 100Hz IMU odometry instead of 10Hz laser odometry"
    )
    
    # Feature extraction node
    feature_extraction_node = Node(
        package="super_odometry",
        executable="feature_extraction_node",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {"calibration_file": LaunchConfiguration("calibration_file")}
        ],
    )

    # Laser mapping node
    # Note: PROJECT_NAME defaults to empty, so topics are published as:
    #   /laser_odometry (10Hz odometry from scan matching)
    #   /registered_scan (registered point cloud)
    laser_mapping_node = Node(
        package="super_odometry",
        executable="laser_mapping_node",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {"calibration_file": LaunchConfiguration("calibration_file")}
        ],
    )

    # IMU preintegration node (for future 100Hz odometry)
    # Currently the laser_odometry is remapped to state_estimation
    # When ready to use 100Hz IMU odom, remap /super_odometry/state_estimation instead
    imu_preintegration_node = Node(
        package="super_odometry",
        executable="imu_preintegration_node",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {"calibration_file": LaunchConfiguration("calibration_file")}
        ],
    )

    # Static transform: map -> camera_init (for compatibility with existing TF tree)
    tf_map_to_camera_init = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_camera_init",
        arguments=["0", "0", "0", "0", "0", "0", "map", "camera_init"]
    )

    # Static transform: sensor -> aft_mapped (for compatibility with existing TF tree)
    tf_sensor_to_aft_mapped = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="sensor_to_aft_mapped",
        arguments=["0", "0", "0", "0", "0", "0", "sensor", "aft_mapped"]
    )
    
    # Odometry to TF broadcaster (for laser-only mode without IMU)
    odom_to_tf_node = Node(
        package="super_odometry",
        executable="odom_to_tf.py",
        name="odom_to_tf",
        output="screen",
        parameters=[{
            "odom_topic": "/laser_odometry",
            "parent_frame": "map",
            "child_frame": "sensor",
        }]
    )
    
    return LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=False),
        config_path_arg,
        calib_path_arg,
        use_imu_odom_arg,
        feature_extraction_node,
        laser_mapping_node,
        imu_preintegration_node,
        tf_map_to_camera_init,
        tf_sensor_to_aft_mapped,
        odom_to_tf_node,
    ])
