"""
SuperOdom launch file for SIMULATION (bag replay).

This launch file runs SuperOdom (Livox Mid360) for bag file replay.
SuperOdom publishes (with PROJECT_NAME=""):
  - /laser_odometry (10Hz odometry from scan matching)
  - /registered_scan (registered point cloud)

Usage: Play a bag file containing /livox/lidar and /livox/imu topics
       ros2 bag play <bag_file> --clock
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
    
    # Feature extraction node
    feature_extraction_node = Node(
        package="super_odometry",
        executable="feature_extraction_node",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {"calibration_file": LaunchConfiguration("calibration_file"),
             "use_sim_time": True}
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
            {"calibration_file": LaunchConfiguration("calibration_file"),
             "use_sim_time": True}
        ],
    )

    # IMU preintegration node (for future 100Hz odometry)
    imu_preintegration_node = Node(
        package="super_odometry",
        executable="imu_preintegration_node",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {"calibration_file": LaunchConfiguration("calibration_file"),
             "use_sim_time": True}
        ],
    )

    # Static transform: map -> camera_init
    tf_map_to_camera_init = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_camera_init",
        arguments=["0", "0", "0", "0", "0", "0", "map", "camera_init"],
        parameters=[{"use_sim_time": True}]
    )

    # Static transform: sensor -> aft_mapped
    tf_sensor_to_aft_mapped = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="sensor_to_aft_mapped",
        arguments=["0", "0", "0", "0", "0", "0", "sensor", "aft_mapped"],
        parameters=[{"use_sim_time": True}]
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
            "use_sim_time": True
        }]
    )
    
    return LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),
        config_path_arg,
        calib_path_arg,
        feature_extraction_node,
        laser_mapping_node,
        imu_preintegration_node,
        tf_map_to_camera_init,
        tf_sensor_to_aft_mapped,
        odom_to_tf_node,
    ])
