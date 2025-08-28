import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros

def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)

def generate_launch_description():
    config_path = get_share_file(
        package_name="super_odometry",
        file_name="config/livox_mid360_flipped.yaml")
    calib_path = get_share_file(
        package_name="super_odometry",
        file_name="config/livox/livox_mid360_calibration_flipped.yaml"
    )
    
    config_path_arg = DeclareLaunchArgument(
        "config_file",
        default_value=config_path,
        description="Path to config file for super_odometry"
    )
    calib_path_arg = DeclareLaunchArgument(
        "calibration_file",
        default_value=calib_path,
    )

    # Launch the flip node
    flip_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sensor_flip'), 'launch/flip.launch.py')
        ),
        launch_arguments={
            'lidar_in': LaunchConfiguration('lidar_in', default='/livox/lidar'),
            'imu_in': LaunchConfiguration('imu_in', default='/livox/imu'),
            'lidar_out': LaunchConfiguration('lidar_out', default='/livox/lidar_flipped'),
            'imu_out': LaunchConfiguration('imu_out', default='/livox/imu_flipped'),
            'roll_deg': LaunchConfiguration('roll_deg', default='180.0'),
            'pitch_deg': LaunchConfiguration('pitch_deg', default='0.0'),
            'yaw_deg': LaunchConfiguration('yaw_deg', default='0.0'),
        }.items()
    )

    feature_extraction_node = Node(
        package="super_odometry",
        executable="feature_extraction_node",
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        parameters=[LaunchConfiguration("config_file"), {"calibration_file": LaunchConfiguration("calibration_file")}],
    )

    laser_mapping_node = Node(
        package="super_odometry",
        executable="laser_mapping_node",
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        parameters=[LaunchConfiguration("config_file"), {"calibration_file": LaunchConfiguration("calibration_file")}],
        remappings=[
            ("laser_odom_to_init", "integrated_to_init"),
        ]
    )

    imu_preintegration_node = Node(
        package="super_odometry",
        executable="imu_preintegration_node",
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        parameters=[LaunchConfiguration("config_file"), {"calibration_file": LaunchConfiguration("calibration_file")}],
    )

    # Add a topic monitoring node for debugging
    # topic_monitor = Node(
    #     package="rqt_topic",
    #     executable="rqt_topic",
    #     output="screen",
    # )

    return LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value='false'),
        config_path_arg,
        calib_path_arg,
        flip_launch,
        feature_extraction_node,
        laser_mapping_node,
        imu_preintegration_node,
        #topic_monitor,
    ])
