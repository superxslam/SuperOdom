#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    lidar_in = LaunchConfiguration('lidar_in', default='/livox/lidar')
    imu_in = LaunchConfiguration('imu_in', default='/livox/imu')
    lidar_out = LaunchConfiguration('lidar_out', default='/livox_flipped/lidar')
    lidar_out_pc2 = LaunchConfiguration('lidar_out_pc2', default='')
    imu_out = LaunchConfiguration('imu_out', default='/livox_flipped/imu')
    roll_deg = LaunchConfiguration('roll_deg', default='180.0')
    pitch_deg = LaunchConfiguration('pitch_deg', default='0.0')
    yaw_deg = LaunchConfiguration('yaw_deg', default='0.0')

    return LaunchDescription([
        DeclareLaunchArgument('lidar_in', default_value=lidar_in, description='Input LiDAR topic'),
        DeclareLaunchArgument('imu_in', default_value=imu_in, description='Input IMU topic'),
        DeclareLaunchArgument('lidar_out', default_value=lidar_out, description='Output LiDAR topic'),
        DeclareLaunchArgument('imu_out', default_value=imu_out, description='Output IMU topic'),
        DeclareLaunchArgument('roll_deg', default_value=roll_deg, description='Roll rotation in degrees'),
        DeclareLaunchArgument('pitch_deg', default_value=pitch_deg, description='Pitch rotation in degrees'),
        DeclareLaunchArgument('yaw_deg', default_value=yaw_deg, description='Yaw rotation in degrees'),

        Node(
            package='sensor_flip',
            executable='flip_node',
            name='sensor_flip_node',
            output='screen',
            parameters=[{
                'lidar_in': lidar_in,
                'imu_in': imu_in,
                'lidar_out': lidar_out,
                'imu_out': imu_out,
                'roll_deg': roll_deg,
                'pitch_deg': pitch_deg,
                'yaw_deg': yaw_deg,
            }]
        )
    ])
