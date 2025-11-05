#!/usr/bin/env python3

import os
import csv
import time
import argparse
import numpy as np
from datetime import datetime

# ROS2 imports
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from tf2_ros import Buffer, TransformListener
import rclpy.time

class OdometryIMULogger(Node):
    def __init__(self, output_dir='.'):
        super().__init__('odometry_imu_logger')
        
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Generate timestamp for filenames
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        self.tf_file = os.path.join(output_dir, f'gravity_tf_{timestamp}.csv')
        self.tf_csv = open(self.tf_file, 'w', newline='')
        self.tf_writer = csv.writer(self.tf_csv)
        self.tf_writer.writerow(['timestamp', 'parent', 'child', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw'])

        # TF buffer/listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.tf_parent = 'sensor'
        self.tf_child = 'gravity'
        self.tf_written = False
        self.tf_timer = self.create_timer(0.5, self.try_write_tf)
        
        # CSV file paths
        self.laser_odom_file = os.path.join(output_dir, f'laser_odometry_{timestamp}.csv')
        self.imu_odom_file = os.path.join(output_dir, f'state_estimation_{timestamp}.csv')
        self.imu_data_file = os.path.join(output_dir, f'imu_data_{timestamp}.csv')
        
        # Open CSV files and create writers
        self.laser_odom_csv = open(self.laser_odom_file, 'w', newline='')
        self.imu_odom_csv = open(self.imu_odom_file, 'w', newline='')
        self.imu_data_csv = open(self.imu_data_file, 'w', newline='')
        
        self.laser_writer = csv.writer(self.laser_odom_csv)
        self.imu_odom_writer = csv.writer(self.imu_odom_csv)
        self.imu_data_writer = csv.writer(self.imu_data_csv)
        
        # Write headers
        self.laser_writer.writerow(['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        self.imu_odom_writer.writerow(['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        self.imu_data_writer.writerow(['timestamp', 'ax', 'ay', 'az', 'gx', 'gy', 'gz'])
        
        # Message counters
        self.laser_count = 0
        self.imu_odom_count = 0
        self.imu_data_count = 0
        
        # Last message times for inactivity detection
        self.laser_last_time = time.time()
        self.imu_odom_last_time = time.time()
        self.imu_data_last_time = time.time()
        
        # Create subscriptions
        self.laser_sub = self.create_subscription(
            Odometry,
            '/laser_odometry',
            self.laser_odom_callback,
            10
        )
        
        self.imu_odom_sub = self.create_subscription(
            Odometry,
            '/state_estimation',
            self.imu_odom_callback,
            10
        )
        
        self.imu_data_sub = self.create_subscription(
            Imu,
            '/livox/imu',
            self.imu_data_callback,
            10
        )
        
        # Timer to check for inactivity and print stats
        self.timer = self.create_timer(2.0, self.check_status)
        
        self.get_logger().info(f'Logging started. Output directory: {output_dir}')
        self.get_logger().info(f'  Laser odometry -> {self.laser_odom_file}')
        self.get_logger().info(f'  IMU odometry   -> {self.imu_odom_file}')
        self.get_logger().info(f'  IMU data       -> {self.imu_data_file}')
    
    
    def set_tf_frames(self, parent: str, child: str):
        self.tf_parent = parent
        self.tf_child = child

    def try_write_tf(self):
        if self.tf_written:
            return
        try:
            t = self.tf_buffer.lookup_transform(self.tf_parent, self.tf_child, rclpy.time.Time())
            ts = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
            tr = t.transform.translation
            q = t.transform.rotation
            self.tf_writer.writerow([
                f'{ts:.9f}', self.tf_parent, self.tf_child,
                f'{tr.x:.6f}', f'{tr.y:.6f}', f'{tr.z:.6f}',
                f'{q.x:.6f}', f'{q.y:.6f}', f'{q.z:.6f}', f'{q.w:.6f}'
            ])
            self.tf_csv.flush()
            self.get_logger().info(f'Wrote TF {self.tf_parent}->{self.tf_child} to {self.tf_file}')
            self.tf_written = True
            self.tf_timer.cancel()
        except Exception:
            pass
    
    
    def laser_odom_callback(self, msg):
        """Callback for /laser_odometry"""
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        
        self.laser_writer.writerow([
            f'{timestamp:.9f}',
            f'{pos.x:.6f}',
            f'{pos.y:.6f}',
            f'{pos.z:.6f}',
            f'{ori.x:.6f}',
            f'{ori.y:.6f}',
            f'{ori.z:.6f}',
            f'{ori.w:.6f}'
        ])
        self.laser_odom_csv.flush()
        
        self.laser_count += 1
        self.laser_last_time = time.time()
    
    def imu_odom_callback(self, msg):
        """Callback for /state_estimation"""
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        
        self.imu_odom_writer.writerow([
            f'{timestamp:.9f}',
            f'{pos.x:.6f}',
            f'{pos.y:.6f}',
            f'{pos.z:.6f}',
            f'{ori.x:.6f}',
            f'{ori.y:.6f}',
            f'{ori.z:.6f}',
            f'{ori.w:.6f}'
        ])
        self.imu_odom_csv.flush()
        
        self.imu_odom_count += 1
        self.imu_odom_last_time = time.time()
    
    def imu_data_callback(self, msg):
        """Callback for /livox/imu"""
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        acc = msg.linear_acceleration
        gyro = msg.angular_velocity
        
        self.imu_data_writer.writerow([
            f'{timestamp:.9f}',
            f'{acc.x:.6f}',
            f'{acc.y:.6f}',
            f'{acc.z:.6f}',
            f'{gyro.x:.6f}',
            f'{gyro.y:.6f}',
            f'{gyro.z:.6f}'
        ])
        self.imu_data_csv.flush()
        
        self.imu_data_count += 1
        self.imu_data_last_time = time.time()
    
    def check_status(self):
        """Print status and check for inactivity"""
        laser_elapsed = time.time() - self.laser_last_time
        imu_odom_elapsed = time.time() - self.imu_odom_last_time
        imu_data_elapsed = time.time() - self.imu_data_last_time
        
        self.get_logger().info(
            f'Messages logged - Laser: {self.laser_count}, '
            f'IMU Odom: {self.imu_odom_count}, IMU Data: {self.imu_data_count}'
        )
        
        # Warn if no messages received for 5 seconds
        if self.laser_count > 0 and laser_elapsed > 5.0:
            self.get_logger().warn(f'No laser odometry messages for {laser_elapsed:.1f}s')
        
        if self.imu_odom_count > 0 and imu_odom_elapsed > 5.0:
            self.get_logger().warn(f'No IMU odometry messages for {imu_odom_elapsed:.1f}s')
        
        if self.imu_data_count > 0 and imu_data_elapsed > 5.0:
            self.get_logger().warn(f'No IMU data messages for {imu_data_elapsed:.1f}s')
    
    def cleanup(self):
        """Close all CSV files"""
        self.laser_odom_csv.close()
        self.imu_odom_csv.close()
        self.imu_data_csv.close()
        self.tf_csv.close()
        
        self.get_logger().info('CSV files closed successfully')
        self.get_logger().info(f'Final counts - Laser: {self.laser_count}, '
                              f'IMU Odom: {self.imu_odom_count}, IMU Data: {self.imu_data_count}')

def main():
    parser = argparse.ArgumentParser(
        description='Subscribe to odometry and IMU topics and save to CSV files'
    )
    parser.add_argument(
        '--output-dir', '-o',
        type=str,
        default='.',
        help='Output directory for CSV files (default: current directory)'
    )
    parser.add_argument('--tf-parent', type=str, default='sensor', help='Parent frame (SENSOR_FRAME)')
    parser.add_argument('--tf-child', type=str, default='gravity', help='Child frame (gravity)')
    
    args = parser.parse_args()
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create the logger node
        logger = OdometryIMULogger(output_dir=args.output_dir)
        logger.set_tf_frames(args.tf_parent, args.tf_child)
        
        # Spin the node to process callbacks
        rclpy.spin(logger)
    
    except KeyboardInterrupt:
        print("\nNode stopped by user")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Clean up
        if 'logger' in locals():
            logger.cleanup()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

