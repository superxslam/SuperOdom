#!/usr/bin/env python3

import os
import time
import argparse
import numpy as np
from typing import List
from datetime import datetime

# ROS2 imports
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path

# Rerun imports
import rerun as rr
import numpy as np

class PathSubscriber(Node):
    def __init__(self):
        super().__init__('path_subscriber')
        
        # Initialize path storage for laser odometry
        self.laser_path_messages = []
        self.laser_last_message_time = time.time()
        
        # Initialize path storage for IMU odometry
        self.imu_path_messages = []
        self.imu_last_message_time = time.time()
        
        # Global storage for all IMU poses
        self.all_imu_poses = []
        
        # Batch management for IMU poses
        self.imu_batch_count = 0
        self.max_imu_poses = 10000  # Maximum number of poses to store before saving a batch
        
        # Create subscription to the laser path topic
        self.laser_subscription = self.create_subscription(
            Path,
            '/laser_odom_path',
            self.laser_path_callback,
            10  # QoS profile depth
        )
        
        # Create subscription to the IMU path topic
        self.imu_subscription = self.create_subscription(
            Path,
            '/imuodom_path',
            self.imu_path_callback,
            10  # QoS profile depth
        )
        
        # Timer to check for message inactivity
        self.timer = self.create_timer(1.0, self.check_inactivity)
        
        self.get_logger().info('Listening for path messages on /laser_odom_path and /imuodom_path...')
        
    def laser_path_callback(self, msg):
        """Callback for receiving laser Path messages"""
        self.laser_path_messages.append(msg)
        self.laser_last_message_time = time.time()
        self.get_logger().info(f'Received laser path message with {len(msg.poses)} poses')
    
    def imu_path_callback(self, msg):
        """Callback for receiving IMU Path messages"""
        self.imu_path_messages.append(msg)
        self.imu_last_message_time = time.time()
        
        # Add all poses from this message to the global list
        for pose in msg.poses:
            self.all_imu_poses.append(pose)
            
        self.get_logger().info(f'Received IMU path message with {len(msg.poses)} poses, total: {len(self.all_imu_poses)}')
        
        # Check if we need to save a batch due to memory constraints
        if len(self.all_imu_poses) >= self.max_imu_poses:
            self.save_imu_batch()
    
    def save_imu_batch(self):
        """Save the current batch of IMU poses and reset the collection"""
        if not self.all_imu_poses:
            return
            
        self.imu_batch_count += 1
        self.get_logger().info(f'Saving IMU batch #{self.imu_batch_count} with {len(self.all_imu_poses)} poses to prevent memory overflow')
        
        # Save the current batch
        self.save_to_rerun_imu('imu', [255, 0, 0, 255], batch_num=self.imu_batch_count)
        
        # Reset the global list
        self.all_imu_poses = []
    
    def check_inactivity(self):
        """Check if no messages have been received for 6 seconds"""
        laser_elapsed = time.time() - self.laser_last_message_time
        imu_elapsed = time.time() - self.imu_last_message_time
        
        # Check for laser path inactivity
        if self.laser_path_messages and laser_elapsed >= 6.0:
            self.get_logger().info(f'No laser path messages received for {laser_elapsed:.1f} seconds - saving data')
            
            # Save laser path to Rerun and generate pose difference file
            self.save_to_rerun(self.laser_path_messages, 'laser', [0, 255, 255, 255])  # Cyan color
            self.save_pose_difference(self.laser_path_messages, 'laser')
            
            # Reset collection for next batch
            self.laser_path_messages = []
            
        # Check for IMU path inactivity
        if self.imu_path_messages and imu_elapsed >= 6.0:
            self.get_logger().info(f'No IMU path messages received for {imu_elapsed:.1f} seconds - saving data')
            
            # Save IMU path to Rerun and generate pose difference file
            if self.all_imu_poses:
                self.imu_batch_count += 1
                self.save_to_rerun_imu('imu', [255, 0, 0, 255], batch_num=self.imu_batch_count)
            
            self.save_pose_difference(self.imu_path_messages, 'imu')
            
            # Reset collections
            self.imu_path_messages = []
            self.all_imu_poses = []
    
    def save_to_rerun(self, path_messages, path_type, color):
        """Save the collected path messages to a Rerun file with timestamp"""
        # Generate output filename based on current time
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_file = f"{path_type}_path_recording_{timestamp}.rrd"
        
        # Initialize Rerun
        rr.init(f"ROS2 {path_type.upper()} Path Visualization")
        
        # Set up recording to a file
        rr.save(output_file)
        
        # Create a path entity
        path_entity = f"world/{path_type}_odom_path"
        
        # Process each path message with timestamp
        for i, path_msg in enumerate(path_messages):
            # Use a dummy timestamp that increments by 100ms (0.1s) per message
            timestamp_ms = i * 10
            
            # Set the time for this frame
            rr.set_time_sequence("frame", timestamp_ms)
            
            # Extract all positions from the path
            positions = []
            for pose in path_msg.poses:
                position = pose.pose.position
                positions.append([position.x, position.y, position.z])
            
            # Skip empty paths
            if not positions:
                continue
                
            # Log the path points for this timestamp
            rr.log(
                path_entity,
                rr.Points3D(
                    positions=np.array(positions),
                    colors=np.array([color] * len(positions))
                )
            )
        
        self.get_logger().info(f"{path_type.upper()} Rerun recording saved to {output_file}")
        
    def save_to_rerun_imu(self, path_type, color, batch_num=None):
        """Save the current batch of IMU poses"""
        # Generate output filename based on current time
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Include the batch number in the filename if provided
        if batch_num is not None:
            output_file = f"{path_type}_path_recording_batch{batch_num}_{timestamp}.rrd"
        else:
            output_file = f"{path_type}_path_recording_{timestamp}.rrd"
        
        # Initialize Rerun
        rr.init(f"ROS2 {path_type.upper()} Path Visualization" + (f" Batch {batch_num}" if batch_num else ""))
        
        # Set up recording to a file
        rr.save(output_file)
        
        # Create a path entity
        path_entity = f"world/{path_type}_odom_path" + (f"_batch{batch_num}" if batch_num else "")
        
        # Extract all positions from the global list of poses
        positions = []
        for pose in self.all_imu_poses:
            position = pose.pose.position
            positions.append([position.x, position.y, position.z])
        
        # Skip if no positions
        if not positions:
            self.get_logger().warn(f"No valid {path_type} poses to save")
            return
            
        # Log all the path points at once (no timestamps)
        rr.log(
            path_entity,
            rr.Points3D(
                positions=np.array(positions),
                colors=np.array([color] * len(positions))
            )
        )
        
        self.get_logger().info(f"{path_type.upper()} Rerun recording saved to {output_file} with {len(positions)} total poses")

    def save_pose_difference(self, path_messages, path_type):
        """Save first and last pose information to a text file"""
        if not path_messages or not path_messages[-1].poses:
            self.get_logger().warn(f"No valid {path_type} path data to save pose difference")
            return
        
        # Generate output filename based on current time
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_file = f"{path_type}_pose_difference_{timestamp}.txt"
        
        # Get the first and last path message
        latest_path = path_messages[-1]
        
        # Check if there are poses in the path
        if not latest_path.poses:
            self.get_logger().warn(f"{path_type} path contains no poses, skipping pose difference calculation")
            return
        
        # Get first and last pose from the latest path message
        first_pose = latest_path.poses[0].pose
        last_pose = latest_path.poses[-1].pose
        
        # Calculate position difference
        pos_diff_x = last_pose.position.x - first_pose.position.x
        pos_diff_y = last_pose.position.y - first_pose.position.y
        pos_diff_z = last_pose.position.z - first_pose.position.z
        
        # Calculate Euclidean distance
        euclidean_dist = np.sqrt(pos_diff_x**2 + pos_diff_y**2 + pos_diff_z**2)
        
        # Write information to file
        with open(output_file, 'w') as f:
            f.write(f"{path_type.upper()} Path Pose Analysis\n")
            f.write("=" * (len(path_type) + 20) + "\n\n")
            f.write(f"Timestamp: {timestamp}\n\n")
            
            f.write("First Pose:\n")
            f.write(f"  Position: [{first_pose.position.x:.6f}, {first_pose.position.y:.6f}, {first_pose.position.z:.6f}]\n")
            f.write(f"  Orientation: [{first_pose.orientation.x:.6f}, {first_pose.orientation.y:.6f}, {first_pose.orientation.z:.6f}, {first_pose.orientation.w:.6f}]\n\n")
            
            f.write("Last Pose:\n")
            f.write(f"  Position: [{last_pose.position.x:.6f}, {last_pose.position.y:.6f}, {last_pose.position.z:.6f}]\n")
            f.write(f"  Orientation: [{last_pose.orientation.x:.6f}, {last_pose.orientation.y:.6f}, {last_pose.orientation.z:.6f}, {last_pose.orientation.w:.6f}]\n\n")
            
            f.write("Position Difference (Last - First):\n")
            f.write(f"  X: {pos_diff_x:.6f}\n")
            f.write(f"  Y: {pos_diff_y:.6f}\n")
            f.write(f"  Z: {pos_diff_z:.6f}\n\n")
            
            f.write(f"Euclidean Distance: {euclidean_dist:.6f}\n\n")
            
            # Add interpretation of whether the path returns to the original point
            tolerance = 0.1  # 10cm tolerance
            if euclidean_dist < tolerance:
                f.write("RESULT: Path DOES return to the original point (within 10cm tolerance)\n")
            else:
                f.write("RESULT: Path DOES NOT return to the original point (exceeds 10cm tolerance)\n")
        
        self.get_logger().info(f"{path_type.upper()} pose difference saved to {output_file}")

def main():
    parser = argparse.ArgumentParser(description='Subscribe to ROS2 path topics and save to Rerun recordings with pose difference analysis')
    
    # Add optional argument for maximum IMU poses per batch
    parser.add_argument('--max-imu-poses', type=int, default=30000,
                        help='Maximum number of IMU poses to collect before saving a batch (default: 10000)')
    
    args = parser.parse_args()

    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create and use the subscriber node
        path_subscriber = PathSubscriber()
        
        # Set the maximum poses from command line args if provided
        if args.max_imu_poses:
            path_subscriber.max_imu_poses = args.max_imu_poses
            path_subscriber.get_logger().info(f'Maximum IMU poses per batch set to {args.max_imu_poses}')
        
        # Spin the node to process callbacks
        rclpy.spin(path_subscriber)
    
    except KeyboardInterrupt:
        print("Node stopped by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Clean up and shut down
        rclpy.shutdown()

if __name__ == "__main__":
    main()