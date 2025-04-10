#!/usr/bin/env python3

import os
import time
import argparse
import numpy as np
from typing import List, Dict
from datetime import datetime

# ROS2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

# Import the specific message type
# You'll need to replace this with the actual message path when using the code
from super_odometry_msgs.msg import OptimizationStats, IterationStats

# Rerun imports
import rerun as rr

class OdometryStatsSubscriber(Node):
    def __init__(self):
        super().__init__('odometry_stats_subscriber')
        
        # Initialize storage for stats messages
        self.stats_messages = []
        self.last_message_time = time.time()
        
        # Create subscription to the stats topic
        self.subscription = self.create_subscription(
            OptimizationStats,
            '/super_odometry_stats',
            self.stats_callback,
            10  # QoS profile depth
        )
        
        # Timer to check for message inactivity
        self.timer = self.create_timer(1.0, self.check_inactivity)
        
        # Create the output directory if it doesn't exist
        self.output_dir = "./superodom_stats"
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.get_logger().info(f'Listening for messages on /super_odometry_stats, will save to {self.output_dir}...')
    
    def stats_callback(self, msg):
        """Callback for receiving odometry stats messages"""
        self.stats_messages.append(msg)
        self.last_message_time = time.time()
        self.get_logger().info(f'Received odometry stats message, total: {len(self.stats_messages)}')
    
    def check_inactivity(self):
        """Check if no messages have been received for 6 seconds"""
        elapsed = time.time() - self.last_message_time
        
        if self.stats_messages and elapsed >= 6.0:
            self.get_logger().info(f'No stats messages received for {elapsed:.1f} seconds - saving data')
            
            # Save stats to Rerun
            self.save_stats_to_rerun()
            
            # Reset collection for next batch
            self.stats_messages = []
    
    def save_stats_to_rerun(self):
        """Save the collected stats messages to separate Rerun files for each field"""
        if not self.stats_messages:
            self.get_logger().warn("No stats messages to save")
            return
            
        # Generate timestamp for this batch
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Fields to extract and visualize as requested
        fields = [
            "laser_cloud_surf_from_map_num",
            "laser_cloud_corner_from_map_num",
            "laser_cloud_surf_stack_num",
            "laser_cloud_corner_stack_num",
            "total_translation",
            "total_rotation",
            "translation_from_last",
            "rotation_from_last",
            "time_elapsed",
            "latency",
            "n_iterations",
            "average_distance"
        ]
        
        # Colors for each field's plot
        colors = {
            "laser_cloud_surf_from_map_num": [0, 255, 0],      # Green
            "laser_cloud_corner_from_map_num": [0, 0, 255],    # Blue
            "laser_cloud_surf_stack_num": [255, 255, 0],       # Yellow
            "laser_cloud_corner_stack_num": [255, 0, 255],     # Magenta
            "total_translation": [255, 165, 0],                # Orange
            "total_rotation": [128, 0, 128],                   # Purple
            "translation_from_last": [255, 69, 0],             # Red-Orange
            "rotation_from_last": [75, 0, 130],                # Indigo
            "time_elapsed": [0, 128, 128],                     # Teal
            "latency": [255, 0, 0],                            # Red
            "n_iterations": [0, 128, 0],                       # Dark Green
            "average_distance": [70, 130, 180]                 # Steel Blue
        }
        
        # Process each field separately
        for field in fields:
            # Collect data for this field
            data_points = []
            has_data = False
            
            for i, msg in enumerate(self.stats_messages):
                timestamp_ms = i * 10  # Dummy timestamp
                if hasattr(msg, field):
                    value = getattr(msg, field)
                    data_points.append((timestamp_ms, value))
                    has_data = True
            
            if not has_data:
                self.get_logger().warn(f"No data for field '{field}', skipping")
                continue
            
            # Create a new Rerun recording for this field
            output_file = os.path.join(self.output_dir, f"{field}_{timestamp}.rrd")
            
            # Initialize Rerun for this field
            rr.init(f"Super Odometry Stats - {field}")
            
            # Set up recording to a file
            rr.save(output_file)
            
            # Get color for this field
            color = colors.get(field, [100, 100, 100])  # Default gray if not in colors dict
            
            # Convert data to arrays for plotting
            timestamps = [t for t, _ in data_points]
            values = [v for _, v in data_points]
            
            # Create arrays for points (x, y coordinates)
            positions = np.column_stack((timestamps, values))
            
            # Log points for scatter plot visualization
            rr.log(
                f"plots/{field}",
                rr.Points2D(
                    positions=positions,
                    colors=np.array([color] * len(positions)),
                    radii=np.array([3.0] * len(positions))
                )
            )
            
            # Add annotation context for labels
            # rr.log(
            #     f"plots/{field}",
            #     rr.AnnotationContext(
            #         title=f"{field} over time",
            #         x_axis_name="Time",
            #         y_axis_name=field
            #     )
            # )
            
            # Log individual points with timestamp for time-based viewing
            for i, (t, v) in enumerate(data_points):
                # Set the time sequence for this data point
                rr.set_time_sequence("frame", t)
                
                # Log the scalar value at this time
                rr.log(
                    f"timeseries/{field}",
                    rr.Scalar(v)
                )
            
            self.get_logger().info(f"Field '{field}' recorded to {output_file} with {len(data_points)} data points")
            
            # Close this recording before moving to the next field
            # rr.shutdown()

def main():
    parser = argparse.ArgumentParser(description='Subscribe to ROS2 super_odometry_stats and visualize with Rerun')
    args = parser.parse_args()

    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create and use the subscriber node
        stats_subscriber = OdometryStatsSubscriber()
        
        # Spin the node to process callbacks
        rclpy.spin(stats_subscriber)
    
    except KeyboardInterrupt:
        print("Node stopped by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Clean up and shut down
        rclpy.shutdown()

if __name__ == "__main__":
    main()