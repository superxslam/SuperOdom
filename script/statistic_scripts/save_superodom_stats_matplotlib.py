#!/usr/bin/env python3

import os
import time
import argparse
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Dict
from datetime import datetime

# ROS2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

# Import the specific message type
# You'll need to replace this with the actual message path when using the code
from super_odometry_msgs.msg import OptimizationStats, IterationStats

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
        
        # Define axis limits for each field
        self.axis_limits = {
            "laser_cloud_surf_from_map_num": {"y_min": 0, "y_max": None},
            "laser_cloud_corner_from_map_num": {"y_min": 0, "y_max": None},
            "laser_cloud_surf_stack_num": {"y_min": 0, "y_max": None},
            "laser_cloud_corner_stack_num": {"y_min": 0, "y_max": None},
            "total_translation": {"y_min": 0, "y_max": None},
            "total_rotation": {"y_min": 0, "y_max": 6.3},  # 0 to 2π radians
            "translation_from_last": {"y_min": 0, "y_max": 1.0},  # Likely small incremental changes
            "rotation_from_last": {"y_min": 0, "y_max": 0.2},  # Small angular changes
            "time_elapsed": {"y_min": 0, "y_max": None},
            "latency": {"y_min": 0, "y_max": 1000},  # In milliseconds
            "n_iterations": {"y_min": 0, "y_max": 50},  # Typical optimization iteration count
            "average_distance": {"y_min": 0, "y_max": None}
        }
        
        # Define colors for each field
        self.colors = {
            "laser_cloud_surf_from_map_num": "green",
            "laser_cloud_corner_from_map_num": "blue",
            "laser_cloud_surf_stack_num": "gold",
            "laser_cloud_corner_stack_num": "magenta",
            "total_translation": "orange",
            "total_rotation": "purple",
            "translation_from_last": "orangered",
            "rotation_from_last": "indigo",
            "time_elapsed": "teal",
            "latency": "red",
            "n_iterations": "darkgreen",
            "average_distance": "steelblue"
        }
        
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
            
            # Save stats to matplotlib plots
            self.save_stats_to_plots()
            
            # Reset collection for next batch
            self.stats_messages = []
    
    def save_stats_to_plots(self):
        """Save the collected stats messages to separate matplotlib plots for each field"""
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
        
        # Process each field separately
        for field in fields:
            # Collect data for this field
            data_points = []
            has_data = False
            
            for i, msg in enumerate(self.stats_messages):
                timestamp_ms = i * 10  # Dummy timestamp for x-axis
                if hasattr(msg, field):
                    value = getattr(msg, field)
                    data_points.append((timestamp_ms, value))
                    has_data = True
            
            if not has_data:
                self.get_logger().warn(f"No data for field '{field}', skipping")
                continue
            
            # Convert data to arrays for plotting
            timestamps = [t for t, _ in data_points]
            values = [v for _, v in data_points]
            
            # Create a new figure for this field
            plt.figure(figsize=(10, 6))
            
            # Get color for this field
            color = self.colors.get(field, "gray")
            
            # Plot the data as a thin line without markers
            plt.plot(timestamps, values, '-', color=color, linewidth=0.8, alpha=1.0)
            
            # Set titles and labels
            plt.title(f"{field} over time", fontsize=14)
            plt.xlabel("Time (frame number)", fontsize=12)
            plt.ylabel(field, fontsize=12)
            
            # Get axis limits for this field
            limits = self.axis_limits.get(field, {"y_min": None, "y_max": None})
            
            # Set y-axis limits
            y_min = limits["y_min"]
            y_max = limits["y_max"]
            
            # If y_max is None, calculate it from the data
            if y_max is None and values:
                y_max = max(values) * 1.1  # Add 10% margin
            
            plt.ylim(bottom=y_min, top=y_max)
            
            # Add grid for better readability
            plt.grid(True, linestyle='--', alpha=0.7)
            
            # Tight layout for better spacing
            plt.tight_layout()
            
            # Save the figure
            output_file = os.path.join(self.output_dir, f"{field}_{timestamp}.png")
            plt.savefig(output_file, dpi=150)
            plt.close()
            
            self.get_logger().info(f"Field '{field}' plotted and saved to {output_file} with {len(data_points)} data points")
        
        # Create a combined plot with all fields
        self.create_combined_plot(fields, timestamp)
    
    def create_combined_plot(self, fields, timestamp):
        """Create a combined plot with all fields in subplots"""
        n_fields = len(fields)
        
        # Calculate grid dimensions for subplots
        n_cols = min(3, n_fields)
        n_rows = (n_fields + n_cols - 1) // n_cols  # Ceiling division
        
        # Create figure and subplots
        fig, axes = plt.subplots(n_rows, n_cols, figsize=(15, 4 * n_rows))
        
        # Flatten axes array for easier indexing if we have multiple rows
        if n_rows > 1:
            axes = axes.flatten()
        elif n_cols == 1:
            axes = [axes]  # Ensure axes is always iterable
        
        # Plot each field in its own subplot
        for i, field in enumerate(fields):
            # Collect data for this field
            data_points = []
            
            for j, msg in enumerate(self.stats_messages):
                timestamp_ms = j * 10  # Dummy timestamp for x-axis
                if hasattr(msg, field):
                    value = getattr(msg, field)
                    data_points.append((timestamp_ms, value))
            
            if not data_points:
                continue
                
            # Get the current axis
            ax = axes[i] if i < len(axes) else None
            
            if ax is None:
                continue
                
            # Convert data to arrays for plotting
            timestamps = [t for t, _ in data_points]
            values = [v for _, v in data_points]
            
            # Get color for this field
            color = self.colors.get(field, "gray")
            
            # Plot the data with thin lines and no markers
            ax.plot(timestamps, values, '-', color=color, linewidth=0.8, alpha=1.0)
            
            # Set titles and labels
            ax.set_title(field, fontsize=10)
            ax.set_xlabel("Time", fontsize=8)
            ax.set_ylabel("Value", fontsize=8)
            
            # Get axis limits for this field
            limits = self.axis_limits.get(field, {"y_min": None, "y_max": None})
            
            # Set y-axis limits
            y_min = limits["y_min"]
            y_max = limits["y_max"]
            
            # If y_max is None, calculate it from the data
            if y_max is None and values:
                y_max = max(values) * 1.1  # Add 10% margin
            
            ax.set_ylim(bottom=y_min, top=y_max)
            
            # Add grid
            ax.grid(True, linestyle='--', alpha=0.5)
        
        # Hide any unused subplots
        for i in range(len(fields), len(axes)):
            if i < len(axes):
                axes[i].set_visible(False)
        
        # Add overall title
        fig.suptitle("Super Odometry Statistics", fontsize=16)
        
        # Adjust layout
        plt.tight_layout(rect=[0, 0, 1, 0.96])  # Make room for the suptitle
        
        # Save the combined figure
        output_file = os.path.join(self.output_dir, f"combined_stats_{timestamp}.png")
        plt.savefig(output_file, dpi=150)
        plt.close()
        
        self.get_logger().info(f"Combined plot saved to {output_file}")

def main():
    parser = argparse.ArgumentParser(description='Subscribe to ROS2 super_odometry_stats and visualize with matplotlib')
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