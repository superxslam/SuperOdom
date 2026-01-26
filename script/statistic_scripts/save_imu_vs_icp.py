#!/usr/bin/env python3

import os
import time
import argparse
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

# ROS2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

# Import the specific message type
from super_odometry_msgs.msg import OptimizationStats, IterationStats

class OptimizationComparisonPlotter(Node):
    def __init__(self):
        super().__init__('optimization_comparison_plotter')
        
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
        
        self.get_logger().info(f'Listening for messages on /super_odometry_stats, will save comparison plot to {self.output_dir}...')
    
    def stats_callback(self, msg):
        """Callback for receiving odometry stats messages"""
        self.stats_messages.append(msg)
        self.last_message_time = time.time()
        self.get_logger().info(f'Received odometry stats message, total: {len(self.stats_messages)}')
    
    def check_inactivity(self):
        """Check if no messages have been received for 6 seconds"""
        elapsed = time.time() - self.last_message_time
        
        if self.stats_messages and elapsed >= 6.0:
            self.get_logger().info(f'No stats messages received for {elapsed:.1f} seconds - saving comparison plot')
            
            # Create and save the IMU vs ICP optimization plot
            self.create_imu_vs_icp_plot()
            
            # Reset collection for next batch
            self.stats_messages = []
    
    def create_imu_vs_icp_plot(self):
        """Create a plot comparing iterations[0-2].translation_norm with total_translation"""
        if not self.stats_messages:
            self.get_logger().warn("No stats messages to analyze")
            return

        # Generate timestamp for this batch
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Data collections
        total_translations = []  # Main total_translation
        iter0_translations = []  # Initial iteration (IMU prediction)
        iter1_translations = []  # First ICP refinement
        iter2_translations = []  # Second ICP refinement
        timestamps = []         # Frame indices for x-axis
        
        valid_messages = 0
        
        # Collect data from each message
        for msg_idx, msg in enumerate(self.stats_messages):
            # Only include points that have all the required data
            has_total = hasattr(msg, 'total_translation')
            has_iterations = hasattr(msg, 'iterations') and len(msg.iterations) >= 3
            
            if has_total and has_iterations:
                # Record the timestamp (use frame index for simplicity)
                timestamps.append(msg_idx)
                
                # Get total translation
                total_translations.append(msg.total_translation)
                
                # Get iteration-specific translation norms
                iter0_translations.append(msg.iterations[0].translation_norm)
                iter1_translations.append(msg.iterations[1].translation_norm)
                iter2_translations.append(msg.iterations[2].translation_norm)
                
                valid_messages += 1
        
        if valid_messages == 0:
            self.get_logger().warn("No messages with both total_translation and at least 3 iterations found")
            return
            
        # Create the plot
        plt.figure(figsize=(12, 8))
        
        # Plot each data seriess
        plt.plot(timestamps, total_translations, '--', color='blue', linewidth=1.5, markersize=3, 
                 label='IMU Preintegrated Translation (Final Result)')
        plt.plot(timestamps, iter0_translations, '--', color='green', linewidth=1.0, alpha=0.8,
                 label='ICP (Iteration 0)')
        plt.plot(timestamps, iter1_translations, '--', color='orange', linewidth=1.0, alpha=0.8,
                 label='ICP (Iteration 1)')
        plt.plot(timestamps, iter2_translations, '--', color='red', linewidth=1.0, alpha=0.8, 
                 label='ICP (Iteration 2)')
        
        # Add titles and labels
        plt.title('IMU/Odometry Prediction vs. ICP Optimization Comparison', fontsize=16)
        plt.xlabel('Frame Number', fontsize=14)
        plt.ylabel('Translation (meters)', fontsize=14)
        
        # Add legend with clear description
        plt.legend(loc='upper left', fontsize=12)
        
        # Add annotations explaining the plot
        plt.figtext(0.5, 0.01, 
                   "This plot compares the initial odometry/IMU-based prediction (Iteration 0) with\n"
                   "the subsequent ICP optimization steps (Iterations 1-2) and the final result (Total Translation).",
                   ha='center', fontsize=10, bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.3))
        
        # Add grid for readability
        plt.grid(True, linestyle='--', alpha=0.7)
        
        # Adjust y-axis to start from 0
        plt.ylim(bottom=0)
        
        # Tight layout for better spacing
        plt.tight_layout(rect=[0, 0.03, 1, 0.97])
        
        # Save the figure
        output_file = os.path.join(self.output_dir, f"imu_odometry_vs_icp_optimization_{timestamp}.png")
        plt.savefig(output_file, dpi=150)
        plt.close()
        
        # Also save a specific version with the exact requested filename
        fixed_output_file = os.path.join(self.output_dir, "imu_odometry_vs_icp_optimization.png")
        plt.figure(figsize=(12, 8))
        
        # Plot the data again for the fixed filename
        plt.plot(timestamps, total_translations, '--', color='blue', linewidth=1.5, markersize=3, 
                 label='IMU Preintegrated Translation (Final Result)')
        plt.plot(timestamps, iter0_translations, '--', color='green', linewidth=1.0, alpha=0.8,
                 label='ICP (Iteration 0)')
        plt.plot(timestamps, iter1_translations, '--', color='orange', linewidth=1.0, alpha=0.8,
                 label='ICP (Iteration 1)')
        plt.plot(timestamps, iter2_translations, '--', color='red', linewidth=1.0, alpha=0.8, 
                 label='ICP (Iteration 2)')
        
        # Add titles and labels
        plt.title('IMU/Odometry Prediction vs. ICP Optimization Comparison', fontsize=16)
        plt.xlabel('Frame Number', fontsize=14)
        plt.ylabel('Translation (meters)', fontsize=14)
        
        # Add legend with clear description
        plt.legend(loc='upper left', fontsize=12)
        
        # Add annotations explaining the plot
        plt.figtext(0.5, 0.01, 
                   "This plot compares the initial odometry/IMU-based prediction (Iteration 0) with\n"
                   "the subsequent ICP optimization steps (Iterations 1-2) and the final result (Total Translation).",
                   ha='center', fontsize=10, bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.3))
        
        # Add grid for readability
        plt.grid(True, linestyle='--', alpha=0.7)
        
        # Adjust y-axis to start from 0
        plt.ylim(bottom=0)
        
        # Tight layout for better spacing
        plt.tight_layout(rect=[0, 0.03, 1, 0.97])
        
        plt.savefig(fixed_output_file, dpi=150)
        plt.close()
        
        self.get_logger().info(f"Comparison plot saved to {fixed_output_file} with {valid_messages} data points")

def main():
    parser = argparse.ArgumentParser(description='Create comparison plot of IMU/odometry vs ICP optimization')
    args = parser.parse_args()

    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create and use the plotter node
        plotter = OptimizationComparisonPlotter()
        
        # Spin the node to process callbacks
        rclpy.spin(plotter)
    
    except KeyboardInterrupt:
        print("Node stopped by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Clean up and shut down
        rclpy.shutdown()

if __name__ == "__main__":
    main()