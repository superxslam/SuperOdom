#!/usr/bin/env python3
"""
ROS2 Lidar Map Builder
======================

This script subscribes to ROS2 point cloud and odometry messages, processes them in real-time,
applies downsampling, and builds a final lidar map.

Features:
- Real-time ROS2 subscription to point cloud and odometry topics
- Voxel grid downsampling for efficient processing
- Outlier filtering for clean maps
- Real-time map building and periodic saving
- Multiple output formats (PLY, PCD, binary)
- Configurable processing parameters
"""

import os
import sys
import numpy as np
import struct
import argparse
import threading
import time
from pathlib import Path
from typing import List, Tuple, Optional
from collections import deque
import json

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

# Point cloud processing
try:
    import open3d as o3d
    OPEN3D_AVAILABLE = True
except ImportError:
    OPEN3D_AVAILABLE = False
    print("Warning: Open3D not available. Some features will be disabled.")

# Visualization
try:
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("Warning: Matplotlib not available. Visualization disabled.")

# Point cloud conversion
try:
    from sensor_msgs_py import point_cloud2
    POINT_CLOUD2_AVAILABLE = True
except ImportError:
    POINT_CLOUD2_AVAILABLE = False
    print("Warning: sensor_msgs_py not available. Using alternative point cloud conversion.")


class ROS2PointCloudProcessor(Node):
    """ROS2 Node for real-time point cloud processing and map building."""
    
    def __init__(self, 
                 voxel_size: float = 0.05,
                 max_points: int = 5000000,
                 save_interval: float = 30.0,
                 output_dir: str = "/home/qb/humanoid_slam/src/mapping_result",
                 cloud_topic: str = "/registered_scan",
                 odom_topic: str = "/laser_odometry"):
        """
        Initialize the ROS2 point cloud processor.
        
        Args:
            voxel_size: Size of voxel for downsampling (meters)
            max_points: Maximum number of points in final map
            save_interval: Interval for saving intermediate maps (seconds)
            output_dir: Directory to save output files
            cloud_topic: ROS2 topic for point cloud data
            odom_topic: ROS2 topic for odometry data
        """
        super().__init__('ros2_lidar_map_builder')
        
        # Processing parameters
        self.voxel_size = voxel_size
        self.max_points = max_points
        self.save_interval = save_interval
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Data storage
        self.final_map = None
        self.scan_count = 0
        self.total_points_processed = 0
        self.last_save_time = time.time()
        self.start_time = time.time()
        
        # Thread safety
        self.map_lock = threading.Lock()
        self.processing_queue = deque(maxlen=100)  # Limit queue size
        
        # Statistics
        self.stats = {
            'scans_received': 0,
            'scans_processed': 0,
            'points_received': 0,
            'points_processed': 0,
            'processing_time': 0.0,
            'last_scan_time': 0.0
        }
        
        # Setup QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Create subscribers
        self.cloud_subscription = self.create_subscription(
            PointCloud2,
            cloud_topic,
            self.cloud_callback,
            qos_profile
        )
        
        self.odom_subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            qos_profile
        )
        
        # Create save command subscriber
        self.save_subscription = self.create_subscription(
            Bool,
            '/save_map',
            self.save_callback,
            10
        )
        
        # Create publisher for current map status
        self.map_status_publisher = self.create_publisher(
            PoseStamped,
            '/map_builder_status',
            10
        )
        
        # Start processing thread
        self.processing_thread = threading.Thread(target=self._processing_loop, daemon=True)
        self.processing_thread.start()
        
        # Start periodic saving timer
        self.save_timer = self.create_timer(save_interval, self._periodic_save)
        
        # Start status publishing timer
        self.status_timer = self.create_timer(5.0, self._publish_status)
        
        self.get_logger().info(f"ROS2 Lidar Map Builder initialized")
        self.get_logger().info(f"Cloud topic: {cloud_topic}")
        self.get_logger().info(f"Odometry topic: {odom_topic}")
        self.get_logger().info(f"Voxel size: {voxel_size}m")
        self.get_logger().info(f"Output directory: {output_dir}")
        
    def cloud_callback(self, msg: PointCloud2):
        """Callback for point cloud messages."""
        self.stats['scans_received'] += 1
        self.stats['last_scan_time'] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Add to processing queue
        if len(self.processing_queue) < self.processing_queue.maxlen:
            self.processing_queue.append(msg)
        else:
            self.get_logger().warn("Processing queue full, dropping scan")
    
    def odom_callback(self, msg: Odometry):
        """Callback for odometry messages."""
        # Store latest pose for reference
        self.latest_pose = msg
    
    def save_callback(self, msg: Bool):
        """Callback for save command."""
        if msg.data:
            self.get_logger().info("Save command received")
            self._save_final_map()
    
    def _processing_loop(self):
        """Main processing loop running in separate thread."""
        while rclpy.ok():
            if self.processing_queue:
                msg = self.processing_queue.popleft()
                start_time = time.time()
                
                try:
                    # Convert ROS2 point cloud to numpy array
                    points = self._ros2_to_numpy(msg)
                    
                    if points is not None and len(points) > 0:
                        # Process the point cloud
                        processed_points = self._process_point_cloud(points)
                        
                        # Add to final map
                        with self.map_lock:
                            self._add_to_final_map(processed_points)
                        
                        # Update statistics
                        self.stats['scans_processed'] += 1
                        self.stats['points_received'] += len(points)
                        self.stats['points_processed'] += len(processed_points)
                        self.stats['processing_time'] = time.time() - start_time
                        
                        self.get_logger().debug(
                            f"Processed scan: {len(points)} -> {len(processed_points)} points, "
                            f"Final map: {len(self.final_map) if self.final_map is not None else 0} points"
                        )
                
                except Exception as e:
                    self.get_logger().error(f"Error processing point cloud: {e}")
            
            time.sleep(0.001)  # Small delay to prevent busy waiting
    
    def _ros2_to_numpy(self, msg: PointCloud2) -> Optional[np.ndarray]:
        """Convert ROS2 PointCloud2 message to numpy array."""
        try:
            if POINT_CLOUD2_AVAILABLE:
                # Use sensor_msgs_py for efficient conversion
                points = []
                for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                    points.append([point[0], point[1], point[2], msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9])
                return np.array(points)
            else:
                # Manual conversion (slower but works without sensor_msgs_py)
                points = []
                point_step = msg.point_step
                data = msg.data
                
                for i in range(0, len(data), point_step):
                    if i + 12 <= len(data):  # x, y, z are float32 (4 bytes each)
                        x = struct.unpack('f', data[i:i+4])[0]
                        y = struct.unpack('f', data[i+4:i+8])[0]
                        z = struct.unpack('f', data[i+8:i+12])[0]
                        
                        # Check for valid points (not NaN or infinite)
                        if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                            points.append([x, y, z, timestamp])
                
                return np.array(points) if points else None
                
        except Exception as e:
            self.get_logger().error(f"Error converting point cloud: {e}")
            return None
    
    def _process_point_cloud(self, points: np.ndarray) -> np.ndarray:
        """Process a single point cloud with downsampling and filtering."""
        if len(points) == 0:
            return points
        
        # Apply voxel downsampling
        downsampled = self._apply_voxel_downsampling(points)
        
        # Filter outliers
        filtered = self._filter_outliers(downsampled)
        
        return filtered
    
    def _apply_voxel_downsampling(self, points: np.ndarray) -> np.ndarray:
        """Apply voxel grid downsampling."""
        if len(points) == 0:
            return points
        
        # Create voxel grid
        voxel_size = self.voxel_size
        min_coords = np.min(points[:, :3], axis=0)
        
        # Calculate voxel indices
        voxel_indices = ((points[:, :3] - min_coords) / voxel_size).astype(int)
        
        # Group points by voxel
        voxel_dict = {}
        for i, voxel_idx in enumerate(voxel_indices):
            voxel_key = tuple(voxel_idx)
            if voxel_key not in voxel_dict:
                voxel_dict[voxel_key] = []
            voxel_dict[voxel_key].append(i)
        
        # Keep one point per voxel (average position, latest timestamp)
        downsampled_points = []
        for voxel_points in voxel_dict.values():
            if len(voxel_points) > 0:
                # Average position
                avg_pos = np.mean(points[voxel_points, :3], axis=0)
                # Latest timestamp
                latest_time = np.max(points[voxel_points, 3])
                downsampled_points.append([avg_pos[0], avg_pos[1], avg_pos[2], latest_time])
        
        return np.array(downsampled_points)
    
    def _filter_outliers(self, points: np.ndarray, nb_neighbors: int = 20, std_ratio: float = 2.0) -> np.ndarray:
        """Remove statistical outliers from point cloud."""
        if len(points) < nb_neighbors:
            return points
        
        try:
            from scipy.spatial import cKDTree
            tree = cKDTree(points[:, :3])
            distances, _ = tree.query(points[:, :3], k=nb_neighbors + 1)
            
            # Remove self-distance (first column)
            distances = distances[:, 1:]
            
            # Calculate mean distance for each point
            mean_distances = np.mean(distances, axis=1)
            
            # Calculate global mean and standard deviation
            global_mean = np.mean(mean_distances)
            global_std = np.std(mean_distances)
            
            # Filter points within acceptable range
            threshold = global_mean + std_ratio * global_std
            mask = mean_distances < threshold
            
            return points[mask]
        except ImportError:
            self.get_logger().warn("scipy not available, skipping outlier filtering")
            return points
    
    def _add_to_final_map(self, new_points: np.ndarray):
        """Add processed points to the final map."""
        if self.final_map is None:
            self.final_map = new_points.copy()
        else:
            # Combine with existing map
            self.final_map = np.vstack([self.final_map, new_points])
            
            # Apply periodic downsampling if map gets too large
            if len(self.final_map) > self.max_points * 1.5:
                self.get_logger().info("Applying periodic downsampling to reduce map size")
                self.final_map = self._apply_voxel_downsampling(self.final_map)
        
        self.scan_count += 1
        self.total_points_processed += len(new_points)
    
    def _periodic_save(self):
        """Periodically save intermediate maps."""
        current_time = time.time()
        if current_time - self.last_save_time >= self.save_interval:
            self._save_intermediate_map()
            self.last_save_time = current_time
    
    def _save_intermediate_map(self):
        """Save intermediate map with timestamp."""
        with self.map_lock:
            if self.final_map is not None and len(self.final_map) > 0:
                timestamp = int(time.time())
                filename = f"intermediate_map_{timestamp}.ply"
                filepath = self.output_dir / filename
                
                if self._save_ply(filepath, self.final_map):
                    self.get_logger().info(f"Saved intermediate map: {filepath}")
    
    def _save_final_map(self):
        """Save final map in multiple formats."""
        with self.map_lock:
            if self.final_map is None or len(self.final_map) == 0:
                self.get_logger().warn("No map to save")
                return
            
            # Apply final downsampling if needed
            final_map = self.final_map.copy()
            if len(final_map) > self.max_points:
                self.get_logger().info("Applying final downsampling")
                original_voxel_size = self.voxel_size
                self.voxel_size = original_voxel_size * 2.0
                final_map = self._apply_voxel_downsampling(final_map)
                self.voxel_size = original_voxel_size
            
            # Save in multiple formats
            timestamp = int(time.time())
            base_name = f"final_map_{timestamp}"
            
            success_count = 0
            
            # Save PLY
            ply_path = self.output_dir / f"{base_name}.ply"
            if self._save_ply(ply_path, final_map):
                success_count += 1
            
            # Save PCD
            pcd_path = self.output_dir / f"{base_name}.pcd"
            if self._save_pcd(pcd_path, final_map):
                success_count += 1
            
            # Save binary
            bin_path = self.output_dir / f"{base_name}.bin"
            if self._save_binary(bin_path, final_map):
                success_count += 1
            
            # Save statistics
            stats_path = self.output_dir / f"{base_name}_stats.json"
            self._save_statistics(stats_path, final_map)
            
            self.get_logger().info(f"Saved final map in {success_count} formats to {self.output_dir}")
    
    def _save_ply(self, filepath: Path, points: np.ndarray) -> bool:
        """Save point cloud in PLY format."""
        try:
            with open(filepath, 'w') as f:
                f.write("ply\n")
                f.write("format ascii 1.0\n")
                f.write(f"element vertex {len(points)}\n")
                f.write("property float x\n")
                f.write("property float y\n")
                f.write("property float z\n")
                f.write("property double timestamp\n")
                f.write("end_header\n")
                
                for point in points:
                    f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} {point[3]:.6f}\n")
            return True
        except Exception as e:
            self.get_logger().error(f"Error saving PLY: {e}")
            return False
    
    def _save_pcd(self, filepath: Path, points: np.ndarray) -> bool:
        """Save point cloud in PCD format."""
        try:
            if OPEN3D_AVAILABLE:
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(points[:, :3])
                o3d.io.write_point_cloud(str(filepath), pcd)
            else:
                # Fallback to text PCD format
                with open(filepath, 'w') as f:
                    f.write("# .PCD v0.7 - Point Cloud Data file format\n")
                    f.write("VERSION 0.7\n")
                    f.write("FIELDS x y z\n")
                    f.write("SIZE 4 4 4\n")
                    f.write("TYPE F F F\n")
                    f.write("COUNT 1 1 1\n")
                    f.write(f"WIDTH {len(points)}\n")
                    f.write("HEIGHT 1\n")
                    f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                    f.write(f"POINTS {len(points)}\n")
                    f.write("DATA ascii\n")
                    
                    for point in points:
                        f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")
            return True
        except Exception as e:
            self.get_logger().error(f"Error saving PCD: {e}")
            return False
    
    def _save_binary(self, filepath: Path, points: np.ndarray) -> bool:
        """Save point cloud in binary format."""
        try:
            with open(filepath, 'wb') as f:
                for point in points:
                    # Write x, y, z (float32) and timestamp (double)
                    f.write(struct.pack('fff', point[0], point[1], point[2]))
                    f.write(struct.pack('d', point[3]))
            return True
        except Exception as e:
            self.get_logger().error(f"Error saving binary: {e}")
            return False
    
    def _save_statistics(self, filepath: Path, points: np.ndarray):
        """Save processing statistics."""
        try:
            stats = {
                'processing_time': time.time() - self.start_time,
                'scans_received': self.stats['scans_received'],
                'scans_processed': self.stats['scans_processed'],
                'points_received': self.stats['points_received'],
                'points_processed': self.stats['points_processed'],
                'final_map_points': len(points),
                'voxel_size': self.voxel_size,
                'max_points': self.max_points,
                'save_interval': self.save_interval,
                'processing_rate': self.stats['scans_processed'] / max(time.time() - self.start_time, 1.0),
                'timestamp': time.time()
            }
            
            with open(filepath, 'w') as f:
                json.dump(stats, f, indent=2)
        except Exception as e:
            self.get_logger().error(f"Error saving statistics: {e}")
    
    def _publish_status(self):
        """Publish current processing status."""
        try:
            status_msg = PoseStamped()
            status_msg.header.stamp = self.get_clock().now().to_msg()
            status_msg.header.frame_id = "map"
            
            # Use pose to encode status information
            status_msg.pose.position.x = float(self.stats['scans_processed'])
            status_msg.pose.position.y = float(self.stats['points_processed'])
            status_msg.pose.position.z = float(len(self.final_map) if self.final_map is not None else 0)
            
            self.map_status_publisher.publish(status_msg)
        except Exception as e:
            self.get_logger().debug(f"Error publishing status: {e}")
    
    def get_statistics(self) -> dict:
        """Get current processing statistics."""
        with self.map_lock:
            return {
                **self.stats,
                'final_map_points': len(self.final_map) if self.final_map is not None else 0,
                'processing_time': time.time() - self.start_time,
                'queue_size': len(self.processing_queue)
            }


def main():
    """Main function."""
    parser = argparse.ArgumentParser(description='ROS2 Lidar Map Builder')
    parser.add_argument('--voxel_size', type=float, default=0.05,
                       help='Voxel size for downsampling (meters)')
    parser.add_argument('--max_points', type=int, default=10000000,
                       help='Maximum number of points in final map')
    parser.add_argument('--save_interval', type=float, default=30.0,
                       help='Interval for saving intermediate maps (seconds)')
    parser.add_argument('--output_dir', type=str, default='/root/superodom_ws/src/SuperOdom/script/mapping_result',
                       help='Directory to save output files')
    parser.add_argument('--cloud_topic', type=str, default='/registered_scan',
                       help='ROS2 topic for point cloud data')
    parser.add_argument('--odom_topic', type=str, default='/laser_odometry',
                       help='ROS2 topic for odometry data')
    parser.add_argument('--log_level', type=str, default='info',
                       choices=['debug', 'info', 'warn', 'error'],
                       help='Log level')
    
    args = parser.parse_args()
    
    # Initialize ROS2
    rclpy.init()
    
    # Set log level
    log_levels = {
        'debug': rclpy.logging.LoggingSeverity.DEBUG,
        'info': rclpy.logging.LoggingSeverity.INFO,
        'warn': rclpy.logging.LoggingSeverity.WARN,
        'error': rclpy.logging.LoggingSeverity.ERROR
    }
    rclpy.logging.set_logger_level('ros2_lidar_map_builder', log_levels[args.log_level])
    
    try:
        # Create and run the processor
        processor = ROS2PointCloudProcessor(
            voxel_size=args.voxel_size,
            max_points=args.max_points,
            save_interval=args.save_interval,
            output_dir=args.output_dir,
            cloud_topic=args.cloud_topic,
            odom_topic=args.odom_topic
        )
        
        print("=== ROS2 Lidar Map Builder ===")
        print(f"Cloud topic: {args.cloud_topic}")
        print(f"Odometry topic: {args.odom_topic}")
        print(f"Voxel size: {args.voxel_size}m")
        print(f"Max points: {args.max_points:,}")
        print(f"Save interval: {args.save_interval}s")
        print(f"Output directory: {args.output_dir}")
        print("\nPress Ctrl+C to stop and save final map...")
        
        # Spin the node
        rclpy.spin(processor)
        
    except KeyboardInterrupt:
        print("\nShutting down...")
        
    finally:
        # Save final map before shutdown
        if 'processor' in locals():
            print("Saving final map...")
            processor._save_final_map()
            
            # Print final statistics
            stats = processor.get_statistics()
            print(f"\n=== Final Statistics ===")
            print(f"Processing time: {stats['processing_time']:.1f}s")
            print(f"Scans received: {stats['scans_received']:,}")
            print(f"Scans processed: {stats['scans_processed']:,}")
            print(f"Points received: {stats['points_received']:,}")
            print(f"Points processed: {stats['points_processed']:,}")
            print(f"Final map points: {stats['final_map_points']:,}")
            print(f"Processing rate: {stats['scans_processed']/max(stats['processing_time'], 1.0):.1f} scans/s")
        
        rclpy.shutdown()


if __name__ == "__main__":
    main()