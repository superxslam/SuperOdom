#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import torch
import rerun as rr
import rerun.blueprint as rrb
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs_py import point_cloud2

import cv2
from cv_bridge import CvBridge
import random
from rclpy.time import Duration, Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2, PointField
from rerun_vis import RerunVis
from numpy.lib.recfunctions import structured_to_unstructured
from utils import quaternion_to_rotation_matrix
from super_odometry_msgs.msg import OptimizationStats



# Global configuration variables 
#=================================================
POINTCLOUD_TOPIC = '/registered_scan'
ODOMETRY_TOPIC = '/laser_odometry'
CAMERA_TOPIC = '/camera/image_raw'
BASE_POINT_SIZE = 0.03        # Base point size

# Point cloud downsampling and accumulation parameters
DOWNSAMPLE_FACTOR = 0.01       # Percentage of points to keep (0-1)
MAX_ACCUMULATED_POINTS = float('inf')  # Maximum number of accumulated points (infinity)
ACCUMULATE_FRAMES = True      # Whether to accumulate frames
#=================================================

class RosRerunVisualizer(Node):
    def print_topic_info(self):
        """Print information about the subscribed topics"""
        self.get_logger().info(f"Current topic configuration:")
        self.get_logger().info(f"  PointCloud topic: {POINTCLOUD_TOPIC}")
        self.get_logger().info(f"  Odometry topic:   {ODOMETRY_TOPIC}")
        self.get_logger().info(f"  Camera topic:     {CAMERA_TOPIC}")
        self.get_logger().info(f"  Base point size:  {BASE_POINT_SIZE}")
        self.get_logger().info(f"Point cloud processing parameters:")
        self.get_logger().info(f"  Downsample factor:       {DOWNSAMPLE_FACTOR}")
        self.get_logger().info(f"  Accumulate frames:       {ACCUMULATE_FRAMES}")
        self.get_logger().info(f"  Max accumulated points:  {MAX_ACCUMULATED_POINTS}")

    def __init__(self):
        super().__init__('ros_rerun_visualizer')
        
        intrinsics = torch.tensor([
            [525.0, 0.0, 320.0],
            [0.0, 525.0, 240.0],
            [0.0, 0.0, 1.0]
        ], dtype=torch.float32)
        
        self.visualizer = RerunVis(
            intrinsics_3x3=intrinsics,
            base_point_size=BASE_POINT_SIZE,  
            split_label_vis=True  
        )

        rr.send_blueprint(self.initialize_blueprint())
        
        self.pointcloud_subscription = self.create_subscription(
            PointCloud2,
            POINTCLOUD_TOPIC,  
            self.pointcloud_callback,
            10)
        
        self.odometry_subscription = self.create_subscription(
            Odometry,
            ODOMETRY_TOPIC,  
            self.odometry_callback,
            10)
        
        self.stats_subscription = self.create_subscription(
            OptimizationStats,
            '/super_odometry_stats',
            self.stats_callback,
            10
        )
        
        # Subscribe to the camera image topic
        # self.image_subscription = self.create_subscription(
        #     Image,
        #     CAMERA_TOPIC,  
        #     self.image_callback,
        #     10)
        
        self.get_logger().info(f'ROS2 ReRun Visualizer initialized with topics: '
                              f'PointCloud: {POINTCLOUD_TOPIC}, '
                              f'Odometry: {ODOMETRY_TOPIC}, '
                              f'Camera: {CAMERA_TOPIC}')
        
        self.current_time = 0.001
        self.first_timestamp = None
        self.in_visualization = False
        
        self.path_poses = []
        
        self.bridge = CvBridge()
        
        self.accumulated_points = []
        self.accumulated_colors = []
        self.trajectory_points = []
        
        self.current_pose_matrix = torch.eye(4, dtype=torch.float32)
        self.prev_pose = torch.eye(4, dtype=torch.float32)
    
    def initialize_blueprint(self) -> rr.BlueprintLike:
        return rrb.Blueprint(
            rrb.Horizontal(
                contents=[
                    rrb.Spatial3DView(origin="/map", name="3D World"),
                    rrb.Tabs(contents=[
                        rrb.TimeSeriesView(
                            origin="uncertainty/position",
                            name="Uncertainty Position (X/Y/Z)"
                        ),
                        rrb.TimeSeriesView(
                            origin="uncertainty/orientation",
                            name="Uncertainty Orientation (Roll/Pitch/Yaw)"
                        ),
                    ])
                ]
            )
        )
        
    def downsample_pointcloud(self, points, colors):
        """Downsample pointcloud using random sampling and/or voxel grid.
        
        Args:
            points: Numpy array of [x,y,z] points
            colors: Numpy array of [r,g,b] colors
            
        Returns:
            Downsampled points and colors as numpy arrays
        """
        if len(points) == 0:
            return np.array([]), np.array([])
        
        points_np = np.array(points) if not isinstance(points, np.ndarray) else points
        colors_np = np.array(colors) if not isinstance(colors, np.ndarray) else colors
        
        if DOWNSAMPLE_FACTOR < 1.0:
            num_points = len(points_np)
            num_to_keep = max(1, int(num_points * DOWNSAMPLE_FACTOR))
            indices = random.sample(range(num_points), num_to_keep)
            points_np = points_np[indices]
            colors_np = colors_np[indices]
            
        return points_np, colors_np
    
    def accumulate_pointcloud(self, points, colors):
        """Accumulate points over time and apply limits.
        
        Args:
            points: Numpy array of new points to add
            colors: Numpy array of new colors to add
            
        Returns:
            Accumulated points and colors
        """
        if len(points) > 0:
            points_np = np.array(points) if not isinstance(points, np.ndarray) else points
            homogeneous_points = np.hstack([points_np, np.ones((len(points_np), 1))])
            
            transformed_points = []
            for pt in homogeneous_points:
                pt_global = self.current_pose_matrix @ torch.tensor(pt, dtype=torch.float32)
                transformed_points.append(pt_global[:3].tolist())

            self.accumulated_points.extend(points_np)
            self.accumulated_colors.extend(colors.tolist() if isinstance(colors, np.ndarray) else colors)
            
            # Limit the maximum number of accumulated points
            # if len(self.accumulated_points) > MAX_ACCUMULATED_POINTS:
            #     # Keep only the most recent points
            #     excess = len(self.accumulated_points) - MAX_ACCUMULATED_POINTS
            #     self.accumulated_points = self.accumulated_points[excess:]
            #     self.accumulated_colors = self.accumulated_colors[excess:]
                
        return self.accumulated_points, self.accumulated_colors
    
    def pointcloud_callback(self, points: PointCloud2) -> None:
        """Log a `PointCloud2` with `log_points`."""
        time = Time.from_msg(points.header.stamp)
        rr.set_time_nanos("ros_time", time.nanoseconds)

        pts = point_cloud2.read_points(points, field_names=["x", "y", "z"], skip_nans=True)
        points.fields = [
            PointField(name="r", offset=16, datatype=PointField.UINT8, count=1),
            PointField(name="g", offset=17, datatype=PointField.UINT8, count=1),
            PointField(name="b", offset=18, datatype=PointField.UINT8, count=1),
        ]

        # colors = point_cloud2.read_points(points, field_names=["r", "g", "b"], skip_nans=True)
        pts = structured_to_unstructured(pts)
        # assign color intensity based on z-coordinate
        z_values = pts[:, 2]
        z_min = np.min(z_values)
        z_max = np.max(z_values)
        z_range = z_max - z_min if z_max != z_min else 1.0
        normalized_z = (z_values - z_min) / z_range

        colormap = cv2.applyColorMap((normalized_z * 255).astype(np.uint8), cv2.COLORMAP_JET)
        colors = cv2.cvtColor(colormap, cv2.COLOR_BGR2RGB) / 255.0  

        downsampled_pts, downsampled_colors = self.downsample_pointcloud(pts, colors)
        self.accumulate_pointcloud(downsampled_pts, downsampled_colors)
 
        #Visualize the accumulated pointcloud
        if len(self.accumulated_points) > 0:
            acc_pts = np.array(self.accumulated_points) if not isinstance(self.accumulated_points, np.ndarray) else self.accumulated_points
            acc_colors = np.array(self.accumulated_colors) if not isinstance(self.accumulated_colors, np.ndarray) else self.accumulated_colors
            self.visualizer._log_pc(acc_pts, acc_colors, layer="registered_scan")

    def odometry_callback(self, msg):
        """Callback for Odometry messages."""
        time = Time.from_msg(msg.header.stamp)
        rr.set_time_nanos("ros_time", time.nanoseconds)
        self.path_poses.append(msg.pose.pose)
        
        if self.path_poses:
            current_pose = self.path_poses[-1]
            position = current_pose.position
            orientation = current_pose.orientation
            quat = [orientation.w, orientation.x, orientation.y, orientation.z]
            rot_matrix = quaternion_to_rotation_matrix(quat)
            
            self.current_pose_matrix = torch.eye(4, dtype=torch.float32)
            self.current_pose_matrix[:3, :3] = torch.tensor(rot_matrix, dtype=torch.float32)
            self.current_pose_matrix[0, 3] = position.x
            self.current_pose_matrix[1, 3] = position.y
            self.current_pose_matrix[2, 3] = position.z
        
        self.visualizer._log_pose(self.current_pose_matrix, layer="current_pose")

        current_position = self.current_pose_matrix[:3, 3].tolist()
        self.trajectory_points.append(current_position)
        
        if len(self.trajectory_points) > 1:
            rr.log("map/laser_odom", rr.LineStrips3D(
                strips=[self.trajectory_points],  
                colors=[(255, 0, 0)]  # Red color for the trajectory
            ))
        
        self.prev_pose = self.current_pose_matrix
    
    def stats_callback(self, msg: OptimizationStats):
        """Callback for visualizing optimization uncertainty stats."""
        time = Time.from_msg(msg.header.stamp)
        rr.set_time_nanos("ros_time", time.nanoseconds)

        rr.log("uncertainty/position/x", rr.Scalar(msg.uncertainty_x))
        rr.log("uncertainty/position/y", rr.Scalar(msg.uncertainty_y))
        rr.log("uncertainty/position/z", rr.Scalar(msg.uncertainty_z))

        rr.log("uncertainty/orientation/roll", rr.Scalar(msg.uncertainty_roll))
        rr.log("uncertainty/orientation/pitch", rr.Scalar(msg.uncertainty_pitch))
        rr.log("uncertainty/orientation/yaw", rr.Scalar(msg.uncertainty_yaw))

    
    def image_callback(self, msg):
        """Callback for camera image messages."""
        timestamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if self.first_timestamp is None:
            self.first_timestamp = timestamp_sec
            self.current_time = 0.001  
        else:
            new_time = timestamp_sec - self.first_timestamp
            if new_time <= self.current_time:
                new_time = self.current_time + 0.001  
            self.current_time = new_time
        
        try:
            if not self.in_visualization:
                self.in_visualization = True
                self.visualizer.next_frame(self.current_time)
                
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
                
                rgb_tensor = torch.from_numpy(cv_image).permute(2, 0, 1).contiguous()
                
                self.visualizer._log_rgb_image(rgb_tensor, layer="camera")
                
                self.in_visualization = False
        except Exception as e:
            self.get_logger().error(f"Error in image visualization: {str(e)}")
            self.in_visualization = False
    
def main(args=None):
    rclpy.init(args=args)
    ros_rerun_visualizer = RosRerunVisualizer()
    
    ros_rerun_visualizer.print_topic_info()
    
    try:
        rclpy.spin(ros_rerun_visualizer)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error during execution: {str(e)}")
    finally:
        ros_rerun_visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()