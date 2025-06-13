#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomToTum(Node):
    def __init__(self):
        super().__init__('odom_to_tum')
        self.declare_parameter('odom_topic', '/state_estimation')
        self.declare_parameter('output_file', 'odometry.tum')
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value

        self.file = open(self.output_file, 'a')
        self.get_logger().info(f'Writing TUM file to: {self.output_file}')
        self.sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

    def odom_callback(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        line = f"{t:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} {q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n"
        self.file.write(line)
        self.file.flush()

    def destroy_node(self):
        self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTum()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
