#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import math
import time

class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_publisher')
        self.publisher_ = self.create_publisher(Point, 'target_pose', 10)
        self.timer = self.create_timer(0.1, self.publish_target)
        self.angle = 0.0

    def publish_target(self):
        msg = Point()
        #msg.header.stamp = self.get_clock().now().to_msg()
        #msg.header.frame_id = 'map'

        # Simple circular trajectory for example
        radius = 2.0
        self.angle += 0.05
        msg.x = radius * math.cos(self.angle)
        msg.y = radius * math.sin(self.angle)
        msg.z = 0.0

        # Orientation set to zero quaternion for simplicity
        #msg.pose.orientation.w = 1.0

        self.publisher_.publish(msg)
        #self.get_logger().info(f'Publishing target at x: {msg.x:.2f}, y: {msg.y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
