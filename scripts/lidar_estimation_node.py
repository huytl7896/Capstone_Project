#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarEstimationNode(Node):
    def __init__(self):
        super().__init__('lidar_estimation_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.subscription  # prevent unused variable warning

    def scan_callback(self, msg: LaserScan):
        desired_angle = 0.0
        index = int((desired_angle - msg.angle_min) / msg.angle_increment)
        if 0 <= index < len(msg.ranges):
            distance = msg.ranges[index]
            self.get_logger().info(f"Distance at angle {desired_angle:.2f} rad: {distance:.2f} m")
        else:
            self.get_logger().warn("Desired angle out of range.")

def main(args=None):
    rclpy.init(args=args)
    node = LidarEstimationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
