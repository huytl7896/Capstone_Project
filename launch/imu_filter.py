#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from scipy.signal import butter, lfilter


class IMUFilter(Node):
    def __init__(self):
        super().__init__('imu_filter')
        self.subscription = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.publisher = self.create_publisher(Imu, '/imu/data_filtered', 10)
        
        # Bộ lọc thông thấp
        self.b, self.a = butter(3, 0.1, btype='low')

    def imu_callback(self, msg):
        # Lọc dữ liệu gia tốc
        msg.linear_acceleration.x = lfilter(self.b, self.a, [msg.linear_acceleration.x])[-1]
        msg.linear_acceleration.y = lfilter(self.b, self.a, [msg.linear_acceleration.y])[-1]
        msg.linear_acceleration.z = lfilter(self.b, self.a, [msg.linear_acceleration.z])[-1]

        # Xuất dữ liệu đã lọc
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = IMUFilter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
