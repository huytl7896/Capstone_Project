import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
import cv2

class DepthImageNormalizer(Node):
    def __init__(self):
        super().__init__('depth_image_normalizer')
        self.bridge = CvBridge()
        self.subscriber = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_image_callback,
            10
        )
        self.publisher = self.create_publisher(
            Image,
            '/camera/depth/image_raw/mono8',
            10
        )

    def depth_image_callback(self, msg):
        # Chuyển đổi image raw (32FC1) sang OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Tiến hành chuẩn hóa image
        cv_image[np.isnan(cv_image)] = 0  # Replace NaN values with 0
        cv_image = np.clip(cv_image, 0, 8)  # Clip depth values to range [0, 5]
        cv_image_normalized = np.uint8((cv_image / 5) * 255)  # Normalize to 8-bit image

        # Chuyển đổi lại về ROS image message
        img_msg = self.bridge.cv2_to_imgmsg(cv_image_normalized, encoding="mono8")

        # Publish image message
        self.publisher.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DepthImageNormalizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
