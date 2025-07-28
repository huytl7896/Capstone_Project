#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from my_package.msg import InferenceResult
from my_package.msg import Yolov8Inference

bridge = CvBridge()

class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')
        # Load model YOLOv8
        self.model = YOLO('/home/huy/dev_ws/src/my_package/scripts/best.pt')
        self.yolov8_inference = Yolov8Inference()

        # Subscriber cho ảnh RGB
        self.rgb_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10)

        # Subscriber cho ảnh độ sâu (Depth)
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10)

        # Publisher cho kết quả suy luận và ảnh đã đánh dấu
        self.inference_pub = self.create_publisher(Yolov8Inference, '/Yolov8_Inference', 1)
        self.annotated_img_pub = self.create_publisher(Image, '/inference_result', 1)

        # Biến lưu ảnh depth mới nhất (dạng numpy array)
        self.depth_image = None

    def depth_callback(self, depth_msg: Image):
        try:
            # Chuyển đổi ảnh độ sâu từ ROS message sang OpenCV image với encoding 32FC1
            self.depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")
            self.get_logger().info("Depth image updated.")
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")

    def camera_callback(self, rgb_msg: Image):
        # Kiểm tra đã nhận được depth image hay chưa
        if self.depth_image is None:
            self.get_logger().warn("No depth image received yet.")
            return

        try:
            # Chuyển đổi ảnh RGB từ ROS message sang OpenCV image với encoding "bgr8"
            rgb_image = bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert RGB image: {e}")
            return

        # Thực hiện suy luận bằng YOLOv8
        results = self.model(rgb_image)

        # Cập nhật thông tin header cho message suy luận
        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

        # Duyệt qua từng kết quả của suy luận
        for r in results:
            boxes = r.boxes
            for box in boxes:
                inference_result = InferenceResult()
                # Lấy tọa độ của hộp giới hạn (theo định dạng [x_min, y_min, x_max, y_max])
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                c = box.cls  # Lấy chỉ số lớp
                inference_result.class_name = self.model.names[int(c)]
                inference_result.confidence = float(box.conf.item())
                inference_result.left = int(b[0])
                inference_result.top = int(b[1])
                inference_result.right = int(b[2])
                inference_result.bottom = int(b[3])

                # Tính tọa độ trung tâm của hộp giới hạn
                center_x = int((b[0] + b[2]) / 2)
                center_y = int((b[1] + b[3]) / 2)

                # Kiểm tra xem tọa độ trung tâm có nằm trong kích thước của depth_image hay không
                if (0 <= center_x < self.depth_image.shape[1]) and (0 <= center_y < self.depth_image.shape[0]):
                    # Lấy giá trị depth tại pixel trung tâm
                    distance = self.depth_image[center_y, center_x]
                    inference_result.distance = float(distance)
                    self.get_logger().info(f"Object {inference_result.class_name} at ({center_x}, {center_y}) distance: {distance:.2f}")
                else:
                    inference_result.distance = -1.0
                    self.get_logger().warn("Bounding box center out of depth image range.")

                self.yolov8_inference.yolov8_inference.append(inference_result)

        # Tạo ảnh đã được vẽ hộp giới hạn từ kết quả của YOLOv8
        annotated_frame = results[0].plot()
        try:
            annotated_msg = bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert annotated image: {e}")
            return

        # Publish kết quả suy luận và ảnh đã đánh dấu
        self.annotated_img_pub.publish(annotated_msg)
        self.inference_pub.publish(self.yolov8_inference)

        # Xoá danh sách kết quả để chuẩn bị cho lần suy luận kế tiếp
        self.yolov8_inference.yolov8_inference.clear()

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

