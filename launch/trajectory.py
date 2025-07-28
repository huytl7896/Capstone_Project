import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
from transforms3d.euler import quat2euler  # Chuyển quaternion sang euler
from my_package.msg import Yolov8Inference  # Import message YOLO

import tf2_ros
from rclpy.duration import Duration
from rclpy.time import Time
from matplotlib.transforms import Affine2D

class RobotPlot(Node):
    def __init__(self):
        super().__init__('robot_plot')
        self.subscription_odom = self.create_subscription(Odometry, 'diff_cont/odom', self.odom_callback, 10)
        self.subscription_lidar = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.subscription_yolo = self.create_subscription(Yolov8Inference, '/Yolov8_Inference', self.yolo_callback, 10)

        # Khởi tạo tf2 buffer và listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Lưu dữ liệu
        self.x_data = []
        self.y_data = []
        self.lidar_points_map = []        # Các điểm LiDAR chuyển sang map frame
        self.yolo_detections = []         # Danh sách kết quả YOLO mới nhất

        self.robot_width = 0.4  # Chiều rộng xe (m)
        self.robot_length = 0.3  # Chiều dài xe (m)
        self.yaw = 0.0         # Góc quay của xe (radian)
        self.x = 0.0           # Tọa độ xe trong map frame
        self.y = 0.0

        # Tạo figure với map frame cố định (không cập nhật giới hạn theo xe)
        self.fig, self.ax = plt.subplots()
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.set_xlim(-8, 8)
        self.ax.set_ylim(-8, 8)

    def odom_callback(self, msg):
        """ Nhận dữ liệu từ Odometry và chuyển sang map frame """
        try:
            trans = self.tf_buffer.lookup_transform('map', msg.header.frame_id, msg.header.stamp, Duration(seconds=1.0))
        except Exception as e:
            self.get_logger().error("Could not transform odom using msg.header.stamp: " + str(e))
            try:
                trans = self.tf_buffer.lookup_transform('map', msg.header.frame_id, Time(), Duration(seconds=1.0))
            except Exception as e2:
                self.get_logger().error("Fallback transform failed: " + str(e2))
                return

        self.x = msg.pose.pose.position.x + trans.transform.translation.x
        self.y = msg.pose.pose.position.y + trans.transform.translation.y

        q = msg.pose.pose.orientation
        _, _, self.yaw = quat2euler([q.w, q.x, q.y, q.z])

        self.x_data.append(self.x)
        self.y_data.append(self.y)

        self.plot_map()

    def lidar_callback(self, msg):
        """ Nhận dữ liệu từ LiDAR và chuyển sang map frame """
        if not self.x_data:
            return  # Nếu chưa có dữ liệu odometry, không chuyển đổi

        # Luôn dùng thời gian hiện tại để lookup transform
        try:
            trans = self.tf_buffer.lookup_transform('map', msg.header.frame_id, Time(), Duration(seconds=1.0))
        except Exception as e:
            self.get_logger().error("Could not transform LiDAR: " + str(e))
            return

        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        # **Sửa: KHÔNG cộng self.yaw** để giữ map frame cố định.
        self.lidar_points_map = [
            (r * np.cos(a) + self.x, r * np.sin(a) + self.y)
            for r, a in zip(ranges, angles) if msg.range_min < r < msg.range_max
        ]

    def yolo_callback(self, msg: Yolov8Inference):
        """ Nhận kết quả từ YOLO """
        self.yolo_detections = msg.yolov8_inference

    def plot_map(self):
        """ Vẽ bản đồ trong map frame cố định """
        self.ax.clear()
        self.ax.set_aspect('equal', adjustable='box')
        # Giữ giới hạn cố định
        self.ax.set_xlim(-8, 8)
        self.ax.set_ylim(-8, 8)

        # Vẽ quỹ đạo xe (trong map frame)
        self.ax.plot(self.x_data, self.y_data, 'b-', label="Trajectory")

        # Vẽ xe với hình chữ nhật XOAY theo yaw (để hiển thị hướng của xe)
        car = plt.Rectangle((self.x - self.robot_length/2, self.y - self.robot_width/2),
                            self.robot_length, self.robot_width,
                            color='blue', alpha=0.8)
        # Áp dụng phép xoay quanh tâm xe (self.x, self.y)
        tform = Affine2D().rotate_around(self.x, self.y, np.degrees(self.yaw)) + self.ax.transData
        car.set_transform(tform)
        self.ax.add_patch(car)

        # Vẽ mũi tên chỉ hướng của xe
        arrow_length = 0.5
        self.ax.arrow(self.x, self.y, arrow_length * np.cos(self.yaw), arrow_length * np.sin(self.yaw),
                      head_width=0.1, head_length=0.1, fc='green', ec='green', label="Heading")

        # Vẽ các điểm LiDAR (map frame)
        if self.lidar_points_map:
            lidar_x, lidar_y = zip(*self.lidar_points_map)
            self.ax.scatter(lidar_x, lidar_y, c='r', s=5, label="LiDAR Points")

        # Gắn nhãn YOLO vào các điểm LiDAR phù hợp với vật detect
        tol_distance = 0.3  # Tolerance (m)
        for detection in self.yolo_detections:
            label = detection.class_name
            distance = detection.distance
            # Tọa độ dự kiến của vật theo detection (không sử dụng self.yaw)
            object_x = self.x + distance  # Giả sử vật nằm phía trước theo trục X của map frame
            object_y = self.y
            if self.lidar_points_map:
                nearest_point = min(
                    self.lidar_points_map,
                    key=lambda p: np.linalg.norm(np.array(p) - np.array([object_x, object_y])),
                    default=None
                )
                if nearest_point and abs(np.linalg.norm(np.array(nearest_point) - np.array([self.x, self.y])) - distance) < tol_distance:
                    pos_x, pos_y = nearest_point
                else:
                    pos_x, pos_y = object_x, object_y
            else:
                pos_x, pos_y = object_x, object_y

            self.ax.text(pos_x, pos_y, f"{label} ({distance:.2f}m)", fontsize=10, color='green',
                         bbox=dict(facecolor='yellow', alpha=0.5))

        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_title("Fixed Map Frame (Map Does Not Rotate)")
        self.ax.legend()
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = RobotPlot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
