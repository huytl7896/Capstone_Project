import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from my_package.msg import Yolov8Inference
from geometry_msgs.msg import PointStamped, PoseStamped, Twist, Point
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import math
import numpy as np
from rclpy.duration import Duration
import tf2_ros
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import time
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import base64
import socketio
import threading


sio = socketio.Client()
ip = "http://127.0.0.1:5000"
ws = "ws://127.0.0.1:5000/ws"

class KalmanFilter:
    def __init__(self):
        self.x = np.zeros((4, 1))  # [x, y, vx, vy]
        self.P = np.eye(4) * 1.0
        self.F = np.array([[1, 0, 1, 0],
                           [0, 1, 0, 1],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]])
        self.R = np.eye(2) * 0.1
        self.Q = np.eye(4) * 0.4
        self.I = np.eye(4)
        self.initialized = False
        self.last_update_time = None

    def predict(self):
        if not self.initialized:
            return 0.0, 0.0, 0.0, 0.0
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x[0, 0], self.x[1, 0], self.x[2, 0], self.x[3, 0]

    def update(self, z, current_time):
        self.initialized = True
        self.last_update_time = current_time
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (self.I - K @ self.H) @ self.P

    def reset(self):
        self.x = np.zeros((4, 1))
        self.P = np.eye(4) * 1.0
        self.initialized = False
        self.last_update_time = None

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        # ... (phần khởi tạo hiện có) ...
        self.kf_dict = {}  # Từ điển lưu bộ lọc Kalman cho mỗi ID phát hiện
        self.detection_ids = {}  # Ánh xạ chỉ số phát hiện sang ID duy nhất

        self.publisher = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Yolov8Inference, '/Yolov8_Inference', self.yolo_callback, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.detections = []
        self.last_yolo_detection = None
        self.last_yolo_time = None
        self.scan_data = None
        self.timer = self.create_timer(0.5, self.timer_callback)  # Giảm tần suất cập nhật
        self.active_marker_ids = set()

        self.image_width = 640
        self.camera_fov_deg = 66.13
        self.camera_frame = 'camera_link'

        self.navigator = BasicNavigator()
        self.last_sent_goal = None

        self.kf = KalmanFilter()
        self.rate = self.create_rate(10)

        self.last_odom_time = None
        self.odom_stuck_count = 0
        self.last_yolo_check = None
        self.yolo_silent_count = 0

        self.bridge = CvBridge()
        self.cv_image = None
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.last_stop_time = None
        self.is_stopping = False
        

        self.latest_lidar_ranges = []  # Lưu dữ liệu LiDAR gần nhất

        # Subscriber để nhận dữ liệu LiDAR từ topic /scan
        self.lidar_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.lidar_callback,
            10
        )

        
        try:
            sio.connect(ip)
            self.get_logger().info("✅ Đã kết nối tới WebSocket Server.")
        except Exception as e:
            self.get_logger().warn(f"❌ Không kết nối được đến WebSocket server: {e}")

    def lidar_callback(self, msg):
        self.latest_lidar_ranges = list(msg.ranges)
        self.lidar_angle_min = msg.angle_min  # rad
        self.lidar_angle_increment = msg.angle_increment  # rad


    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"Lỗi khi chuyển đổi ảnh: {e}")
            self.cv_image = None

    def send_image_async(self, image, label):
        def send_task():
            try:
                start_time = time.time()
                _, img_encoded = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
                if img_encoded is not None:
                    img_base64 = "data:image/jpeg;base64," + base64.b64encode(img_encoded.tobytes()).decode('utf-8')
                    message = {"label": label, "image": img_base64}
                    sio.emit("upload_image", message)
                    elapsed = time.time() - start_time
                    self.get_logger().info(f"✅ Gửi ảnh {label} qua WebSocket, mất {elapsed:.3f}s.")
                else:
                    self.get_logger().warn("❌ Lỗi encode ảnh.")
            except Exception as e:
                self.get_logger().warn(f"❌ Lỗi gửi ảnh WebSocket: {e}")

        thread = threading.Thread(target=send_task, daemon=True)
        thread.start()

    def yolo_callback(self, msg):
        current_time = self.get_clock().now()
        self.last_yolo_check = current_time
        self.yolo_silent_count = 0

        num_objects = len(msg.yolov8_inference)
        self.get_logger().info(f"Nhận {num_objects} object từ YOLO")
        self.detections = []

        detected = False
        for det in msg.yolov8_inference:
            if det.class_name in ["person", "fire"]:
                center_x = (det.left + det.right) / 2.0
                angle_offset = -((center_x - self.image_width / 2.0) / (self.image_width / 2.0)) * (self.camera_fov_deg / 2.0)
                
                det_data = {
                    "label": det.class_name,
                    "distance": det.distance,
                    "center_x": center_x,
                    "confidence": det.confidence,
                    "top": det.top,
                    "left": det.left,
                    "bottom": det.bottom,
                    "right": det.right
                }
                # Chỉ thêm phát hiện "person" nếu confidence > 0.5
                if det.class_name == "person" and det.confidence <= 0.5:
                    self.get_logger().info(f"Bỏ qua person với confidence {det.confidence:.2f} (dưới ngưỡng 0.5)")
                    continue
                self.detections.append(det_data)
                self.last_yolo_detection = det_data
                self.last_yolo_time = current_time

                if det.class_name == "fire":
                    self.get_logger().warn(f"Phát hiện {det.class_name.upper()}: khoảng cách {det.distance:.2f}m, center_x {center_x:.2f}, góc {angle_offset:.1f}°")
                else:
                    self.get_logger().info(f"Phát hiện {det.class_name.upper()}: khoảng cách {det.distance:.2f}m, center_x {center_x:.2f}, góc {angle_offset:.1f}°")

                detected = True

        if not detected:
            self.get_logger().info("Không phát hiện đối tượng nào.")

    def scan_callback(self, msg):
        self.scan_data = msg

    def get_position_from_yolo(self, detection):
        distance = detection["distance"]
        center_x = detection["center_x"]

        if distance <= 0:
            self.get_logger().warn("Khoảng cách YOLO không hợp lệ")
            return None, None

        angle_offset_rad = - ((center_x - self.image_width / 2.0) / (self.image_width / 2.0)) * math.radians(self.camera_fov_deg / 2.0)
        x = distance * math.cos(angle_offset_rad)
        y = distance * math.sin(angle_offset_rad)

        point_cam = PointStamped()
        point_cam.header.stamp = self.get_clock().now().to_msg()
        point_cam.header.frame_id = self.camera_frame
        point_cam.point.x = x
        point_cam.point.y = y
        point_cam.point.z = 0.0
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', self.camera_frame, rclpy.time.Time(), timeout=Duration(seconds=0.5)
            )
            point_map = do_transform_point(point_cam, transform)
            return point_map.point.x, point_map.point.y
        except Exception as e:
            self.get_logger().warn(f"Không thể transform từ {self.camera_frame} sang map: {e}")
            return None, None

    def get_robot_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=0.5)
            )
            translation = transform.transform.translation
            self.get_logger().debug(f"Vị trí robot: x={translation.x:.2f}, y={translation.y:.2f}")
            return translation.x, translation.y
        except Exception as e:
            self.get_logger().warn(f"Không thể lấy pose robot từ TF: {e}")
            return None, None

    def get_robot_orientation(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=0.5)
            )
            rotation = transform.transform.rotation
            _, _, yaw = euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
            self.get_logger().debug(f"Hướng robot: {math.degrees(yaw):.2f}°")
            self.last_odom_time = self.get_clock().now()
            self.odom_stuck_count = 0
            return yaw
        except Exception as e:
            self.get_logger().warn(f"Không thể lấy hướng robot từ TF: {e}")
            return None

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
        
    def navigation_through_goal(self):
        if self.is_stopping:
            return

        if not hasattr(self, 'waypoints'):
            self.waypoints = [
                (-8.6, 0.0, 0.0),
                (-7.5, -3.7, 0),
                (4.73, -3.3, -1.571),
                (22.1542, -4, 1.5648),
                (25.9873, 3.5, 3.06822),
                (15, 3.76, 1.59446),
                (0.1242, 4.1605, -3.13)
            ]
            self.auto_nav_index = 0

        if not self.navigator.isTaskComplete():
            return
        else:
            x, y, yaw = self.waypoints[self.auto_nav_index]
            goal = self.build_goal(x, y, yaw)
            self.get_logger().info(f"[Auto Nav] Đang tới waypoint {self.auto_nav_index}: ({x:.2f}, {y:.2f})")
            self.navigator.goToPose(goal)
            self.last_sent_goal = goal

            self.auto_nav_index += 1
            if self.auto_nav_index >= len(self.waypoints):
                self.auto_nav_index = 0

    def timer_callback(self):
            now = self.get_clock().now()
            current_ids = set()

            person_tracked = None
            high_confidence = False

            # Xử lý gửi ảnh cho các detections
            if self.detections and self.cv_image is not None:
                current_time_sec = time.time()
                self.last_image_send_time = getattr(self, 'last_image_send_time', 0)
                if current_time_sec - self.last_image_send_time >= 1.5:  # Giới hạn 1 ảnh/giây
                    for det in self.detections:
                        if (det["label"] == "person" and det["confidence"] > 0.73) or det["label"] == "fire":
                            top, left, bottom, right = int(det["top"]), int(det["left"]), int(det["bottom"]), int(det["right"])
                            h, w, _ = self.cv_image.shape
                            top = max(0, min(top, h))
                            bottom = max(0, min(bottom, h))
                            left = max(0, min(left, w))
                            right = max(0, min(right, w))

                            crop = self.cv_image[top:bottom, left:right]
                            self.get_logger().info(f"Kích thước ảnh crop: {crop.shape}")
                            start_send_time = time.time()
                            self.send_image_async(crop, det["label"])
                            self.get_logger().info(f"Khởi động gửi ảnh {det['label']} sau {time.time() - start_send_time:.3f}s")
                            self.last_image_send_time = current_time_sec
                            break  # Chỉ gửi 1 ảnh mỗi lần để giảm tải

            # Xử lý bám người và marker
            if self.detections:
                # Lọc các phát hiện "person" với confidence > 0.5
                people_detections = [d for d in self.detections if d.get("label") == "person" and d.get("confidence", 0.0) > 0.5]
                if people_detections:
                    low_conf_people = [d for d in people_detections if d.get("confidence", 0.0) < 0.7]
                    if low_conf_people:
                        person_tracked = min(low_conf_people, key=lambda d: d.get("distance", float("inf")))
                        high_confidence = False
                        world_x, world_y = self.get_position_from_yolo(person_tracked)
                        if world_x is not None and world_y is not None:
                            self.kf.update(np.array([[world_x], [world_y]]), now)
                            self.kf.last_update_time = now
                            self.kf.initialized = True
                    else:
                        high_confidence = True  # Confidence ≥ 0.7, chuyển sang Nav2

                for i, det in enumerate(self.detections):
                    # Chỉ xuất bản marker cho "person" với confidence > 0.5 hoặc "fire"
                    if det["label"] == "person" and det["confidence"] <= 0.5:
                        continue
                    world_x, world_y = self.get_position_from_yolo(det)
                    if world_x is None or world_y is None:
                        continue

                    # Gán ID duy nhất cho mỗi phát hiện để theo dõi
                    det_id = f"{det['label']}_{i}"
                    if det_id not in self.kf_dict:
                        self.kf_dict[det_id] = KalmanFilter()
                        self.detection_ids[i] = det_id

                    # Cập nhật bộ lọc Kalman với vị trí hiện tại
                    kf = self.kf_dict[det_id]
                    kf.update(np.array([[world_x], [world_y]]), now)
                    pred_x, pred_y, _, _ = kf.predict()
                    pred_distance = math.hypot(pred_x, pred_y)
                    if pred_distance < 0.05:
                        self.get_logger().info("❌ Vị trí dự đoán gần gốc tọa độ, bỏ qua.")
                        continue

                    angle_rad = math.atan2(pred_y, pred_x)
                    if math.isnan(angle_rad):
                        self.get_logger().info("❌ Góc bị NaN, bỏ qua.")
                        continue

                    pred_distance = math.hypot(pred_x, pred_y)

                    # Nếu có dữ liệu LiDAR và đủ thông tin góc
                    if (
                        self.latest_lidar_ranges and
                        hasattr(self, "lidar_angle_min") and
                        hasattr(self, "lidar_angle_increment")
                    ):
                        # Tính chỉ số trong mảng ranges tương ứng với góc angle_rad
                        index = int((angle_rad - self.lidar_angle_min) / self.lidar_angle_increment)

                        if 0 <= index < len(self.latest_lidar_ranges):
                            lidar_distance = self.latest_lidar_ranges[index]

                            if math.isinf(lidar_distance):
                                # ✅ Fallback nếu YOLO rất tự tin và KF gần
                                if det['confidence'] > 0.6 and pred_distance < 4.0:
                                    self.get_logger().info(
                                        f"⚠️ Fallback: LiDAR ∞ nhưng YOLO tự tin ({det['confidence']:.2f}), "
                                        f"KF={pred_distance:.2f}m — vẫn xuất marker"
                                    )
                                    self.publish_marker(i, pred_x, pred_y, det)
                                    current_ids.add(i)
                                else:
                                    self.get_logger().info(f"❌ Bỏ qua marker '{det['label']}' vì LiDAR trả về ∞ tại index {index}")
                                continue

                            if abs(pred_distance - lidar_distance) < 1.0:
                                self.get_logger().info(
                                    f"✅ LiDAR xác nhận '{det['label']}' tại góc {math.degrees(angle_rad):.1f}°, "
                                    f"LiDAR={lidar_distance:.2f}m ≈ KF={pred_distance:.2f}m"
                                )
                                self.publish_marker(i, pred_x, pred_y, det)
                                current_ids.add(i)
                            else:
                                self.get_logger().info(
                                    f"❌ Bỏ qua marker '{det['label']}' do LiDAR không xác nhận "
                                    f"(LiDAR={lidar_distance:.2f}, KF={pred_distance:.2f})"
                                )
                        else:
                            self.get_logger().info("❌ Góc ngoài vùng quan sát của LiDAR.")
                    else:
                        # Nếu không có dữ liệu LiDAR hoặc thiếu thông tin, fallback
                        self.get_logger().warn("⚠️ Không có dữ liệu LiDAR hoặc thiếu góc. Xuất marker fallback.")
                        self.publish_marker(i, pred_x, pred_y, det)
                        current_ids.add(i)


            # Dọn dẹp các bộ lọc Kalman cũ
            for det_id in list(self.kf_dict.keys()):
                if det_id not in [self.detection_ids.get(i) for i in current_ids]:
                    del self.kf_dict[det_id]

            # Xử lý logic bám người hoặc chuyển về Nav2
            if person_tracked and person_tracked.get("confidence", 0.0) < 0.7 and person_tracked.get("distance", float("inf")) < 0.5:
                # Tính target_angle dựa trên center_x
                center_x = person_tracked.get("center_x", 0.0)
                angle_offset = -((center_x - self.image_width / 2.0) / (self.image_width / 2.0)) * (self.camera_fov_deg / 2.0)
                target_angle = math.radians(angle_offset)

                # Hàm kiểm tra nếu phát hiện người mới với confidence cao
                def found_person_check_fn():
                    if self.detections:
                        for det in self.detections:
                            if det["label"] == "person" and det["confidence"] >= 0.7 or det["distance"] >= 0.5:
                                return True
                    return False
                
                # Gọi rotate_toward_angle để xoay về hướng người
                self.get_logger().info(f"Bám người với confidence {person_tracked['confidence']:.2f}, xoay về góc {math.degrees(target_angle):.1f}°")
                if not hasattr(self, 'rotate_thread') or not self.rotate_thread.is_alive():
                    self.rotate_thread = threading.Thread(target=self.rotate_toward_angle, args=(target_angle, found_person_check_fn), daemon=True)
                    self.rotate_thread.start()           

                self.auto_nav_enabled = False

            elif high_confidence:
                self.get_logger().info("Phát hiện người với confidence ≥ 0.7 – quay lại hành trình Nav2.")
                self.auto_nav_enabled = True

            else:
                self.get_logger().info("Không phát hiện người < 0.7 – tiếp tục hành trình Nav2.")
                self.auto_nav_enabled = True

            # Kiểm soát Nav2 để tránh gửi goal liên tục
            if self.auto_nav_enabled and hasattr(self, 'last_nav_attempt'):
                if time.time() - self.last_nav_attempt < 2.0:  # Chờ 2 giây trước khi thử goal mới
                    return

            # if self.auto_nav_enabled:
            #     self.navigation_through_goal()
            #     self.last_nav_attempt = time.time()

            self.get_logger().debug("Timer callback kết thúc.")

    # def publish_marker(self, marker_id, x, y, det):
    #     marker_array = MarkerArray()

    #     text_marker = Marker()
    #     text_marker.header = Header()
    #     text_marker.header.stamp = self.get_clock().now().to_msg()
    #     text_marker.header.frame_id = "map"
    #     text_marker.ns = "detections"
    #     text_marker.id = marker_id * 2
    #     text_marker.type = Marker.TEXT_VIEW_FACING
    #     text_marker.action = Marker.ADD
    #     text_marker.pose.position.x = float(x)
    #     text_marker.pose.position.y = float(y)
    #     text_marker.pose.position.z = 0.2
    #     text_marker.scale.z = 0.2
    #     text_marker.text = f"{det['label']}:{det['distance']:.2f}m"
    #     text_marker.color.r = 1.0
    #     text_marker.color.g = 0.0
    #     text_marker.color.b = 0.0
    #     text_marker.color.a = 1.0
    #     text_marker.lifetime = Duration(seconds=2).to_msg()

    #     marker_array.markers.append(text_marker)

    #     circle_marker = Marker()
    #     circle_marker.header = text_marker.header
    #     circle_marker.ns = "detections"
    #     circle_marker.id = marker_id * 2 + 1
    #     circle_marker.type = Marker.LINE_STRIP
    #     circle_marker.action = Marker.ADD
    #     circle_marker.pose.orientation.w = 1.0
    #     circle_marker.scale.x = 0.03
    #     circle_marker.color.r = 0.0
    #     circle_marker.color.g = 0.5
    #     circle_marker.color.b = 1.0
    #     circle_marker.color.a = 1.0
    #     circle_marker.lifetime = Duration(seconds=2).to_msg()
        
    #     radius = 0.6
    #     num_points = 70
    #     for i in range(num_points + 1):
    #         angle = 2 * math.pi * i / num_points
    #         px = x + radius * math.cos(angle)
    #         py = y + radius * math.sin(angle)
    #         p = Point(x=px, y=py, z=0.05)
    #         circle_marker.points.append(p)

    #     marker_array.markers.append(circle_marker)

    #     self.publisher.publish(marker_array)
    #     self.active_marker_ids.add(marker_id)


    def publish_marker(self, marker_id, x, y, det):
        marker_array = MarkerArray()

        # Marker dạng CYLINDER
        cylinder_marker = Marker()
        cylinder_marker.header = Header()
        cylinder_marker.header.stamp = self.get_clock().now().to_msg()
        cylinder_marker.header.frame_id = "map"
        cylinder_marker.ns = "detections"
        cylinder_marker.id = marker_id
        cylinder_marker.type = Marker.CYLINDER
        cylinder_marker.action = Marker.ADD
        cylinder_marker.pose.position.x = float(x)
        cylinder_marker.pose.position.y = float(y)
        cylinder_marker.pose.position.z = 0.0  # Đặt cao hơn mặt đất một chút
        cylinder_marker.pose.orientation.w = 1.0

        # Kích thước trụ
        cylinder_marker.scale.x = 0.6
        cylinder_marker.scale.y = 0.6
        cylinder_marker.scale.z = 1.5

        # 🎨 Màu xanh lá + trong suốt
        cylinder_marker.color.r = 0.0
        cylinder_marker.color.g = 1.0
        cylinder_marker.color.b = 0.0
        cylinder_marker.color.a = 0.4  # Trong suốt, từ 0.0 (hoàn toàn trong) -> 1.0 (đục)

        cylinder_marker.lifetime = Duration(seconds=2).to_msg()

        marker_array.markers.append(cylinder_marker)

        self.publisher.publish(marker_array)
        self.active_marker_ids.add(marker_id)





    def build_goal(self, x, y, yaw=None):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = 0.0
        if yaw is not None:
            q = quaternion_from_euler(0.0, 0.0, yaw)
            goal.pose.orientation.x = q[0]
            goal.pose.orientation.y = q[1]
            goal.pose.orientation.z = q[2]
            goal.pose.orientation.w = q[3]
        else:
            goal.pose.orientation.w = 1.0
        return goal

    def cleanup_old_markers(self, current_ids):
        for old_id in self.active_marker_ids - current_ids:
            for marker_id in [old_id * 2, old_id * 2 + 1]:
                delete_marker = MarkerArray()
                delete_marker.header.stamp = self.get_clock().now().to_msg()
                delete_marker.header.frame_id = "map"
                delete_marker.ns = "detections"
                delete_marker.id = marker_id
                delete_marker.action = Marker.DELETE
                delete_marker.lifetime = Duration(seconds=0).to_msg()
                self.publisher.publish(delete_marker)
        
        self.active_marker_ids = current_ids

    def _distance(self, goal1, goal2):
        dx = goal1.pose.position.x - goal2.pose.position.x
        dy = goal1.pose.position.y - goal2.pose.position.y
        return math.sqrt(dx ** 2 + dy ** 2)
    
    def rotate_toward_angle(self, target_angle, found_person_check_fn=None):
        if not hasattr(self, 'cmd_vel_pub'):
            self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        target_angle = self.normalize_angle(target_angle)
        tolerance = math.radians(5)
        angular_speed = 0.5
        r = self.create_rate(20, self.get_clock())
        max_rotate_duration = 10
        start_time = time.time()
        twist = Twist()
        twist.linear.x = 0.0
        self.get_logger().info(f"Bắt đầu xoay về góc {math.degrees(target_angle):.1f}°")
        yaw_stuck_count = 0
        max_yaw_stuck = 30
        last_yaw = None

        while rclpy.ok():
            if found_person_check_fn and found_person_check_fn():
                self.get_logger().info("YOLO phát hiện lại người – hủy xoay.")
                break
            robot_yaw = self.get_robot_orientation()
            if robot_yaw is None:
                self.get_logger().warn("Không lấy được góc robot khi xoay.")
                r.sleep()
                continue
            if last_yaw is not None and abs(last_yaw - robot_yaw) < 0.01:
                yaw_stuck_count += 1
                self.get_logger().warn(f"Robot không xoay, yaw không đổi ({yaw_stuck_count}/{max_yaw_stuck}).")
                if yaw_stuck_count >= max_yaw_stuck:
                    self.get_logger().error("Yaw kẹt quá lâu – dừng xoay để an toàn.")
                    break
            else:
                yaw_stuck_count = 0
            last_yaw = robot_yaw
            angle_diff = self.normalize_angle(target_angle - robot_yaw)
            self.get_logger().info(f"[Rotate] robot_yaw: {math.degrees(robot_yaw):.1f}°, target: {math.degrees(target_angle):.1f}°, diff: {math.degrees(angle_diff):.1f}°")
            if abs(angle_diff) < tolerance:
                self.get_logger().info("Đạt được góc mục tiêu.")
                break
            twist.angular.z = angular_speed if angle_diff > 0 else -angular_speed
            self.cmd_vel_pub.publish(twist)
            r.sleep()
            if time.time() - start_time > max_rotate_duration:
                self.get_logger().warn("Xoay quá lâu – dừng để tránh kẹt.")
                break
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Hoàn thành xoay.")

def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()