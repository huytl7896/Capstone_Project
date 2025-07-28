# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# from deep_sort_realtime.deepsort_tracker import DeepSort
# import onnxruntime as ort
# import time
# import os

# # Config values
# conf_threshold = 0.2
# iou_threshold = 0.3
# model_path = "/home/binhnguyenduc/datn/src/object_detection_pkg/Model/yolo11l.onnx"

# # Initialize DeepSort
# tracker = DeepSort(
#     max_age=5,
#     n_init=5,
#     nn_budget=100,
#     max_cosine_distance=0.15,
#     max_iou_distance=0.5,
# )

# # COCO class names
# coco_classes = [
#     'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 
#     'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench',
#     'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra',
#     'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
#     'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove',
#     'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup',
#     'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
#     'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
#     'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
#     'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
#     'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier',
#     'toothbrush'
# ]  # Same as in obj_tracking_carvideo.py

# car_class_id = coco_classes.index('person')

# if not os.path.exists(model_path):
#     print(f"ERROR: Model not found at {model_path}")
#     # Try alternative location
#     model_path = "/home/binhnguyenduc/datn/src/object_detection_pkg/object_detection_pkg-0/Model/yolo11l.onnx"
#     if not os.path.exists(model_path):
#         print(f"ERROR: Alternative model path also not found!")

# # Initialize ONNX model
# def initialize_onnx_model(model_path):
#     providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
#     session = ort.InferenceSession(model_path, providers=providers)
#     input_name = session.get_inputs()[0].name
#     input_shape = session.get_inputs()[0].shape
#     output_names = [output.name for output in session.get_outputs()]
#     return session, input_name, input_shape, output_names

# def preprocess_image(image, input_shape):
#     target_size = (input_shape[3], input_shape[2])
#     resized = cv2.resize(image, target_size)
#     normalized = resized.astype(np.float32) / 255.0
#     input_tensor = np.transpose(normalized, (2, 0, 1))
#     input_tensor = np.expand_dims(input_tensor, axis=0)
#     return input_tensor, target_size

# def apply_nms(boxes, scores, iou_threshold):
#     boxes_for_nms = [[x1, y1, x2 - x1, y2 - y1] for (x1, y1, x2, y2) in boxes]
#     indices = cv2.dnn.NMSBoxes(boxes_for_nms, scores, conf_threshold, iou_threshold)
#     return [boxes[i[0]] for i in indices] if len(indices) > 0 else []

# # Modify postprocess_detections function to add debugging
# def postprocess_detections(outputs, original_shape, target_size):
#     predictions = outputs[0][0]
#     detections, boxes, scores, class_ids = [], [], [], []
    
#     print(f"Processing {len(predictions)} predictions")
    
#     scale_x = original_shape[1] / target_size[0]
#     scale_y = original_shape[0] / target_size[1]
    
#     all_detections = []  # Track all classes for debugging
    
#     for detection in predictions:
#         if len(detection) >= 4 + len(coco_classes):
#             x_center, y_center, width, height = detection[:4]
#             class_scores = detection[4:4 + len(coco_classes)]
#             class_id = int(np.argmax(class_scores))
#             max_score = float(class_scores[class_id])
            
#             # Log all confident detections for debugging
#             if max_score > 0.2:  # Lower threshold for debugging
#                 all_detections.append({
#                     'class': coco_classes[class_id],
#                     'confidence': max_score
#                 })
            
#             # Original filtering logic
#             if class_id == car_class_id and max_score > conf_threshold:
#                 x1 = int(max(0, x_center * scale_x - width * scale_x / 2))
#                 y1 = int(max(0, y_center * scale_y - height * scale_y / 2))
#                 x2 = int(min(original_shape[1] - 1, x_center * scale_x + width * scale_x / 2))
#                 y2 = int(min(original_shape[0] - 1, y_center * scale_y + height * scale_y / 2))
#                 boxes.append([x1, y1, x2, y2])
#                 scores.append(max_score)
#                 class_ids.append(class_id)
    
#     print(f"All detections above 0.2: {all_detections}")
    
#     # Rest of the function stays the same
#     nms_boxes = apply_nms(boxes, scores, iou_threshold)
#     for i, box in enumerate(boxes):
#         if box in nms_boxes:
#             detections.append({'bbox': box, 'confidence': scores[i], 'class_id': class_ids[i]})
    
#     print(f"Final person detections: {detections}")
#     return detections

# class ObjectDetector(Node):
#     def __init__(self):
#         super().__init__('object_detector')
#         self.rgb_sub = self.create_subscription(Image, '/camera/rgb/image_raw', self.rgb_callback, 10)
#         self.processed_pub = self.create_publisher(Image, '/object_detection/processed_image', 10)
#         self.bridge = CvBridge()
#         self.session, self.input_name, self.input_shape, self.output_names = initialize_onnx_model(model_path)
#         self.get_logger().info('Object Detector node started')

#     def rgb_callback(self, msg):
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#             input_tensor, target_size = preprocess_image(cv_image, self.input_shape)
#             outputs = self.session.run(self.output_names, {self.input_name: input_tensor})
#             detections = postprocess_detections(outputs, cv_image.shape, target_size)

#             result_image = cv_image.copy()
#             for det in detections:
#                 x1, y1, x2, y2 = det['bbox']
#                 cv2.rectangle(result_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
#                 cv2.putText(result_image, "Car", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

#             self.processed_pub.publish(self.bridge.cv2_to_imgmsg(result_image, encoding='bgr8'))
#             cv2.imshow("Object Detection", result_image)
#             cv2.waitKey(1)
#         except Exception as e:
#             self.get_logger().error(f'Error processing image: {e}')

# def main(args=None):
#     rclpy.init(args=args)
#     node = ObjectDetector()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     cv2.destroyAllWindows()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



# filepath: object_detection_pkg/obj_tracking_ros2.py
# ROS 2 node: phát hiện & theo dõi xe hơi (YOLO + Deep SORT)

import os
import time
import cv2
import numpy as np
import onnxruntime as ort
from deep_sort_realtime.deepsort_tracker import DeepSort

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

# ─────────────────────────────┐
#       1. Cấu hình chung       │
# ─────────────────────────────┘
CONF_THRESHOLD = 0.20        # Ngưỡng tin cậy
IOU_THRESHOLD  = 0.40        # Ngưỡng NMS
MODEL_PATH     = "/home/binhnguyenduc/datn/src/object_detection_pkg/Model/yolo11l.onnx"

# Danh sách lớp COCO
COCO_CLASSES = [
    'person','bicycle','car','motorcycle','airplane','bus','train','truck','boat',
    'traffic light','fire hydrant','stop sign','parking meter','bench','bird','cat',
    'dog','horse','sheep','cow','elephant','bear','zebra','giraffe','backpack',
    'umbrella','handbag','tie','suitcase','frisbee','skis','snowboard','sports ball',
    'kite','baseball bat','baseball glove','skateboard','surfboard','tennis racket',
    'bottle','wine glass','cup','fork','knife','spoon','bowl','banana','apple',
    'sandwich','orange','broccoli','carrot','hot dog','pizza','donut','cake','chair',
    'couch','potted plant','bed','dining table','toilet','tv','laptop','mouse',
    'remote','keyboard','cell phone','microwave','oven','toaster','sink',
    'refrigerator','book','clock','vase','scissors','teddy bear','hair drier',
    'toothbrush'
]
CAR_CLASS_ID = COCO_CLASSES.index('person')

# Deep SORT (tham số giống file video)
# tracker = DeepSort(
#     max_age=10,
#     n_init=5,
#     nn_budget=100,
#     max_cosine_distance=0.25,
#     max_iou_distance=0.6,
# )

# Cấu hình tối ưu cho tracking ổn định
tracker = DeepSort(
    max_age=30,                    # Tăng lên để track tồn tại lâu hơn khi bị che
    n_init=3,                      # Giảm để confirm track nhanh hơn
    nn_budget=220,                 # Tăng feature budget
    max_cosine_distance=0.4,       # Nới lỏng appearance matching
    max_iou_distance=0.7,          # Nới lỏng motion matching
    embedder="mobilenet",          # Sử dụng feature extractor tốt hơn
    half=True,                     # Tối ưu performance
    bgr=True,                      # Đúng với OpenCV BGR format
    embedder_gpu=True,             # Sử dụng GPU cho feature extraction
    embedder_model_name="osnet_x0_25",  # Model appearance features tốt
    polygon=False,
    today=None,
)


# Màu vẽ (cố định để reproducible)
np.random.seed(42)
TRACK_COLORS = np.random.randint(0, 255, size=(100, 3))

# ─────────────────────────────┐
#   2. Hàm khởi tạo & tiền xử  │
# ─────────────────────────────┘
def initialize_onnx_model(path: str):
    """Tạo ONNX InferenceSession với CUDA nếu khả dụng."""
    providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
    if not os.path.exists(path):
        raise FileNotFoundError(f"Model not found: {path}")
    sess      = ort.InferenceSession(path, providers=providers)
    input_nm  = sess.get_inputs()[0].name
    inp_shape = sess.get_inputs()[0].shape        # [N,C,H,W]
    out_names = [o.name for o in sess.get_outputs()]
    return sess, input_nm, inp_shape, out_names

def preprocess_image(img: np.ndarray, inp_shape):
    """Resize, chuẩn hoá, chuyển HWC→CHW & thêm batch dim."""
    target = (inp_shape[3], inp_shape[2])         # (W,H)
    resized = cv2.resize(img, target)
    normalized = resized.astype(np.float32) / 255.0
    tensor = np.transpose(normalized, (2, 0, 1))
    tensor = np.expand_dims(tensor, 0)
    return tensor, target

def apply_nms(boxes, scores, iou_thr):
    boxes_nms = [[x1, y1, x2-x1, y2-y1] for (x1,y1,x2,y2) in boxes]
    idxs = cv2.dnn.NMSBoxes(boxes_nms, scores, CONF_THRESHOLD, iou_thr)
    if len(idxs) == 0:
        return []
    # Tương thích OpenCV các phiên bản
    return [boxes[i[0]] if isinstance(idxs[0], (list,np.ndarray)) else boxes[i]
            for i in idxs]

# ─────────────────────────────┐
#       3. Hậu xử lý YOLO       │
# ─────────────────────────────┘
def postprocess_detections(outputs, orig_shape, target_size):
    """Chuyển output YOLO→list dict, chỉ giữ xe hơi và áp NMS."""
    preds = outputs[0][0]                         # (N, 4+80)
    if preds.shape[0] < preds.shape[1]:
        preds = preds.T                           # Phù hợp một số mô hình
    boxes, scores, class_ids = [], [], []

    scale_x = orig_shape[1] / target_size[0]
    scale_y = orig_shape[0] / target_size[1]

    for det in preds:
        if len(det) < 4 + len(COCO_CLASSES):
            continue

        x_c, y_c, w, h = det[:4]
        class_scores   = det[4:4+len(COCO_CLASSES)]
        cls_id         = int(np.argmax(class_scores))
        score          = float(class_scores[cls_id])

        if cls_id != CAR_CLASS_ID or score < CONF_THRESHOLD:
            continue

        x1 = int(max(0, x_c*scale_x - w*scale_x/2))
        y1 = int(max(0, y_c*scale_y - h*scale_y/2))
        x2 = int(min(orig_shape[1]-1, x_c*scale_x + w*scale_x/2))
        y2 = int(min(orig_shape[0]-1, y_c*scale_y + h*scale_y/2))

        if x2 <= x1 or y2 <= y1:
            continue
        if (x2-x1) < 30 or (y2-y1) < 30:         # Lọc box nhỏ
            continue

        boxes.append([x1, y1, x2, y2])
        scores.append(score)
        class_ids.append(cls_id)

    keep = apply_nms(boxes, scores, IOU_THRESHOLD)
    detections = []
    for i, b in enumerate(boxes):
        if b in keep:
            detections.append({'bbox': b,
                               'confidence': scores[i],
                               'class_id': class_ids[i]})
    return detections

# ─────────────────────────────┐
#      4. Vẽ bounding & ID      │
# ─────────────────────────────┘
def draw_track(frame, track):
    """Vẽ 1 track đã xác nhận, an toàn với mọi kiểu track_id."""
    
    if not track.is_confirmed():
        return False

    # Ép kiểu về int để chắc chắn có thể dùng toán tử %
    try:
        tid = int(track.track_id)
    except Exception:
        # Phòng trường hợp track_id là dạng UUID hay string kỳ lạ
        tid = abs(hash(track.track_id)) % 10_000

    x1, y1, x2, y2 = map(int, track.to_tlbr())
    if x2 <= x1 or y2 <= y1:
        return False

    color = tuple(int(c) for c in TRACK_COLORS[tid % len(TRACK_COLORS)])
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)

    label = f"Person #{tid}"
    (w, _), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
    cv2.rectangle(frame, (x1, y1 - 25), (x1 + w, y1), color, -1)
    cv2.putText(frame, label, (x1, y1 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return True


def filter_overlapping_detections(detections, overlap_threshold=0.3):
    """Lọc detection chồng lấp thông minh hơn, giữ lại detection có confidence cao nhất"""
    if len(detections) <= 1:
        return detections
    
    # Sắp xếp theo confidence giảm dần
    sorted_dets = sorted(detections, key=lambda x: x['confidence'], reverse=True)
    filtered = []
    
    for det in sorted_dets:
        x1, y1, x2, y2 = det['bbox']
        det_area = (x2 - x1) * (y2 - y1)
        
        keep = True
        for kept_det in filtered:
            kx1, ky1, kx2, ky2 = kept_det['bbox']
            
            # Tính toán IoU
            ix1, iy1 = max(x1, kx1), max(y1, ky1)
            ix2, iy2 = min(x2, kx2), min(y2, ky2)
            
            if ix2 > ix1 and iy2 > iy1:
                intersect = (ix2 - ix1) * (iy2 - iy1)
                kept_area = (kx2 - kx1) * (ky2 - ky1)
                union = det_area + kept_area - intersect
                iou = intersect / union
                
                if iou > overlap_threshold:
                    keep = False
                    break
        
        if keep:
            filtered.append(det)
    
    return filtered



# ─────────────────────────────┐
#      5. Lớp ROS 2  Node       │
# ─────────────────────────────┘
class ObjectTrackingNode(Node):
    """Node ROS 2 nhận ảnh, phát hiện & theo dõi xe, xuất ảnh đã vẽ."""
    def __init__(self):
        super().__init__("object_tracking_node")

        # Mô hình ONNX
        self.session, self.input_name, self.input_shape, self.output_names = \
            initialize_onnx_model(MODEL_PATH)

        # ROS 2 I/O
        self.sub  = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.image_cb, 10)
        self.pub  = self.create_publisher(
            Image, '/object_tracking/processed_image', 10)
        
        self.det_pub = self.create_publisher(         # NEW ▶
            Detection2DArray,                         # NEW ▶
            '/object_tracking/detections',            # NEW ▶
            10)    

        self.bridge  = CvBridge()

        # FPS counter
        self.start_t   = time.time()
        self.frame_cnt = 0

        self.get_logger().info("Object-Tracking node started ✅")

    # ────────────────────────
    #      Callback ảnh
    # ────────────────────────
    def image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.frame_cnt += 1

        # --- YOLO Inference ---
        tensor, target = preprocess_image(frame, self.input_shape)
        outputs = self.session.run(self.output_names, {self.input_name: tensor})

        detections = postprocess_detections(outputs, frame.shape, target)

        detections = filter_overlapping_detections(detections, overlap_threshold=0.7)

        # --- Chuyển sang format Deep SORT ---
        car_dets = []
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            w, h           = x2-x1, y2-y1
            bbox           = [x1, y1, w, h]
            conf           = det['confidence']

            # Lọc “too close” (để tránh 2 box chồng nhau)
            # too_close = False
            # for b,_,_ in car_dets:
            #     cx1, cy1 = b[0]+b[2]/2, b[1]+b[3]/2
            #     cx2, cy2 = bbox[0]+bbox[2]/2, bbox[1]+bbox[3]/2
            #     if np.hypot(cx1-cx2, cy1-cy2) < 20:
            #         too_close = True
            #         break
            # if too_close:
            #     continue

            car_dets.append((bbox, conf, int(CAR_CLASS_ID)))

        # --- Deep SORT update ---
        tracks = tracker.update_tracks(car_dets, frame=frame)

        # --- Publish detections ---
        # --- Xây dựng Detection2DArray ---------------------------------
        det_arr = Detection2DArray()
        det_arr.header.stamp    = msg.header.stamp
        det_arr.header.frame_id = msg.header.frame_id

        for trk in tracks:
            if not trk.is_confirmed():
                continue

            x1, y1, x2, y2 = map(int, trk.to_tlbr())
            x_c = (x1 + x2) / 2.0
            y_c = (y1 + y2) / 2.0
            w   =  x2 - x1
            h   =  y2 - y1

            det = Detection2D()
            det.id = str(trk.track_id)          # id phải là chuỗi

            # ---- GÁN BOUNDING BOX ------------------------------------
            det.bbox.size_x = float(w)
            det.bbox.size_y = float(h)

            # Replace lines 421-433 with:

            

            # Handle different ROS2 message types correctly
            center = det.bbox.center
            if hasattr(center, "x"):            # Direct x,y attributes
                center.x = float(x_c)
                center.y = float(y_c)
                if hasattr(center, "theta"):
                    center.theta = 0.0
            elif hasattr(center, "position"):   # Position with Point2D
                center.position.x = float(x_c)
                center.position.y = float(y_c)
                # Don't try to set position.z as it doesn't exist
                if hasattr(center, "orientation") and hasattr(center.orientation, "w"):
                    center.orientation.w = 1.0  # quaternion unit if available

            # ---- Nhãn / độ tin cậy -----------------------------------
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = "person"          # đổi "car" nếu theo dõi xe
            #hyp.hypothesis.score    = float(getattr(trk, "det_conf", 1.0))
            confidence = getattr(trk, "det_conf", None)
            hyp.hypothesis.score = float(confidence if confidence is not None else 1.0)
            det.results.append(hyp)

            det_arr.detections.append(det)

        # --------------------------------------------------------------
        self.det_pub.publish(det_arr)

        drawn = sum(draw_track(frame, trk) for trk in tracks)

        # --- Overlay FPS ---
        fps = self.frame_cnt / (time.time() - self.start_t)
        cv2.putText(frame, f"Frame:{self.frame_cnt}  FPS:{fps:.1f}  Tracks:{drawn}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        # --- Publish & (tuỳ chọn) hiển thị ---
        self.pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
        #cv2.imshow("ROS2 Car Tracking", frame)
        #cv2.waitKey(1)

# ─────────────────────────────┐
#             Main             │
# ─────────────────────────────┘
def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
