import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import threading
import time
import math
import numpy as np

class PathTracker(Node):
    def __init__(self):
        super().__init__('path_tracker')

        self.create_subscription(Path, '/local_plan', self.planned_callback, 10)
        self.create_subscription(Odometry, 'diff_cont/odom', self.odom_callback, 10)

        self.planned_path = []
        self.actual_path = []
        self.errors = []

        self.lock = threading.Lock()

    def planned_callback(self, msg):
        self.get_logger().info('Received planned path')
        with self.lock:
            self.planned_path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        with self.lock:
            self.actual_path.append((x, y))
            if self.planned_path:
                min_dist = min(
                    math.hypot(x - px, y - py) for px, py in self.planned_path
                )
                self.errors.append(min_dist)

    def get_paths_and_errors(self):
        with self.lock:
            return self.planned_path.copy(), self.actual_path.copy(), self.errors.copy()


def main(args=None):
    rclpy.init(args=args)
    node = PathTracker()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    plt.ion()
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 10))

    try:
        while rclpy.ok():
            planned, actual, errors = node.get_paths_and_errors()

            if planned or actual:
                ax1.cla()
                ax2.cla()

                if planned:
                    px, py = zip(*planned)
                    ax1.plot(px, py, label='Planned Path', color='blue', linestyle='--')

                if actual:
                    ax, ay = zip(*actual)
                    ax1.plot(ax, ay, label='Actual Path', color='red')

                ax1.set_title('Actual vs Planned Path (Live)')
                ax1.set_xlabel('X')
                ax1.set_ylabel('Y')
                ax1.legend()
                ax1.axis('equal')
                ax1.grid(True)

                if errors:
                    ax2.plot(errors, label='Error (distance)', color='green')
                    ax2.set_title('Path Tracking Error Over Time (Live)')
                    ax2.set_xlabel('Step')
                    ax2.set_ylabel('Error (m)')
                    ax2.grid(True)
                    ax2.legend()

                plt.pause(0.01)

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopping...")

        planned, actual, errors = node.get_paths_and_errors()
        avg_error = np.mean(errors) if errors else 0.0
        print(f"✅ Trung bình sai số: {avg_error:.4f} m")

        # Tắt chế độ tương tác và làm sạch lại hình ảnh
        plt.ioff()
        ax1.cla()
        ax2.cla()

        if planned:
            px, py = zip(*planned)
            ax1.plot(px, py, label='Planned Path', color='blue', linestyle='--')

        if actual:
            ax, ay = zip(*actual)
            ax1.plot(ax, ay, label='Actual Path', color='red')

        ax1.set_title('Actual vs Planned Path (Full)')
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        ax1.legend()
        ax1.axis('equal')
        ax1.grid(True)

        if errors:
            ax2.plot(errors, label=f'Error (mean={avg_error:.3f} m)', color='green')
            ax2.set_title('Path Tracking Error Over Time (Full)')
            ax2.set_xlabel('Step')
            ax2.set_ylabel('Error (m)')
            ax2.grid(True)
            ax2.legend()

        plt.tight_layout()
    
        # 📌 Lưu ảnh
        save_path = '/home/huy/dev_ws/src/my_package/scripts/image1.png'
        plt.savefig(save_path)
        print(f"✅ Đã lưu ảnh tại: {save_path}")
        plt.show()

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()