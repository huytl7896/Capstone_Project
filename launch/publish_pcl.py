import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import open3d as o3d
import sensor_msgs_py.point_cloud2 as pc2

class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('pointcloud_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, '/rtabmap/point_cloud', 10)

        # Load point cloud (PCD or PLY)
        pcd = o3d.io.read_point_cloud('/home/huy/dev_ws/src/my_package/Rtabmap/3rd/voxelized.pcd')  # ← sửa đường dẫn ở đây

        # Convert to numpy
        points = np.asarray(pcd.points)

        if pcd.has_colors():
            colors = np.asarray(pcd.colors)  # RGB in [0, 1]
            rgb_floats = np.floor(colors * 255).astype(np.uint8)
            rgb_packed = (rgb_floats[:, 0].astype(np.uint32) << 16) | \
                         (rgb_floats[:, 1].astype(np.uint32) << 8) | \
                         (rgb_floats[:, 2].astype(np.uint32))
            rgb_packed = rgb_packed.view(np.float32)  # convert to float32
            cloud_data = np.column_stack((points, rgb_packed))

            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
        else:
            cloud_data = points
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]

        header = Header()
        header.frame_id = 'map'
        self.cloud_msg = pc2.create_cloud(header, fields, cloud_data)

        self.timer = self.create_timer(1.0, self.publish_cloud)

    def publish_cloud(self):
        self.cloud_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
