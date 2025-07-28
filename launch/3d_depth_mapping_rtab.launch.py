from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')

    parameters = {
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'use_sim_time': use_sim_time,
        'subscribe_depth': True,  # Nếu dùng LiDAR, có thể tắt Depth
        'subscribe_scan': True,  # Kích hoạt LiDAR
        'subscribe_imu': True,  # Kích hoạt IMU
        'subscribe_odom': True,  # Bật Odom để cải thiện Localization
        'use_action_for_goal': True,


        # Đồng bộ dữ liệu
        'qos_image': qos,
        'qos_scan': qos,
        'qos_imu': qos,
        'sync_queue_size': 2,
        'topic_queue_size': 5,

        # Cấu hình 2D Mapping
        'Reg/Force3DoF': 'true',
        'Optimizer/GravitySigma': '0',
        'Grid/FromDepth': 'false',  # Dùng LiDAR thay vì depth camera
        'Grid/MaxObstacleHeight': '0.2',  # Giảm chiều cao vật cản để tránh lỗi
        'Grid/RayTracing': 'true',  # Dùng ray-tracing để lọc map
        'RGBD/ProximityBySpace': 'true',
        'Odom/Strategy': '1',  # Dùng Visual odometry nếu có camera
        'Odom/GuessMotion': 'true',

        # Loop Closure (Đóng vòng lặp)
        'Mem/LaserScanSave': 'true',  
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ProximityPathMaxNeighbors': '1000',
        'Vis/MinInliers': '15',
        'Mem/STMSize': '100000',
        'Mem/NotLinkedNodesKept': 'true',
        'RGBD/LoopClosureDepth': 0.2,
        'Mem/IncrementalMemory': 'true',

        # Tối ưu ICP cho LiDAR
        'Mem/LaserScanFormat': '1',  # Sử dụng LiDAR 2D
        'Reg/Strategy': '1',  # ICP-based loop closure
        'Reg/Force3DoF': 'true',  # Giữ chế độ 2D
        'Icp/CorrespondenceRatio': '0.2',  # Giữ lại 30% điểm phù hợp
        'Icp/PointToPlane': 'false',  # Không dùng Point-to-Plane
        'Icp/VoxelSize': '0.2',  # Giảm độ phân giải để tránh nhiễu
        'Vis/CorGuessWinSize': '20',
        'RGBD/OptimizeMaxError': '2',

        #Odom
        'Odom/ResetCountdown': '5',  
        'Odom/Strategy': '1',  # Dùng Visual odometry thay vì chỉ encoder
        'Odom/FrameId': 'odom',  
        'Odom/GuessMotion': 'true',  
        'RGBD/NeighborLinkRefining': 'true',  
        'RGBD/ProximityBySpace': 'true',  
        'Vis/InlierDistance': '0.02'

    }

    remappings = [
        ('/odom', '/diff_cont/odom'),
        ('rgb/image', '/camera/image_raw'),
        ('rgb/camera_info', '/camera/camera_info'),
        ('depth/image', '/camera/depth/image_raw'),
        ('scan', '/scan'),  # Thêm LiDAR
        ('imu', '/imu_sensor_controller/imu')  
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('qos', default_value='2', description='QoS used for input sensor topics'),
        DeclareLaunchArgument('localization', default_value='false', description='Launch in localization mode.'),

        # SLAM mode
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']),  # Xóa database cũ để cập nhật bản đồ mới

        # Localization mode
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters, {'Mem/IncrementalMemory': 'False', 'Mem/InitWMWithAllNodes': 'True'}],
            remappings=remappings),

        # Visualization
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[parameters],
            remappings=remappings),
    ])

