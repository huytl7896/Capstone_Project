from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        Node(
            package='my_package',
            executable='yolov8_ros2_pt.py',
            output='screen'
        ),
        # Node(
        #     package='my_package',
        #     executable='lidar_estimation_node.py',
        #     output='screen'
        # ),        
    ])
