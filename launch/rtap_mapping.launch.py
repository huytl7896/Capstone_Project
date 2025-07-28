import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('my_package'), 'config')
    param_file = os.path.join(config_dir, 'my_controller_4WD.yaml')
    rviz_config_dir = os.path.join(config_dir, 'navigation.rviz')

    # Đường dẫn đến launch file của nav2_bringup
    nav2_bringup_launch = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'bringup_launch.py'
    )

    # Đường dẫn đến launch file của RTAB-Map SLAM
    rtab_launch = os.path.join(
        get_package_share_directory('my_package'),
        'launch',
        '3d_depth_mapping_rtab.launch.py'
    )

    return LaunchDescription([
        # Chạy RTAB-Map SLAM với camera Astra

        # Chạy Navigation2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup_launch)
        ),
        # Chạy Rviz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'
        )
    ])
