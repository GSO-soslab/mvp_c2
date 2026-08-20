import os
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


# from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    traffic_setting_file = os.path.join(get_package_share_directory('mvp_c2'), 'config', 'commander_traffic.yaml') 

    return LaunchDescription([
    

        Node(
            package='mvp_c2',
            namespace='commander',
            executable='mvp_c2_traffic_control_ros',
            name='mvp_c2_traffic_control',
            output='screen',
            prefix=['stdbuf -o L'],
            parameters=[traffic_setting_file],
        ),
    ])