import os
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


# from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    buffer_setting = os.path.join(get_package_share_directory('mvp_c2'), 'config', 'dynamic_buffer.yaml') 

    return LaunchDescription([
    

        Node(
            package='mvp_c2',
            namespace='test',
            executable='mvp_c2_traffic_control_ros',
            name='mvp_c2_traffic_control',
            output='screen',
            prefix=['stdbuf -o L'],
            parameters=[buffer_setting],
        ),
    ])