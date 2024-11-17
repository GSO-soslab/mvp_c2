import os
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


# from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    udp_setting_file = os.path.join(get_package_share_directory('mvp_c2'), 'config', 'topside_udp_setting.yaml') 

    return LaunchDescription([
        Node(
            package='mvp_c2',
            namespace='topside',
            executable='mvp_c2_dccl_ros',
            name='topside_c2',
            output='screen',
            # prefix=['stdbuf -o L']
        ),
        Node(
            package = 'mvp_c2',
            namespace = 'topside',
            executable='mvp_c2_udp_comm',
            name = 'topside_c2_udp_comm',
            output='screen',
            prefix=['stdbuf -o L'],
            parameters=[udp_setting_file],
            remappings=[
                ('dccl_msg_tx', 'dccl_msg_tx')
            ]
        ),
    ])