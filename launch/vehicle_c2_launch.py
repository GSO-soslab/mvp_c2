import os
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


# from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    udp_setting_file = os.path.join(get_package_share_directory('mvp_c2'), 'config', 'vehicle_udp_setting.yaml') 

    return LaunchDescription([
        Node(
            package='mvp_c2',
            namespace='mvp2_test_robot',
            executable='mvp_c2_dccl_ros',
            name='vehicle_c2',
            output='screen',
            # prefix=['stdbuf -o L']
            remappings=[
                ('local/odometry', 'odometry/filtered'),
                ('local/geopose', 'odometry/geopose')
            ]
        ),
        Node(
            package = 'mvp_c2',
            namespace = 'mvp2_test_robot',
            executable='mvp_c2_udp_comm',
            name = 'vehicle_c2_udp_comm',
            output='screen',
            prefix=['stdbuf -o L'],
            parameters=[udp_setting_file],
            remappings=[
                ('dccl_msg_tx', 'mvp_c2/dccl_msg_tx'),
                ('dccl_msg_rx', 'mvp_c2/dccl_msg_rx')
            ]
        ),
    ])