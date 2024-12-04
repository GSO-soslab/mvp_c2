import os
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


# from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    comm_setting_file = os.path.join(get_package_share_directory('mvp_c2'), 'config', 'serial_setting.yaml') 

    return LaunchDescription([
        
        Node(
            package = 'mvp_c2',
            namespace = 'reporter',
            executable='mvp_c2_serial_comm',
            name = 'reporter_c2_serial_comm',
            output='screen',
            prefix=['stdbuf -o L'],
            parameters=[comm_setting_file],
            remappings=[
                ('dccl_msg_tx', 'mvp_c2/dccl_msg_tx'),
                ('dccl_msg_rx', 'mvp_c2/dccl_msg_rx'),
            ]
        ),

        Node(
            package='mvp_c2',
            namespace='reporter',
            executable='mvp_c2_reporter_ros',
            name='mvp_c2_reporter',
            output='screen',
            prefix=['stdbuf -o L']
        ),
    ])