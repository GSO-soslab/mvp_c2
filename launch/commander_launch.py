import os
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


# from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    comm_setting_file = os.path.join(get_package_share_directory('mvp_c2'), 'config', 'serial_setting.yaml') 
    udp_comm_setting_file = os.path.join(get_package_share_directory('mvp_c2'), 'config', 'udp_setting.yaml') 
    commander_setting_file = os.path.join(get_package_share_directory('mvp_c2'), 'config', 'commander_setting.yaml') 

    return LaunchDescription([
        
        # Node(
        #     package = 'mvp_c2',
        #     namespace = 'commander',
        #     executable='mvp_c2_serial_comm',
        #     name = 'commander_c2_serial_comm',
        #     output='screen',
        #     prefix=['stdbuf -o L'],
        #     parameters=[comm_setting_file],
        #     remappings=[
        #         ('dccl_msg_tx', 'mvp_c2/dccl_msg_tx'),
        #         ('dccl_msg_rx', 'mvp_c2/dccl_msg_rx'),
        #     ]
        # ),

        Node(
            package = 'mvp_c2',
            namespace = 'commander',
            executable='mvp_c2_udp_comm',
            name = 'commander_c2_udp_comm',
            output='screen',
            prefix=['stdbuf -o L'],
            parameters=[udp_comm_setting_file],
            remappings=[
                ('dccl_msg_tx', 'mvp_c2/dccl_msg_tx'),
                ('dccl_msg_rx', 'mvp_c2/dccl_msg_rx'),
            ]
        ),
    #commander node
        Node(
            package='mvp_c2',
            namespace='commander',
            executable='mvp_c2_commander_ros',
            name='mvp_c2_commander',
            output='screen',
            prefix=['stdbuf -o L'],
            parameters=[commander_setting_file],
        ),

        # Node(
        #     package="joy",
        #     executable="joy_node",
        #     name="joy_node",
        #     namespace='commander',
        #     output="screen",
        #     parameters=[
        #         {'coalesce_interval': 10},
        #         {'autorepeat_rate': 0.0}
        #     ],
        #     remappings=[
        #         ('joy', 'remote/id_2/joy'),
        #     ]   
        # ),
    ])