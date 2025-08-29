import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction


def generate_launch_description():

    # robot
    robot_name = 'mvp2_test_robot'

    # param path
    acomms_param = os.path.join(get_package_share_directory('mvp_c2_traffic_manager'), 'config', 'acomms.yaml')
    serial_param = os.path.join(get_package_share_directory('mvp_c2_traffic_manager'), 'config', 'serial.yaml')
    udp_param = os.path.join(get_package_share_directory('mvp_c2_traffic_manager'), 'config', 'udp.yaml')

    # launch the node
    return LaunchDescription([

        Node(
            package= 'mvp_c2_traffic_manager',
            executable='mvp_c2_traffic_manager',
            namespace=robot_name,
            name='mvp_c2_traffic_manager',
            output='screen',
            prefix=['stdbuf -o L'],
            parameters=[
                {'load_config': ["acomms"]},
                acomms_param,
                serial_param,
                udp_param

            ],
            # arguments=["--ros-args", "--log-level", "debug"],
        )
    ])