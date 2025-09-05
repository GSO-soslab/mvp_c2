import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction


def generate_launch_description():

    # robot
    robot_name = 'mvp2_test_robot'

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
                {'type': "acomms"},
            ],
            # arguments=["--ros-args", "--log-level", "debug"],
        )
    ])