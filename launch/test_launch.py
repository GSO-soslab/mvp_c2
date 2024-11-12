from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mvp_c2',
            namespace='mvp_c2',
            executable='mvp_c2_reporter',
            name='mvp_c2_reporter',
            output='screen',
            prefix=['stdbuf -o L']
        ),
        Node(
            package='mvp_c2',
            namespace='mvp_c2',
            executable='mvp_c2_commander',
            name='mvp_c2_commander',
            output='screen',
            prefix=['stdbuf -o L'],
        ),
    ])