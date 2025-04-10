from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dvrk_magewell',
            executable='publish_video',
            name='publish_video',
            output='screen'
        ),
        Node(
            package='dvrk_magewell',
            executable='display_video',
            name='display_video',
            output='screen'
        ),
    ])
