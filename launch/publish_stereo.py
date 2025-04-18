from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    left_camera = Node(
        package='dvrk_magewell',
        executable='publish_video',
        name='publish_video_left',
        output='screen',
        arguments=['--topic', 'davinci_endo/left/image_raw', '--device', '2']
    )
    right_camera = Node(
        package='dvrk_magewell',
        executable='publish_video',
        name='publish_video_right',
        output='screen',
        arguments=['--topic', 'davinci_endo/right/image_raw', '--device', '0']
    )
    return LaunchDescription([left_camera, right_camera])
