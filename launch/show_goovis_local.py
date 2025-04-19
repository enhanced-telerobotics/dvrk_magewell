from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    mtm_display = Node(
        package='dvrk_magewell',
        executable='local_display_video',
        name='local_display_video',
        output='screen',
        arguments=['-c', '-h', '1080', '-w', f'{1920//2}', '--left-offset', f'{2560*2}']
        # Concatenate stereo images and squeeze to 1080p
        # Add 2 WQHD monitors padded to the left windows
    )

    return LaunchDescription([mtm_display])
