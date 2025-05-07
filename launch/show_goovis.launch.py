from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    display_node = Node(
        package='dvrk_magewell',
        executable='display_video',
        name='display_video',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-c', '-h', '1080', '-w', f'{1920//2}', '--left-offset', f'{2560*2}']
        # Concatenate stereo images and squeeze to 1080p
        # Add 2 WQHD monitors padded to the left windows
    )

    display_node = TimerAction(
        period=1.0,
        actions=[display_node]
    )

    return LaunchDescription([display_node])
