from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


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
    mtm_display = Node(
        package='dvrk_magewell',
        executable='display_video',
        name='display_video',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-h', '768', '-w', '1024', '-l', f'{2*2560}', '--crop']
        # Add 2 WQHD monitors padded to the left windows
    )

    mtm_display = TimerAction(
        period=1.0,
        actions=[mtm_display]
    )

    return LaunchDescription([left_camera, right_camera, mtm_display])
