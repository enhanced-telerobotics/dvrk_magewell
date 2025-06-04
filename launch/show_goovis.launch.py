from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='HD',
        description='Select device: SD or HD'
    )

    hd_display = Node(
        package='dvrk_magewell',
        executable='display_video',
        name='display_video',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-c',
            '-h', '1080',
            '-w', f'{1920//2}',
            '--left-offset', f'{2560*2}'
        ],
        condition=LaunchConfigurationEquals('device', 'HD')
    )

    sd_display = Node(
        package='dvrk_magewell',
        executable='display_video',
        name='display_video',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-c',
            '-h', '1080',
            '-w', f'{1920//2}',
            '--left-offset', f'{2560*2}',
            '--device', 'SD',
            '--ratio', '16:9',
            '--method', 'pad',
        ],
        condition=LaunchConfigurationEquals('device', 'SD')
    )

    return LaunchDescription([
        device_arg,
        hd_display,
        sd_display,
    ])
