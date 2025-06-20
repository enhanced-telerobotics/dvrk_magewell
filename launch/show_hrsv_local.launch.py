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
        executable='local_display_video',
        name='local_display_video',
        output='screen',
        arguments=[
            '-h', '768',
            '-w', '1024',
            '--left-offset', f'{2*2560}',
            '--right-offset', f'{2*2560 + 1024}',
            '--ratio', '4:3',
            '--method', 'crop',
        ],
        condition=LaunchConfigurationEquals('device', 'HD')
    )

    sd_display = Node(
        package='dvrk_magewell',
        executable='local_display_video',
        name='local_display_video',
        output='screen',
        arguments=[
            '-h', '768',
            '-w', '1024',
            '--left-offset', f'{2*2560}',
            '--right-offset', f'{2*2560 + 1024}',
            '--ratio', '4:3',
            '--method', 'original',
            '--device', 'SD'
        ],
        condition=LaunchConfigurationEquals('device', 'SD')
    )

    return LaunchDescription([
        device_arg,
        hd_display,
        sd_display,
    ])
