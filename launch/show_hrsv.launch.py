from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='HD',
        description='Select device: SD or HD'
    )

    rectify_arg = DeclareLaunchArgument(
        'rectify',
        default_value='False',
        description='Use rectified images if True'
    )

    # Determine remappings based on rectify argument
    rectify = LaunchConfiguration('rectify')
    
    # HD display with raw images (rectify=False)
    hd_display_raw = Node(
        package='dvrk_magewell',
        executable='display_video',
        name='display_video',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-h', '768',
            '-w', '1024',
            '--left-offset', f'{2*2560}',
            '--right-offset', f'{2*2560 + 1024}',
            '--ratio', '4:3',
            '--method', 'crop',
        ],
        condition=IfCondition(
            PythonExpression([
                '"', LaunchConfiguration('device'), '" == "HD" and "',
                LaunchConfiguration('rectify'), '" == "False"'
            ])
        )
    )

    # HD display with rectified images (rectify=True)
    hd_display_rect = Node(
        package='dvrk_magewell',
        executable='display_video',
        name='display_video',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-h', '768',
            '-w', '1024',
            '--left-offset', f'{2*2560}',
            '--right-offset', f'{2*2560 + 1024}',
            '--ratio', '4:3',
            '--method', 'crop',
        ],
        remappings=[
            ('davinci_endo/left/image_raw', 'davinci_endo/left/image_rect'),
            ('davinci_endo/right/image_raw', 'davinci_endo/right/image_rect'),
        ],
        condition=IfCondition(
            PythonExpression([
                '"', LaunchConfiguration('device'), '" == "HD" and "',
                LaunchConfiguration('rectify'), '" == "True"'
            ])
        )
    )

    # SD display with raw images (rectify=False)
    sd_display_raw = Node(
        package='dvrk_magewell',
        executable='display_video',
        name='display_video',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-h', '768',
            '-w', '1024',
            '--left-offset', f'{2*2560}',
            '--right-offset', f'{2*2560 + 1024}',
            '--ratio', '4:3',
            '--method', 'original',
            '--device', 'SD'
        ],
        condition=IfCondition(
            PythonExpression([
                '"', LaunchConfiguration('device'), '" == "SD" and "',
                LaunchConfiguration('rectify'), '" == "False"'
            ])
        )
    )

    # SD display with rectified images (rectify=True)
    sd_display_rect = Node(
        package='dvrk_magewell',
        executable='display_video',
        name='display_video',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-h', '768',
            '-w', '1024',
            '--left-offset', f'{2*2560}',
            '--right-offset', f'{2*2560 + 1024}',
            '--ratio', '4:3',
            '--method', 'original',
            '--device', 'SD'
        ],
        remappings=[
            ('davinci_endo/left/image_raw', 'davinci_endo/left/image_rect'),
            ('davinci_endo/right/image_raw', 'davinci_endo/right/image_rect'),
        ],
        condition=IfCondition(
            PythonExpression([
                '"', LaunchConfiguration('device'), '" == "SD" and "',
                LaunchConfiguration('rectify'), '" == "True"'
            ])
        )
    )

    return LaunchDescription([
        device_arg,
        rectify_arg,
        hd_display_raw,
        hd_display_rect,
        sd_display_raw,
        sd_display_rect,
    ])
