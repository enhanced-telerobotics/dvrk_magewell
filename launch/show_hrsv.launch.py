from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    device = LaunchConfiguration('device')
    rectify = LaunchConfiguration('rectify')
    ns = LaunchConfiguration('namespace')

    device_arg = DeclareLaunchArgument(
        'device', default_value='HD',
        description='Select device: SD or HD'
    )
    rectify_arg = DeclareLaunchArgument(
        'rectify', default_value='False',
        description='Use rectified images if True'
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='davinci_endo',
        description='Image topic namespace'
    )

    # Common arguments (device-dependent bits done once)
    common_args = [
        '-h', '768',
        '-w', '1024',
        '--left-offset', f'{2*2560}',
        '--right-offset', f'{2*2560 + 1024}',
        '--ratio', '4:3',
        '--method',
        PythonExpression(['"crop" if "', device, '" == "HD" else "original"']),
        PythonExpression(['"--device SD" if "', device, '" == "SD" else ""']),
    ]

    # Build absolute topic names based on namespace at launch-time
    left_raw = "davinci_endo/left/image_raw"
    right_raw = "davinci_endo/right/image_raw"
    left_rect = PythonExpression(['"', ns, '/left/image_rect"'])
    right_rect = PythonExpression(['"', ns, '/right/image_rect"'])

    # rectify=False: no remapping
    display_raw = Node(
        package='dvrk_magewell',
        executable='display_video',
        name='display_video',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=common_args,
        condition=IfCondition(PythonExpression(['"', rectify, '" == "False"']))
    )

    # rectify=True: remap raw -> rect (absolute names)
    display_rect = Node(
        package='dvrk_magewell',
        executable='display_video',
        name='display_video',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=common_args,
        remappings=[
            (left_raw, left_rect),
            (right_raw, right_rect),
        ],
        condition=IfCondition(PythonExpression(['"', rectify, '" == "True"']))
    )

    return LaunchDescription([
        device_arg,
        rectify_arg,
        namespace_arg,
        display_raw,
        display_rect,
    ])
