from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    device = LaunchConfiguration('device')
    rectify = LaunchConfiguration('rectify')
    compressed = LaunchConfiguration('compressed')
    profile = LaunchConfiguration('profile')
    sync_queue_size = LaunchConfiguration('sync_queue_size')
    sync_inter_message_lower_bound_ms = LaunchConfiguration(
        'sync_inter_message_lower_bound_ms')
    transport = PythonExpression([
        '"compressed" if "', compressed, '".lower() in ("true", "1") else "raw"'
    ])
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

    compressed_arg = DeclareLaunchArgument(
        'compressed', default_value='true',
        description='Use compressed image transport for display'
    )

    profile_arg = DeclareLaunchArgument(
        'profile', default_value='false',
        description='Log display latency and rendering timing once per second'
    )

    sync_queue_size_arg = DeclareLaunchArgument(
        'sync_queue_size', default_value='10',
        description='ApproximateTime stereo synchronization queue depth'
    )

    sync_lower_bound_arg = DeclareLaunchArgument(
        'sync_inter_message_lower_bound_ms', default_value='15.0',
        description='Known minimum interval between frames; 15 ms is suitable for 60 Hz'
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

    display_remappings = []
    for eye in ('left', 'right'):
        # Accept namespaces with or without a leading slash.
        base = PythonExpression(['"/" + "', ns, '".strip("/") + "/', eye, '"'])
        source_image = PythonExpression([
            '"', base, '/" + ("image_rect" if "', rectify,
            '".lower() in ("true", "1") else "image_raw")'
        ])
        display_remappings.append((f'davinci_endo/{eye}/image_raw', source_image))

    display = Node(
        package='dvrk_magewell',
        executable='display_video',
        name='display_video',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'image_transport': transport,
            'profile': ParameterValue(profile, value_type=bool),
            'sync_queue_size': ParameterValue(sync_queue_size, value_type=int),
            'sync_inter_message_lower_bound_ms': ParameterValue(
                sync_inter_message_lower_bound_ms,
                value_type=float,
            ),
        }],
        arguments=common_args,
        remappings=display_remappings,
    )

    return LaunchDescription([
        device_arg,
        rectify_arg,
        namespace_arg,
        compressed_arg,
        profile_arg,
        sync_queue_size_arg,
        sync_lower_bound_arg,
        display,
    ])
