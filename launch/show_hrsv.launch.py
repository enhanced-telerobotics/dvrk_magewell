from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    device = LaunchConfiguration('device')
    rectify = LaunchConfiguration('rectify')
    resize = LaunchConfiguration('resize')
    compressed = LaunchConfiguration('compressed')
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

    resize_arg = DeclareLaunchArgument(
        'resize', default_value='false',
        description='Resize each image to 960x540 before display'
    )

    compressed_arg = DeclareLaunchArgument(
        'compressed', default_value='true',
        description='Use compressed image transport for display and resize inputs'
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

    resize_nodes = []
    display_remappings = []
    for eye in ('left', 'right'):
        # Accept namespaces with or without a leading slash.
        base = PythonExpression(['"/" + "', ns, '".strip("/") + "/', eye, '"'])
        source_image = PythonExpression([
            '"', base, '/" + ("image_rect" if "', rectify,
            '".lower() in ("true", "1") else "image_raw")'
        ])
        resize_nodes.append(Node(
            package='image_proc', executable='resize_node',
            name=f'resize_{eye}', output='screen',
            parameters=[{
                'use_scale': False, 'height': 540, 'width': 960,
                'interpolation': 1,
                'image_transport': transport,
            }],
            remappings=[
                ('image/image_raw', source_image),
                ('image/image_raw/compressed', [source_image, '/compressed']),
                ('image/camera_info', [base, '/camera_info']),
                ('resize/image_raw', [base, '/resized/image_raw']),
                ('resize/image_raw/compressed', [base, '/resized/image_raw/compressed']),
                ('resize/camera_info', [base, '/resized/camera_info']),
            ],
            condition=IfCondition(resize),
        ))
        display_image = PythonExpression([
            '"', base, '/resized/image_raw" if "', resize,
            '".lower() in ("true", "1") else "', source_image, '"'
        ])
        display_remappings.append((f'davinci_endo/{eye}/image_raw', display_image))

    display = Node(
        package='dvrk_magewell',
        executable='display_video',
        name='display_video',
        output='screen',
        parameters=[{'use_sim_time': False, 'image_transport': transport}],
        arguments=common_args,
        remappings=display_remappings,
    )

    return LaunchDescription([
        device_arg,
        rectify_arg,
        namespace_arg,
        resize_arg,
        compressed_arg,
        *resize_nodes,
        display,
    ])
