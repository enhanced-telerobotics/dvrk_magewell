from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        # resize_node for left image
        launch_ros.actions.Node(
            package='image_proc', executable='resize_node', output='screen',
            name='resize_left',
            parameters=[
                {'use_scale': False},
                {'height': 540},
                {'width': 960},
                {'interpolation': 1}  # Optional: Bilinear interpolation
            ],
            remappings=[
                ('image/image_raw', '/davinci_endo/left/image_raw'),
                ('image/camera_info', '/davinci_endo/left/camera_info'),
                ('resize/image_raw', '/davinci_endo/left/resized/image_raw'),
                ('resize/camera_info', '/davinci_endo/left/resized/camera_info')
            ]),

        # resize_node for right image
        launch_ros.actions.Node(
            package='image_proc', executable='resize_node', output='screen',
            name='resize_right',
            parameters=[
                {'use_scale': False},
                {'height': 540},
                {'width': 960},
                {'interpolation': 1}  # Optional: Bilinear interpolation
            ],
            remappings=[
                ('image/image_raw', '/davinci_endo/right/image_raw'),
                ('image/camera_info', '/davinci_endo/right/camera_info'),
                ('resize/image_raw', '/davinci_endo/right/resized/image_raw'),
                ('resize/camera_info', '/davinci_endo/right/resized/camera_info')
            ]),
    ])
