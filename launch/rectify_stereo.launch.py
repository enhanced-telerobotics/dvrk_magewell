from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        # resize_node for left image
        launch_ros.actions.Node(
            package='image_proc', executable='rectify_node', output='screen',
            name='rectify_left',
            parameters=[
                {'interpolation': 1}  # Optional: Bilinear interpolation
            ],
            remappings=[
                ('image', '/davinci_endo/left/image_raw'),
                ('camera_info', '/davinci_endo/left/camera_info'),
                ('image_rect', '/davinci_endo/left/image_rect')
            ]),

        # resize_node for right image
        launch_ros.actions.Node(
            package='image_proc', executable='rectify_node', output='screen',
            name='rectify_right',
            parameters=[
                {'interpolation': 1}  # Optional: Bilinear interpolation
            ],
            remappings=[
                ('image', '/davinci_endo/right/image_raw'),
                ('camera_info', '/davinci_endo/right/camera_info'),
                ('image_rect', '/davinci_endo/right/image_rect')
            ]),
    ])
