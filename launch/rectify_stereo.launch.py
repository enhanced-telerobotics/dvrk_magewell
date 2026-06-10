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
            namespace='/davinci_endo/left',
            remappings=[
                ('image', 'image_raw'),
            ]),

        # resize_node for right image
        launch_ros.actions.Node(
            package='image_proc', executable='rectify_node', output='screen',
            name='rectify_right',
            parameters=[
                {'interpolation': 1}  # Optional: Bilinear interpolation
            ],
            namespace='/davinci_endo/right',
            remappings=[
                ('image', 'image_raw'),
            ]),
    ])
