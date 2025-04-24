from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():
    device_0 = '2'
    device_1 = '0'
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),

        # image_publisher stereo left
        launch_ros.actions.Node(
            package='image_publisher', executable='image_publisher_node', output='screen',
            arguments=[device_0],
            parameters=[{'use_sim_time': use_sim_time, 'publish_rate': 60.0}],
            name='image_publisher_left',
            remappings=[('image_raw', '/davinci_endo/left/image_raw'),
                        ('camera_info', '/davinci_endo/left/camera_info')]),

        # image_publisher stereo right
        launch_ros.actions.Node(
            package='image_publisher', executable='image_publisher_node', output='screen',
            arguments=[device_1],
            parameters=[{'use_sim_time': use_sim_time, 'publish_rate': 60.0}],
            name='image_publisher_right',
            remappings=[('image_raw', '/davinci_endo/right/image_raw'),
                        ('camera_info', '/davinci_endo/right/camera_info')]),
    ])
