from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():
    device_0 = 4
    device_1 = 5
    return LaunchDescription([

        # image_publisher stereo left
        launch_ros.actions.Node(
            package='dvrk_magewell', executable='publish_video', output='screen',
            parameters=[{'device': device_0}],
            name='image_publisher_left',
            remappings=[('image_raw', '/davinci_endo/left/image_raw')]),

        # image_publisher stereo right
        launch_ros.actions.Node(
            package='dvrk_magewell', executable='publish_video', output='screen',
            parameters=[{'device': device_1}],
            name='image_publisher_right',
            remappings=[('image_raw', '/davinci_endo/right/image_raw')]),
    ])
