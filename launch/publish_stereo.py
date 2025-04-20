from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare a launch argument for extra arguments
    height_arg = DeclareLaunchArgument(
        'height',
        default_value='0',  # Default value for full resolution
        description='Height of the video stream'
    )

    # Use LaunchConfiguration to retrieve the value of the height argument
    height_config = LaunchConfiguration('height')

    left_camera = Node(
        package='dvrk_magewell',
        executable='publish_video',
        name='publish_video_left',
        output='screen',
        arguments=['--topic', 'davinci_endo/left/image_raw',
                   '--device', '2', '--height', height_config]
    )
    right_camera = Node(
        package='dvrk_magewell',
        executable='publish_video',
        name='publish_video_right',
        output='screen',
        arguments=['--topic', 'davinci_endo/right/image_raw',
                   '--device', '0', '--height', height_config]
    )

    return LaunchDescription([height_arg, left_camera, right_camera])
