from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Usage Examples:
# ros2 launch dvrk_magewell publish_stereo.py
# ros2 launch dvrk_magewell publish_stereo.py height:=720
# ros2 launch dvrk_magewell publish_stereo.py height:=540

# Display Examples:
# ros2 run dvrk_magewell display_video -d -m -h 720 -w 1280 --left-offset 0
# ros2 run dvrk_magewell display_video -d -c -h 720 -w 1280 --left-offset 0
# ros2 run dvrk_magewell display_video -c -h 1080 -w 960 --left-offset 2560 # goovis
# ros2 run dvrk_magewell display_video -h 768 -w 1024 --crop --left-offset 5120 # mtm

def generate_launch_description():
    # Declare a launch argument for extra arguments
    height_arg = DeclareLaunchArgument(
        'height',
        default_value='0', # Default value for full resolution
        description='Height of the video stream'
    )

    # Use LaunchConfiguration to retrieve the value of the height argument
    height_config = LaunchConfiguration('height')

    left_camera = Node(
        package='dvrk_magewell',
        executable='publish_video',
        name='publish_video_left',
        output='screen',
        arguments=['--topic', 'davinci_endo/left/image_raw', '--device', '2', '--height', height_config]
    )
    right_camera = Node(
        package='dvrk_magewell',
        executable='publish_video',
        name='publish_video_right',
        output='screen',
        arguments=['--topic', 'davinci_endo/right/image_raw', '--device', '0', '--height', height_config]
    )

    return LaunchDescription([height_arg, left_camera, right_camera])
