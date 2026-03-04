from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
import launch_ros.actions


def generate_launch_description():
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='HD',
        description='Select device: SD or HD'
    )

    device = LaunchConfiguration('device')

    # HD block
    hd_group = GroupAction([
        launch_ros.actions.Node(
            package='v4l2_camera', executable='v4l2_camera_node', output='screen',
            namespace='davinci_endo/left',
            parameters=[{
                'video_device': '/dev/video2',
                'camera_frame_id': 'left',
                'camera_info_url': 'file:///home/erie_lab/ros2_ws/src/dvrk_magewell/resources/left_0.yaml',
                'output_encoding': 'bgr8',
                'image_size': [1920, 1080],
            }],
            name='left_image_publisher',
        ),
        launch_ros.actions.Node(
            package='v4l2_camera', executable='v4l2_camera_node', output='screen',
            namespace='davinci_endo/right',
            parameters=[{
                'video_device': '/dev/video0',
                'camera_frame_id': 'right',
                'camera_info_url': 'file:///home/erie_lab/ros2_ws/src/dvrk_magewell/resources/right_0.yaml',
                'output_encoding': 'bgr8',
                'image_size': [1920, 1080],
            }],
            name='right_image_publisher',
        ),
    ], condition=LaunchConfigurationEquals('device', 'HD'))

    # SD block
    sd_group = GroupAction([
        launch_ros.actions.Node(
            package='v4l2_camera', executable='v4l2_camera_node', output='screen',
            namespace='davinci_endo/left',
            parameters=[{
                'video_device': '/dev/video4',
                'camera_frame_id': 'left',
                'camera_info_url': 'file:///home/erie_lab/ros2_ws/src/dvrk_magewell/resources/sd_left.yaml',
                'output_encoding': 'bgr8',
                'image_size': [640, 480]
            }],
            name='left_image_publisher',
        ),
        launch_ros.actions.Node(
            package='v4l2_camera', executable='v4l2_camera_node', output='screen',
            namespace='davinci_endo/right',
            parameters=[{
                'video_device': '/dev/video5',
                'camera_frame_id': 'right',
                'camera_info_url': 'file:///home/erie_lab/ros2_ws/src/dvrk_magewell/resources/sd_right.yaml',
                'output_encoding': 'bgr8',
                'image_size': [640, 480]
            }],
            name='right_image_publisher',
        ),
    ], condition=LaunchConfigurationEquals('device', 'SD'))

    return LaunchDescription([
        device_arg,
        hd_group,
        sd_group,
    ])
