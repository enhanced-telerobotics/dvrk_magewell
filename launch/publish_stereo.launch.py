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

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    device = LaunchConfiguration('device')

    # HD block
    hd_group = GroupAction([
        launch_ros.actions.Node(
            package='image_publisher', executable='image_publisher_node', output='screen',
            arguments=['2'],
            parameters=[{'use_sim_time': use_sim_time, 'publish_rate': 60.0,
                         'camera_info_url': 'file:///home/erie_lab/ros2_ws/src/dvrk_magewell/resources/left_0.yaml'}],
            name='image_publisher_left',
            remappings=[('image_raw', '/davinci_endo/left/image_raw'),
                        ('camera_info', '/davinci_endo/left/camera_info')]),
        launch_ros.actions.Node(
            package='image_publisher', executable='image_publisher_node', output='screen',
            arguments=['0'],
            parameters=[{'use_sim_time': use_sim_time, 'publish_rate': 60.0,
                         'camera_info_url': 'file:///home/erie_lab/ros2_ws/src/dvrk_magewell/resources/right_0.yaml'}],
            name='image_publisher_right',
            remappings=[('image_raw', '/davinci_endo/right/image_raw'),
                        ('camera_info', '/davinci_endo/right/camera_info')]),
    ], condition=LaunchConfigurationEquals('device', 'HD'))

    # SD block
    sd_group = GroupAction([
        launch_ros.actions.Node(
            package='dvrk_magewell', executable='publish_video', output='screen',
            parameters=[{'device': 4}],
            name='image_publisher_left',
            remappings=[('image_raw', '/davinci_endo/left/image_raw')]),
        launch_ros.actions.Node(
            package='dvrk_magewell', executable='publish_video', output='screen',
            parameters=[{'device': 5}],
            name='image_publisher_right',
            remappings=[('image_raw', '/davinci_endo/right/image_raw')]),
    ], condition=LaunchConfigurationEquals('device', 'SD'))

    return LaunchDescription([
        device_arg,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),
        hd_group,
        sd_group,
    ])
