from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', '/home/erie_lab/ros2_ws/src/dvrk_magewell/src/local_display_video.py'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'dvrk_robot', 'dvrk_console_json', '-j', '/home/erie_lab/cwru-erie-dVRK/console-ECM-MTML-PSM2-Teleop.json'],
            output='screen',
            additional_env={'ROS_LOG_DIR': '/tmp'}
        ),
        TimerAction(
            period=1.0,
            actions=[
                ExecuteProcess(
                    cmd=['qlacommand', '-c', 'close-relays'],
                    output='screen'
                )
            ]
        ),
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=['python3', '/home/erie_lab/ros2_ws/src/dvrk_magewell/launch/teleop_init.py'],
                    output='screen'
                )
            ]
        )
    ])
