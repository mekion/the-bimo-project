from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('state_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('baudrate', default_value='921600'),
        DeclareLaunchArgument('calibrate', default_value='false'),
        Node(
            package='bimo_ros2',
            executable='bimo_comms',
            name='bimo_comms',
            output='screen',
            emulate_tty=True,
            additional_env={'PYTHONUNBUFFERED': '1'},
            parameters=[{
                'state_rate_hz': LaunchConfiguration('state_rate_hz'),
                'baudrate': LaunchConfiguration('baudrate'),
                'calibrate': LaunchConfiguration('calibrate'),
            }],
        ),
    ])
