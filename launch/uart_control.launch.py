from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/serial0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        Node(
            package='articubot_one',
            executable='agv_uart_node',
            name='agv_uart_node',
            output='screen',
            parameters=[{
                'port': ParameterValue(LaunchConfiguration('port'), value_type=str),
                'baud': ParameterValue(LaunchConfiguration('baud'), value_type=int),
            }],
        ),
    ])
