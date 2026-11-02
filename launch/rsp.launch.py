import os
import re

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_joint_state_publisher = LaunchConfiguration('use_joint_state_publisher')

    pkg_path = get_package_share_directory('articubot_one')
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')

    # Expand Xacro to URDF.
    robot_description_config = xacro.process_file(xacro_file)
    robot_description_xml = robot_description_config.toxml()

    # IMPORTANT for ROS 2 Humble + gazebo_ros2_control:
    # The plugin forwards robot_description through ROS parameter parsing.
    # XML comments containing YAML-special text such as "word: value" or
    # "###" can make that parser fail even though the URDF XML is valid.
    # Comments do not affect the robot model, so remove them before publishing.
    robot_description_xml = re.sub(
        r'<!--.*?-->', '', robot_description_xml, flags=re.DOTALL
    )
    robot_description = ParameterValue(robot_description_xml, value_type=str)

    params = {
        'robot_description': robot_description,
        'use_sim_time': use_sim_time,
    }

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[params],
        condition=IfCondition(use_joint_state_publisher),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time',
        ),
        DeclareLaunchArgument(
            'use_joint_state_publisher',
            default_value='true',
            description='Publish default joint states outside Gazebo',
        ),
        node_joint_state_publisher,
        node_robot_state_publisher,
    ])
