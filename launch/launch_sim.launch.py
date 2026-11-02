import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_gazebo_with_world(context):
    world_arg = LaunchConfiguration('world').perform(context)

    if world_arg.startswith('./') or world_arg.startswith('../'):
        world_path = os.path.abspath(world_arg)
    else:
        world_path = world_arg

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py',
            )
        ),
        launch_arguments={'world': world_path}.items(),
    )
    return [gazebo_launch]


def generate_launch_description():
    pkg_path = get_package_share_directory('articubot_one')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_path, 'worlds', 'empty.world'),
        description='Path to Gazebo world file',
    )

    # Publish robot_description and fixed-joint TFs.
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            # Wheel joint states come from joint_state_broadcaster.
            'use_joint_state_publisher': 'false',
        }.items(),
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'agv_4banh',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.13',
        ],
        output='screen',
    )

    # Give Gazebo time to spawn the model and load gazebo_ros2_control.
    delayed_spawn = TimerAction(period=3.0, actions=[spawn_entity])

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_broad',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
        output='screen',
    )

    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_cont',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
        output='screen',
    )

    # Start the state broadcaster before the drive controller.
    delayed_joint_state_broadcaster = TimerAction(
        period=8.0,
        actions=[joint_state_broadcaster],
    )
    delayed_diff_drive_controller = TimerAction(
        period=12.0,
        actions=[diff_drive_controller],
    )

    return LaunchDescription([
        world_arg,
        rsp,
        OpaqueFunction(function=launch_gazebo_with_world),
        delayed_spawn,
        delayed_joint_state_broadcaster,
        delayed_diff_drive_controller,
    ])
