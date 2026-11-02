from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [640, 480],
            'pixel_format': 'YUYV',
            'camera_frame_id': 'camera_link_optical',
        }],
    )

    return LaunchDescription([
        camera_node,
    ])