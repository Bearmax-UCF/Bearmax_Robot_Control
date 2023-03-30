import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    camera = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        output="log",
        parameters=[{
            "image_size": [640, 480],
            "time_per_frame": [1, 6],
            "camera_frame_id": "camera_optical_link",
            "video_device": "/dev/video0"
        }]
    )

    return LaunchDescription([camera])
