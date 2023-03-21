import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    image_topic = LaunchConfiguration("image_topic")
    head_position_topic = LaunchConfiguration("head_position_topic")
    emotion_topic = LaunchConfiguration("emotion_topic")

    pipeline = Node(
        package="bearmax_emotion",
        executable="emotion_pipeline",
        remappings=[
            ("/image_in", image_topic),
            ("/head_out", head_position_topic),
            ("/emotion_out", emotion_topic),
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "image_topic",
                default_value="/image_raw",
                description="The name of the input image topic."
            ),
            DeclareLaunchArgument(
                "head_position_topic",
                default_value="/head_pos",
                description="The name of the output head position topic."
            ),
            DeclareLaunchArgument(
                "emotion_topic",
                default_value="/emotion",
                description="The name of the output detected emotion topic."
            ),
            pipeline
        ]
    )

