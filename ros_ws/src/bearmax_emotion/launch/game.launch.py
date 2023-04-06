from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    emotion_topic = LaunchConfiguration("emotion_topic")
    state_topic = LaunchConfiguration("state_topic")
    emotion_threshold = LaunchConfiguration("emotion_threshold")

    game = Node(
        package="bearmax_emotion",
        executable="emotion_game",
        remappings=[
            ("/emotion_in", emotion_topic),
            ("/state_out", state_topic),
        ],
        parameters=[
            {"emotion_threshold": emotion_threshold}
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "emotion_topic",
                default_value="/emotion",
                description="The name of the input detected emotion topic."
            ),
            DeclareLaunchArgument(
                "state_topic",
                default_value="/emotion_game",
                description="The name of the output game state topic."
            ),
            DeclareLaunchArgument(
                "emotion_threshold",
                default_value="15000",
                description="The amount of time (milliseconds) needed for an emotion to be submitted to the emotion game instance."
            ),
            game
        ]
    )
