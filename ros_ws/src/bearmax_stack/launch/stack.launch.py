from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    ws_url = LaunchConfiguration("ws_url")

        stack = Node(
            package="bearmax_stack",
            executable="stack_connector",
            parameters=[
                {"ws_url": ws_url}
                    ]
                )

            return LaunchDescription(
            [
                DeclareLaunchArgument(
                    "ws_url",
                    default_value="https://carewithbearmax.com",
                    description="The base url of the stack."
                    ),
                stack
                ]
                )

