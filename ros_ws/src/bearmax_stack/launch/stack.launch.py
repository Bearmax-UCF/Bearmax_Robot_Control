from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.substitutions import LaunchConfiguration
# from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    stack = Node(
        package="bearmax_stack",
        executable="stack_connector",
    )

    return LaunchDescription(
        [
            stack
        ]
    )
