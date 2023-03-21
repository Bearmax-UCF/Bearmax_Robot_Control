from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("bearmax_moveit_config").to_moveit_configs()

    # Launch test executable
    move_group_test = Node(
        name="bearmax_moveit",
        package="bearmax_moveit",
        executable="bearmax_moveit",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([move_group_test])
