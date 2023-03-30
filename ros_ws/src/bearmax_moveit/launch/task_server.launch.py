from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    head_position_topic = LaunchConfiguration("head_position_topic")
    should_follow = LaunchConfiguration("should_follow")
    foot_z = LaunchConfiguration("foot_z")
    head_position_error = LaunchConfiguration("head_position_error")

    moveit_config = (
        MoveItConfigsBuilder("robot", package_name="bearmax_moveit_config")
        .to_moveit_configs()
    )

    task_server_node = Node(
        name="task_server",
        package="bearmax_moveit",
        executable="task_server",
        remappings=[
            ("/head_in", head_position_topic),
        ],
        output="screen",
        parameters=[
            {
                "should_follow": should_follow,
                "foot_z": foot_z, 
                "head_position_error": head_position_error
            },
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "head_position_topic",
                default_value="/head_pos",
                description="The name of the input head position topic."
            ),
            DeclareLaunchArgument(
                "should_follow",
                default_value='True',
                description="Should the robot follow the detected face?"
            ),
            DeclareLaunchArgument(
                "foot_z",
                default_value='0.312',
                description="The face z value at a distance of 1 foot from the camera."
            ),
            DeclareLaunchArgument(
                "head_position_error",
                default_value='0.02',
                description="The amount of error allowed when deciding whether to move the head to track a face."
            ),
            task_server_node
        ]
    )


