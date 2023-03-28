import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    foxglove = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        output="log"
    )

    # move_groups and ros2 controllers
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bearmax_moveit"),
                'launch',
                'launch_robot.launch.py')
        )
    )

    return LaunchDescription(
        [
#            foxglove,
            control
        ]
    )
