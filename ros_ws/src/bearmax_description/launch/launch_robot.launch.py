import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node

def generate_launch_description():
    package_name = "bearmax_description"

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name),
                    'launch',
                    'rsp.launch.py')
            ]
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'use_ros2_control': 'true'
        }.items()
    )

    robot_description = Command(
        [
            ("ros2 param get --hide-type "
            "/robot_state_publisher robot_description")
        ]
    )

    controller_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'my_controllers.yaml'
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': robot_description},
            controller_params_file
        ]
    )

    delayed_controller_manager = TimerAction(
        period=3.0,
        actions=[controller_manager]
    )

    bearmax_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["main_controller"]
    )

    delayed_bearmax_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[bearmax_spawner]
        )
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"]
    )

    delayed_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner]
        )
    )

    return LaunchDescription(
        [
            rsp,
            delayed_controller_manager,
            delayed_bearmax_spawner,
            delayed_joint_state_broadcaster_spawner
        ]
    )
