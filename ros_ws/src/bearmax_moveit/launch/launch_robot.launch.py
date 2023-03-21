import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("robot", package_name="bearmax_moveit_config")
        .to_moveit_configs()
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bearmax_moveit_config"),
                'launch',
                'move_group.launch.py')
        )
    )

    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bearmax_moveit_config"),
                "launch",
                "moveit_rviz.launch.py")
        )
    )

    static_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bearmax_moveit_config"),
                'launch',
                'static_virtual_joint_tfs.launch.py')
        )
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bearmax_moveit_config"),
                'launch',
                'rsp.launch.py')
        )
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            os.path.join(
                moveit_config.package_path,
                "config",
                "ros2_controllers.yaml"
            )
        ]
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "ears_controller",
        "left_arm_controller",
        "right_arm_controller",
        "arms_controller",
        "head_platform_controller",
        "chassis_controller",
        "body_controller",
        "joint_state_broadcaster"
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    return LaunchDescription(
        [
            rviz_node,
            static_tf,
            rsp,
            move_group,
            ros2_control_node
        ]
        + load_controllers
    )
