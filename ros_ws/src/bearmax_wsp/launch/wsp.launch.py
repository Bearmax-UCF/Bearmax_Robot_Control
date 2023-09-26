from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    address = LaunchConfiguration("address")
    name = LaunchConfiguration("name")
    gsr_uuid = LaunchConfiguration("gsr_uuid")

    stack = Node(
        package="bearmax_wsp",
        executable="wsp_connector",
        parameters=[
            {
                "address": address,
                "name": name,
                "gsr_uuid": gsr_uuid
            }
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "address",
                default_value="D5:79:C3:1D:B0:21",
                description="The address of the wearable device"
                ),
            DeclareLaunchArgument(
                "name",
                default_value="Feather nRF52840 Sense",
                description="The name of the wearable device"
                ),
            DeclareLaunchArgument(
                "gsr_uuid",
                default_value="00000403-1212-efde-1523-785feabcd123",
                description="The UUID of the GSR sensor characteristic"
                ),
            stack
        ]
    )
