from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    video_device = LaunchConfiguration("video_device")

    camera = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        output="log",
        parameters=[{
            "image_size": [640, 480],
            "time_per_frame": [1, 6],
            "camera_frame_id": "camera_optical_link",
            "video_device": video_device
        }]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "video_device",
                default_value="/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_8mp_SN0001-video-index0",
                description="The device file of the camera to use."
            ),
            camera
        ]
    )
