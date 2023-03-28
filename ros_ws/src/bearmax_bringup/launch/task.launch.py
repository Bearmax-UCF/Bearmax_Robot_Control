import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bearmax_emotion"),
                'camera.launch.py')
        )
    )

    emotion_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bearmax_emotion"),
                'pipeline.launch.py')
        )
    )

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bearmax_bringup"),
                'launch',
                'base.launch.py')
        )
    )

    task_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bearmax_moveit"),
                'launch',
                'task_server.launch.py')
        )
    )

    return LaunchDescription(
        [
            camera,
            emotion_pipeline,
            base,
#            task_server
        ]
    )
