from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="opencv_perception",
                executable="stream_camera",
                name="stream_camera",
                output="screen",
            )
        ]
    )
