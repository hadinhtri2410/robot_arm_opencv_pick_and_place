from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="opencv_perception",
                executable="hsv_from_mouse",
                name="hsv_from_mouse",
                output="screen",
            )
        ]
    )
