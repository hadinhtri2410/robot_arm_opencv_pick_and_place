from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="opencv_perception",
                executable="load_images",
                name="load_images",
                output="screen",
            )
        ]
    )
