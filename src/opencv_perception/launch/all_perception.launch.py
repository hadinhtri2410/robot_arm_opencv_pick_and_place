from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="opencv_perception",
                executable="load_images_from_camera",
                name="load_images_from_camera",
                output="screen",
            ),
            Node(
                package="opencv_perception",
                executable="color_filtering",
                name="color_filtering",
                output="screen",
            ),
            Node(
                package="opencv_perception",
                executable="edge_detection",
                name="edge_detection",
                output="screen",
            ),
            Node(
                package="opencv_perception",
                executable="stream_camera",
                name="stream_camera",
                output="screen",
            ),
            Node(
                package="opencv_perception",
                executable="hsv_from_mouse",
                name="hsv_from_mouse",
                output="screen",
            ),
        ]
    )
