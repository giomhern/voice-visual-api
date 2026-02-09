from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="stretch_study",
            executable="clean_surface_patched",
            name="clean_surface_patched",
            output="screen",
        )
    ])