from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="stretch_study",
            executable="study_engine",
            name="stretch_study_engine",
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare("stretch_study"),
                    "config",
                    "defaults.yaml",
                ])
            ],
            output="screen",
        )
    ])