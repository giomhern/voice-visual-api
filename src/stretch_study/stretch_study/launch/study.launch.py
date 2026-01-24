from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('stretch_study')
    defaults = os.path.join(share, 'config', 'defaults.yaml')
    return LaunchDescription([
        Node(
            package='stretch_study',
            executable='study_engine',
            name='stretch_study_engine',
            output='screen',
            parameters=[
                defaults,
                # You can override in your own launch file or with --ros-args -p
                {'study.session_id': 'session_001'},
                {'study.participant_id': 'p001'},
            ],
        ),
    ])
