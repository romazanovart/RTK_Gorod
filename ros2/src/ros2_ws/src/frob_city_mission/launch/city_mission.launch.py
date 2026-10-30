import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('frob_city_mission'), 'config', 'mission.yaml'
    )
    return LaunchDescription([
        Node(
            package='frob_city_mission',
            executable='city_mission',
            name='city_mission',
            output='screen',
            parameters=[config],
        )
    ])
