from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import sys


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'python_executable', default_value=sys.executable,
            description='Python interpreter with ultralytics and ROS 2 dependencies',
        ),
        Node(
            package='frob_sign_detection',
            executable='sign_detector',
            name='sign_detector',
            prefix=LaunchConfiguration('python_executable'),
            output='screen',
            parameters=[{
                'camera_device': 0,
                'camera_width': 640,
                'camera_height': 480,
                'confidence': 0.45,
                'image_size': 320,
                'max_fps': 4.0,
                'http_port': 8080,
            }],
        )
    ])
