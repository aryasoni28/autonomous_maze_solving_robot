import launch
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="Starting ArUco marker detection..."),
        Node(
            package='autonomous_tb3',  # Name of your package
            executable='aruco_marker_detection.py',  # The Python script you wrote
            output='screen',
            parameters=[],
        ),
    ])

