from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_pkg_name',
            executable='aruco_node.py',
            name='aruco_node',
            output='screen',
            parameters=['/home/pihas/drone-swarm-challenge-2026/ground-station-software/vision/aruco_params.yaml']
        )
    ])
