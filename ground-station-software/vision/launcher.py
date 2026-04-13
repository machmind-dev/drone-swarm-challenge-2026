from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_pkg_name',
            executable='aruco_node.py',
            name='aruco_node',
            output='screen',
            parameters=['/home/pihas/esp32s3-microros/gcs/vision/aruco_params.yaml']
        )
    ])
