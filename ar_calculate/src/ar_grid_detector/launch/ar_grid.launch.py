from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="ar_grid_detector",
            executable="ar_grid_node.py",
            name="ar_grid_node",
            output="screen",
            parameters=["/home/r1/9_grid_ar_detection/ar_calculate/src/ar_grid_detector/config/ar_grid.params.yaml"],
        ),
    ])
