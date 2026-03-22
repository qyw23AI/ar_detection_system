from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        # default_value="/home/r1/9_grid_ar_detection/ar_calculate/src/ar_grid_detector/config/ar_grid.layer3.params.yaml",
        default_value="/home/r1/9_grid_ar_detection/ar_calculate/src/ar_grid_detector/config/ar_grid.params.yaml",
        description="Path to AR grid parameter yaml",
    )

    return LaunchDescription([
        params_file_arg,
        Node(
            package="ar_grid_detector",
            executable="ar_grid_node.py",
            name="ar_grid_node",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])
