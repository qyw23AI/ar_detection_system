from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    default_params = (
        get_package_share_directory("sensor_spatial_transform")
        + "/config/sensor_spatial_transform.params.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="sensor_spatial_transform",
                executable="gimbal_tf_handler",
                name="gimbal_tf_handler",
                output="screen",
                parameters=[default_params],
            ),
            Node(
                package="sensor_spatial_transform",
                executable="ar_pose_adapter",
                name="ar_pose_adapter",
                output="screen",
                parameters=[default_params],
            ),
        ]
    )