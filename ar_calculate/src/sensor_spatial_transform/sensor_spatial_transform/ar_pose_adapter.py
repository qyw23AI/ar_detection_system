#!/usr/bin/env python3

from typing import List

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class ArPoseAdapter(Node):
    def __init__(self) -> None:
        super().__init__("ar_pose_adapter")

        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("odom_topic", "/camera_pose_in_map")
        self.declare_parameter("tf_lookup_timeout_sec", 0.05)
        self.declare_parameter(
            "pose_covariance",
            [
                1e-4,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1e-4,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1e-4,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1e-3,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1e-3,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1e-3,
            ],
        )

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.tf_lookup_timeout_sec = float(self.get_parameter("tf_lookup_timeout_sec").value)

        covariance_value = self.get_parameter("pose_covariance").value
        self.pose_covariance: List[float] = [float(v) for v in covariance_value]
        if len(self.pose_covariance) != 36:
            self.get_logger().warn("pose_covariance length is not 36, fallback to all zeros")
            self.pose_covariance = [0.0] * 36

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        timer_period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.on_timer)

        self.get_logger().info(
            f"ar_pose_adapter started. map_frame={self.map_frame}, camera_frame={self.camera_frame}, "
            f"odom_topic={self.odom_topic}, rate={self.publish_rate_hz:.1f}Hz"
        )

    def on_timer(self) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.camera_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_lookup_timeout_sec),
            )
        except TransformException as ex:
            self.get_logger().warn(
                f"lookup map->camera failed: {ex}",
                throttle_duration_sec=2.0,
            )
            return

        odom = Odometry()
        odom.header.stamp = transform.header.stamp
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = self.camera_frame

        odom.pose.pose.position.x = transform.transform.translation.x
        odom.pose.pose.position.y = transform.transform.translation.y
        odom.pose.pose.position.z = transform.transform.translation.z

        odom.pose.pose.orientation = transform.transform.rotation
        odom.pose.covariance = self.pose_covariance

        self.odom_pub.publish(odom)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArPoseAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()