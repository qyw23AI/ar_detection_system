#!/usr/bin/env python3

import math
from typing import Tuple

import rclpy
from geometry_msgs.msg import TransformStamped, Vector3
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class GimbalTfHandler(Node):
    def __init__(self) -> None:
        super().__init__("gimbal_tf_handler")

        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("require_ack", False)
        self.declare_parameter("cmd_topic", "/gimbal/target_cmd")
        self.declare_parameter("ack_topic", "/gimbal/ack")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("lidar_frame", "lidar_link")
        self.declare_parameter("motor_2006_base_frame", "motor_2006_base")
        self.declare_parameter("motor_2006_shaft_frame", "motor_2006_shaft")
        self.declare_parameter("motor_scs0009_base_frame", "motor_scs0009_base")
        self.declare_parameter("camera_link_frame", "camera_link")
        self.declare_parameter("camera_optical_frame", "camera_color_optical_frame")

        self.declare_parameter("lidar_to_motor_2006_base_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("lidar_to_motor_2006_base_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("motor_2006_shaft_to_scs0009_base_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("motor_2006_shaft_to_scs0009_base_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("camera_link_to_optical_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("camera_link_to_optical_rpy", [0.0, 0.0, 0.0])

        self.declare_parameter("pitch_min", -math.pi)
        self.declare_parameter("pitch_max", math.pi)
        self.declare_parameter("yaw_min", -math.pi)
        self.declare_parameter("yaw_max", math.pi)

        self.require_ack = bool(self.get_parameter("require_ack").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.ack_topic = str(self.get_parameter("ack_topic").value)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.lidar_frame = str(self.get_parameter("lidar_frame").value)
        self.motor_2006_base_frame = str(self.get_parameter("motor_2006_base_frame").value)
        self.motor_2006_shaft_frame = str(self.get_parameter("motor_2006_shaft_frame").value)
        self.motor_scs0009_base_frame = str(self.get_parameter("motor_scs0009_base_frame").value)
        self.camera_link_frame = str(self.get_parameter("camera_link_frame").value)
        self.camera_optical_frame = str(self.get_parameter("camera_optical_frame").value)

        self.pitch_min = float(self.get_parameter("pitch_min").value)
        self.pitch_max = float(self.get_parameter("pitch_max").value)
        self.yaw_min = float(self.get_parameter("yaw_min").value)
        self.yaw_max = float(self.get_parameter("yaw_max").value)

        self.static_lidar_to_motor_xyz = [float(v) for v in self.get_parameter("lidar_to_motor_2006_base_xyz").value]
        self.static_lidar_to_motor_rpy = [float(v) for v in self.get_parameter("lidar_to_motor_2006_base_rpy").value]
        self.static_shaft_to_scs_xyz = [
            float(v) for v in self.get_parameter("motor_2006_shaft_to_scs0009_base_xyz").value
        ]
        self.static_shaft_to_scs_rpy = [
            float(v) for v in self.get_parameter("motor_2006_shaft_to_scs0009_base_rpy").value
        ]
        self.static_camera_to_optical_xyz = [float(v) for v in self.get_parameter("camera_link_to_optical_xyz").value]
        self.static_camera_to_optical_rpy = [float(v) for v in self.get_parameter("camera_link_to_optical_rpy").value]

        self.pending_pitch = 0.0
        self.pending_yaw = 0.0
        self.applied_pitch = 0.0
        self.applied_yaw = 0.0
        self.waiting_ack = False

        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_sub = self.create_subscription(Vector3, self.cmd_topic, self.on_cmd, 10)
        self.ack_sub = self.create_subscription(Bool, self.ack_topic, self.on_ack, 10)

        timer_period = 1.0 / max(publish_rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.on_timer)

        self.get_logger().info(
            f"gimbal_tf_handler started. cmd_topic={self.cmd_topic}, ack_topic={self.ack_topic}, "
            f"require_ack={self.require_ack}, rate={publish_rate_hz:.1f}Hz"
        )

    def on_cmd(self, msg: Vector3) -> None:
        raw_pitch = float(msg.x)
        raw_yaw = float(msg.y)
        self.pending_pitch = max(self.pitch_min, min(self.pitch_max, raw_pitch))
        self.pending_yaw = max(self.yaw_min, min(self.yaw_max, raw_yaw))

        if self.require_ack:
            self.waiting_ack = True
            self.get_logger().info(
                f"received cmd pitch={self.pending_pitch:.4f}, yaw={self.pending_yaw:.4f}, waiting ack"
            )
            return

        self.applied_pitch = self.pending_pitch
        self.applied_yaw = self.pending_yaw

    def on_ack(self, msg: Bool) -> None:
        if not self.require_ack:
            return
        if not self.waiting_ack:
            return
        if not bool(msg.data):
            return

        self.applied_pitch = self.pending_pitch
        self.applied_yaw = self.pending_yaw
        self.waiting_ack = False
        self.get_logger().info(
            f"ack accepted, apply pitch={self.applied_pitch:.4f}, yaw={self.applied_yaw:.4f}"
        )

    def make_transform(
        self,
        now,
        parent: str,
        child: str,
        xyz: Tuple[float, float, float],
        rpy: Tuple[float, float, float],
    ) -> TransformStamped:
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = parent
        tf_msg.child_frame_id = child
        tf_msg.transform.translation.x = float(xyz[0])
        tf_msg.transform.translation.y = float(xyz[1])
        tf_msg.transform.translation.z = float(xyz[2])

        qx, qy, qz, qw = quaternion_from_euler(float(rpy[0]), float(rpy[1]), float(rpy[2]))
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        return tf_msg

    def on_timer(self) -> None:
        now = self.get_clock().now().to_msg()

        tf_lidar_to_motor = self.make_transform(
            now,
            self.lidar_frame,
            self.motor_2006_base_frame,
            tuple(self.static_lidar_to_motor_xyz),
            tuple(self.static_lidar_to_motor_rpy),
        )

        tf_pitch = self.make_transform(
            now,
            self.motor_2006_base_frame,
            self.motor_2006_shaft_frame,
            (0.0, 0.0, 0.0),
            (0.0, self.applied_pitch, 0.0),
        )

        tf_shaft_to_scs = self.make_transform(
            now,
            self.motor_2006_shaft_frame,
            self.motor_scs0009_base_frame,
            tuple(self.static_shaft_to_scs_xyz),
            tuple(self.static_shaft_to_scs_rpy),
        )

        tf_yaw = self.make_transform(
            now,
            self.motor_scs0009_base_frame,
            self.camera_link_frame,
            (0.0, 0.0, 0.0),
            (0.0, 0.0, self.applied_yaw),
        )

        tf_camera_to_optical = self.make_transform(
            now,
            self.camera_link_frame,
            self.camera_optical_frame,
            tuple(self.static_camera_to_optical_xyz),
            tuple(self.static_camera_to_optical_rpy),
        )

        self.tf_broadcaster.sendTransform(
            [tf_lidar_to_motor, tf_pitch, tf_shaft_to_scs, tf_yaw, tf_camera_to_optical]
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GimbalTfHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()