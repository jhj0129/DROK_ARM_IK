#!/usr/bin/env python3
import math
from typing import Dict, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


JOINT_NAMES = ["JOINT1", "JOINT2", "JOINT3", "JOINT4", "JOINT5", "JOINT6"]


class RealJointStateMapper(Node):
    """Map verified raw RMD feedback into the NEW URDF/MuJoCo joint coordinates.

    Coordinate contract:
      q_model = q_model_home + raw_per_model_sign * (raw - raw_home) [rad]

    No extra IK-to-real sign array is allowed downstream. /joint_states is already in
    the exact logical coordinates used by the new kinematic model and MuJoCo model.
    """

    def __init__(self) -> None:
        super().__init__("real_joint_state_mapper")

        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("frame_id", "ARM_BASE_LINK")
        self.declare_parameter("model_home_rad", [0.0] * 6)
        self.declare_parameter("motor_topics", [""] * 6)
        self.declare_parameter("raw_home_deg", [0.0] * 6)
        self.declare_parameter("raw_per_model_sign", [1.0] * 6)
        self.declare_parameter("gripper_topic", "/motor_angles/can11_motor_0x144")
        self.declare_parameter("gripper_raw_open_deg", -108.771667)
        self.declare_parameter("gripper_raw_close_deg", 417.555000)
        self.declare_parameter("gripper_q_open", -1.70000)
        self.declare_parameter("gripper_q_close", 45.00000)

        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.model_home = [float(v) for v in self.get_parameter("model_home_rad").value]
        self.motor_topics = [str(v) for v in self.get_parameter("motor_topics").value]
        self.raw_home = [float(v) for v in self.get_parameter("raw_home_deg").value]
        self.raw_sign = [float(v) for v in self.get_parameter("raw_per_model_sign").value]

        if not (
            len(self.model_home)
            == len(self.motor_topics)
            == len(self.raw_home)
            == len(self.raw_sign)
            == 6
        ):
            raise ValueError("All arm mapping arrays must contain exactly six values")

        self.gripper_raw_open = float(self.get_parameter("gripper_raw_open_deg").value)
        self.gripper_raw_close = float(self.get_parameter("gripper_raw_close_deg").value)
        self.gripper_q_open = float(self.get_parameter("gripper_q_open").value)
        self.gripper_q_close = float(self.get_parameter("gripper_q_close").value)

        self.q: Dict[str, float] = {
            name: home for name, home in zip(JOINT_NAMES, self.model_home)
        }
        self.received: Dict[str, bool] = {name: False for name in JOINT_NAMES}
        self.q7 = self.gripper_q_open
        self.received_q7 = False

        for index, topic in enumerate(self.motor_topics):
            self.create_subscription(
                Float64,
                topic,
                lambda msg, i=index: self._arm_angle_cb(msg, i),
                10,
            )
            self.get_logger().info(
                f"{topic} -> {JOINT_NAMES[index]} "
                f"(raw_home={self.raw_home[index]:.6f} deg, "
                f"model_home={self.model_home[index]:.9f} rad, "
                f"sign={self.raw_sign[index]:+.0f})"
            )

        gripper_topic = str(self.get_parameter("gripper_topic").value)
        self.create_subscription(Float64, gripper_topic, self._gripper_cb, 10)

        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.timer = self.create_timer(1.0 / max(self.publish_hz, 1.0), self._publish)

    def _arm_angle_cb(self, msg: Float64, index: int) -> None:
        raw_deg = float(msg.data)
        self.q[JOINT_NAMES[index]] = (
            self.model_home[index]
            + self.raw_sign[index] * math.radians(raw_deg - self.raw_home[index])
        )
        self.received[JOINT_NAMES[index]] = True

    def _gripper_cb(self, msg: Float64) -> None:
        denominator = self.gripper_raw_close - self.gripper_raw_open
        if abs(denominator) < 1e-9:
            return
        ratio = (float(msg.data) - self.gripper_raw_open) / denominator
        ratio = max(0.0, min(1.0, ratio))
        self.q7 = self.gripper_q_open + ratio * (
            self.gripper_q_close - self.gripper_q_open
        )
        self.received_q7 = True

    def _publish(self) -> None:
        if not all(self.received.values()):
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.name = JOINT_NAMES + ["JOINT7"]
        msg.position = [self.q[name] for name in JOINT_NAMES] + [self.q7]
        msg.velocity = [0.0] * 7
        msg.effort = [0.0] * 7
        self.publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = RealJointStateMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
