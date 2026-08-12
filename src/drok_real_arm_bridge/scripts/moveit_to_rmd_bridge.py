#!/usr/bin/env python3
import math
import socket
import struct
import time
from typing import Dict, List, Tuple

import rclpy
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node


CAN_FRAME_FMT = "=IB3x8s"
RAD2DEG = 180.0 / math.pi
JOINT_ORDER = ["JOINT1", "JOINT2", "JOINT3", "JOINT4", "JOINT5", "JOINT6"]
MotorKey = Tuple[str, int]


class MoveItToRMDBridge(Node):
    """New-model joint trajectory -> field-validated RMD motor commands.

    The new URDF/MuJoCo joint coordinates are consumed directly. Only the proven
    physical motor direction, raw HOME and gear conversion are applied here.
    """

    def __init__(self) -> None:
        super().__init__("moveit_to_rmd_bridge")

        self.declare_parameter("dry_run", True)
        self.declare_parameter("default_max_speed", 30)
        self.declare_parameter("gripper_max_speed", 50)
        self.declare_parameter("model_home_rad", [0.0] * 6)
        self.declare_parameter("primary_motor_ifaces", ["can10"] * 6)
        self.declare_parameter("primary_motor_ids", [321, 322, 324, 321, 322, 323])
        self.declare_parameter("primary_raw_home_deg", [0.0] * 6)
        self.declare_parameter("raw_per_model_sign", [1.0] * 6)
        self.declare_parameter("primary_gear_ratio", [1.0] * 6)
        self.declare_parameter("joint2_mirror_iface", "can10")
        self.declare_parameter("joint2_mirror_id", 323)
        self.declare_parameter("joint2_mirror_raw_home_deg", -0.03)
        self.declare_parameter("joint2_mirror_sign", -1.0)
        self.declare_parameter("joint2_mirror_gear_ratio", 36.0)
        self.declare_parameter("gripper_iface", "can11")
        self.declare_parameter("gripper_id", 324)
        self.declare_parameter("gripper_raw_home_deg", -18.128611166667)
        self.declare_parameter("gripper_raw_close_deg", 69.592500000000)
        self.declare_parameter("gripper_q_home", -1.7)
        self.declare_parameter("gripper_q_close", 45.0)
        self.declare_parameter("gripper_gear_ratio", 1.0)

        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.default_max_speed = int(self.get_parameter("default_max_speed").value)
        self.gripper_max_speed = int(self.get_parameter("gripper_max_speed").value)
        self.model_home = [float(v) for v in self.get_parameter("model_home_rad").value]
        ifaces = [str(v) for v in self.get_parameter("primary_motor_ifaces").value]
        ids = [int(v) for v in self.get_parameter("primary_motor_ids").value]
        raw_home = [float(v) for v in self.get_parameter("primary_raw_home_deg").value]
        raw_sign = [float(v) for v in self.get_parameter("raw_per_model_sign").value]
        gear = [float(v) for v in self.get_parameter("primary_gear_ratio").value]

        if not (len(self.model_home) == len(ifaces) == len(ids) == len(raw_home) == len(raw_sign) == len(gear) == 6):
            raise ValueError("All arm command mapping arrays must contain exactly six values")

        self.primary_keys: List[MotorKey] = list(zip(ifaces, ids))
        self.primary_raw_home = raw_home
        self.primary_sign = raw_sign
        self.gear_ratio: Dict[MotorKey, float] = {
            key: value for key, value in zip(self.primary_keys, gear)
        }

        self.joint2_mirror_key: MotorKey = (
            str(self.get_parameter("joint2_mirror_iface").value),
            int(self.get_parameter("joint2_mirror_id").value),
        )
        self.joint2_mirror_raw_home = float(
            self.get_parameter("joint2_mirror_raw_home_deg").value
        )
        self.joint2_mirror_sign = float(self.get_parameter("joint2_mirror_sign").value)
        self.gear_ratio[self.joint2_mirror_key] = float(
            self.get_parameter("joint2_mirror_gear_ratio").value
        )

        self.gripper_key: MotorKey = (
            str(self.get_parameter("gripper_iface").value),
            int(self.get_parameter("gripper_id").value),
        )
        self.gripper_raw_home = float(
            self.get_parameter("gripper_raw_home_deg").value
        )
        self.gripper_raw_close = float(
            self.get_parameter("gripper_raw_close_deg").value
        )
        self.gripper_q_home = float(
            self.get_parameter("gripper_q_home").value
        )
        self.gripper_q_close = float(
            self.get_parameter("gripper_q_close").value
        )
        self.gear_ratio[self.gripper_key] = float(
            self.get_parameter("gripper_gear_ratio").value
        )

        self.sockets: Dict[str, socket.socket] = {}
        if self.dry_run:
            self.get_logger().warn("DRY RUN: CAN command sockets are not opened.")
        else:
            self.get_logger().error("REAL SEND MODE ENABLED. Motors may move.")
            self._open_can_sockets(sorted({key[0] for key in self.gear_ratio}))

        self.arm_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
        )
        self.gripper_server = ActionServer(
            self,
            GripperCommand,
            "/gripper_controller/gripper_cmd",
            execute_callback=self.execute_gripper_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
        )

        self.get_logger().info("Coordinate contract: trajectory q == NEW URDF/MuJoCo q")
        self.get_logger().info("Action server ready: /arm_controller/follow_joint_trajectory")
        self.get_logger().info("Action server ready: /gripper_controller/gripper_cmd")

    def goal_cb(self, _goal_request):
        return GoalResponse.ACCEPT

    def cancel_cb(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _open_can_sockets(self, ifaces: List[str]) -> None:
        for iface in ifaces:
            sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            sock.bind((iface,))
            self.sockets[iface] = sock
            self.get_logger().info(f"Opened command CAN socket: {iface}")

    def _q_to_raw_deg(self, q_by_joint: Dict[str, float]) -> Dict[MotorKey, float]:
        raw: Dict[MotorKey, float] = {}
        for index, joint in enumerate(JOINT_ORDER):
            delta_deg = (q_by_joint[joint] - self.model_home[index]) * RAD2DEG
            raw[self.primary_keys[index]] = (
                self.primary_raw_home[index] + self.primary_sign[index] * delta_deg
            )

        q2_delta_deg = (q_by_joint["JOINT2"] - self.model_home[1]) * RAD2DEG
        raw[self.joint2_mirror_key] = (
            self.joint2_mirror_raw_home + self.joint2_mirror_sign * q2_delta_deg
        )
        return raw

    def _raw_deg_to_counts(self, key: MotorKey, raw_deg: float) -> int:
        return int(round(raw_deg * self.gear_ratio[key] / 0.01))

    def _send_rmd_position(self, key: MotorKey, raw_deg: float, speed: int) -> None:
        iface, motor_id = key
        counts = self._raw_deg_to_counts(key, raw_deg)
        if self.dry_run:
            return

        data = bytearray(8)
        data[0] = 0xA4
        data[1] = 0x00
        data[2] = speed & 0xFF
        data[3] = (speed >> 8) & 0xFF
        data[4] = counts & 0xFF
        data[5] = (counts >> 8) & 0xFF
        data[6] = (counts >> 16) & 0xFF
        data[7] = (counts >> 24) & 0xFF
        frame = struct.pack(CAN_FRAME_FMT, motor_id, 8, bytes(data))
        self.sockets[iface].send(frame)

    @staticmethod
    def _point_to_qdict(point, trajectory_joint_names: List[str]) -> Dict[str, float]:
        index = {name: i for i, name in enumerate(trajectory_joint_names)}
        return {joint: float(point.positions[index[joint]]) for joint in JOINT_ORDER}

    @staticmethod
    def _time_from_start(point) -> float:
        return point.time_from_start.sec + point.time_from_start.nanosec * 1e-9

    def _print_summary(self, q_first, q_last, raw_first, raw_last) -> None:
        print("\n========== NEW MODEL -> VERIFIED HARDWARE ==========")
        for joint in JOINT_ORDER:
            print(f"{joint}: {q_first[joint]:+.6f} -> {q_last[joint]:+.6f} rad")
        print("--- raw motor delta ---")
        for key in raw_first:
            print(f"{key[0]} 0x{key[1]:X}: {raw_last[key]-raw_first[key]:+.6f} deg")
        print("====================================================\n")

    def execute_cb(self, goal_handle):
        result = FollowJointTrajectory.Result()
        trajectory = goal_handle.request.trajectory
        missing = [joint for joint in JOINT_ORDER if joint not in trajectory.joint_names]
        if not trajectory.points or missing:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = f"empty trajectory or missing joints: {missing}"
            return result

        q_points = [self._point_to_qdict(point, trajectory.joint_names) for point in trajectory.points]
        raw_points = [self._q_to_raw_deg(q_point) for q_point in q_points]
        self._print_summary(q_points[0], q_points[-1], raw_points[0], raw_points[-1])

        if self.dry_run:
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            result.error_string = "dry_run success"
            return result

        start = time.monotonic()
        for point, raw in zip(trajectory.points, raw_points):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "cancelled"
                return result

            wait = start + self._time_from_start(point) - time.monotonic()
            if wait > 0.0:
                time.sleep(wait)

            # One waypoint contains all motors. Frames are emitted back-to-back so every
            # motor follows the same normalized Poly5 progress and finishes together.
            for key, raw_deg in raw.items():
                self._send_rmd_position(key, raw_deg, self.default_max_speed)

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "success"
        return result

    def execute_gripper_cb(self, goal_handle):
        result = GripperCommand.Result()

        q_requested = float(goal_handle.request.command.position)
        q_low = min(self.gripper_q_home, self.gripper_q_close)
        q_high = max(self.gripper_q_home, self.gripper_q_close)
        q = max(q_low, min(q_high, q_requested))

        q_span = self.gripper_q_close - self.gripper_q_home
        if abs(q_span) < 1.0e-12:
            goal_handle.abort()
            result.position = q
            result.effort = 0.0
            result.stalled = True
            result.reached_goal = False
            return result

        # Exact inverse of real_joint_state_mapper._gripper_cb().
        ratio = (q - self.gripper_q_home) / q_span
        ratio = max(0.0, min(1.0, ratio))
        raw_output_deg = (
            self.gripper_raw_home
            + ratio * (self.gripper_raw_close - self.gripper_raw_home)
        )

        self._send_rmd_position(
            self.gripper_key,
            raw_output_deg,
            self.gripper_max_speed,
        )

        self.get_logger().info(
            "JOINT7 command: "
            f"q={q:.6f}, ratio={ratio:.6f}, "
            f"output={raw_output_deg:.6f} deg, gear=6"
        )

        goal_handle.succeed()
        result.position = q
        result.effort = 0.0
        result.stalled = False
        result.reached_goal = True
        return result


def main() -> None:
    rclpy.init()
    node = MoveItToRMDBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
