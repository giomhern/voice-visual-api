from __future__ import annotations

import math
import time
from typing import Dict, List, Optional

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from stretch_study.capabilities.base_motion import BaseMotion


class PickupDropSequence(Node):
    def __init__(self):
        super().__init__("pickup_drop_sequence")

        self.declare_parameter("cmd_vel_topic", "/stretch/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("traj_action", "/stretch_controller/follow_joint_trajectory")
        self.declare_parameter("switch_to_navigation_mode_service", "/switch_to_navigation_mode")
        self.declare_parameter("switch_to_position_mode_service", "/switch_to_position_mode")

        self.declare_parameter("turn_left_rad", math.pi / 2.0)
        self.declare_parameter("turn_timeout_s", 12.0)
        self.declare_parameter("drive_forward_m", 0.5)
        self.declare_parameter("drive_timeout_s", 20.0)
        self.declare_parameter("post_turn_settle_s", 1.0)
        self.declare_parameter("pre_drive_settle_s", 0.75)

        self.declare_parameter("lift_pre_pick_m", 0.57)
        self.declare_parameter("wrist_extension_goal_m", 0.16)
        self.declare_parameter("head_pan_goal_rad", 0.09023612385371113)
        self.declare_parameter("head_tilt_goal_rad", -1.0957223349494476)

        self.declare_parameter("wrist_yaw_neutral_rad", 0.0)
        self.declare_parameter("wrist_pitch_neutral_rad", 0.0)
        self.declare_parameter("wrist_roll_neutral_rad", 0.0)
        self.declare_parameter("wrist_yaw_stowed_rad", -1.57)

        self.declare_parameter("gripper_open_pos", 0.5339577172721249)
        self.declare_parameter("gripper_closed_pos", 0.0)

        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        self.traj_action = str(self.get_parameter("traj_action").value)
        self.turn_left_rad = float(self.get_parameter("turn_left_rad").value)
        self.turn_timeout_s = float(self.get_parameter("turn_timeout_s").value)
        self.drive_forward_m = float(self.get_parameter("drive_forward_m").value)
        self.drive_timeout_s = float(self.get_parameter("drive_timeout_s").value)
        self.post_turn_settle_s = float(self.get_parameter("post_turn_settle_s").value)
        self.pre_drive_settle_s = float(self.get_parameter("pre_drive_settle_s").value)

        self.lift_pre_pick_m = float(self.get_parameter("lift_pre_pick_m").value)
        self.wrist_extension_goal_m = float(self.get_parameter("wrist_extension_goal_m").value)
        self.head_pan_goal_rad = float(self.get_parameter("head_pan_goal_rad").value)
        self.head_tilt_goal_rad = float(self.get_parameter("head_tilt_goal_rad").value)
        self.wrist_yaw_neutral_rad = float(self.get_parameter("wrist_yaw_neutral_rad").value)
        self.wrist_pitch_neutral_rad = float(self.get_parameter("wrist_pitch_neutral_rad").value)
        self.wrist_roll_neutral_rad = float(self.get_parameter("wrist_roll_neutral_rad").value)
        self.wrist_yaw_stowed_rad = float(self.get_parameter("wrist_yaw_stowed_rad").value)
        self.gripper_open_pos = float(self.get_parameter("gripper_open_pos").value)
        self.gripper_closed_pos = float(self.get_parameter("gripper_closed_pos").value)

        self._joint_state: Optional[JointState] = None
        self.create_subscription(JointState, self.joint_states_topic, self._on_joint_state, 10)

        self._traj_client = ActionClient(self, FollowJointTrajectory, self.traj_action)
        self._nav_mode_client = self.create_client(
            Trigger, str(self.get_parameter("switch_to_navigation_mode_service").value)
        )
        self._pos_mode_client = self.create_client(
            Trigger, str(self.get_parameter("switch_to_position_mode_service").value)
        )

        self.motion = BaseMotion(self, cmd_vel_topic=self.cmd_vel_topic, odom_topic=self.odom_topic)

    def _on_joint_state(self, msg: JointState) -> None:
        self._joint_state = msg

    def _wait_for_joint_state(self, timeout_s: float = 5.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout_s and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._joint_state is not None:
                return True
        self.get_logger().error(f"[SEQ] No joint states received on {self.joint_states_topic}")
        return False

    def _joint_map(self) -> Dict[str, float]:
        if self._joint_state is None:
            return {}
        return {
            name: float(pos)
            for name, pos in zip(self._joint_state.name, self._joint_state.position)
        }

    def _call_trigger(self, client, name: str, timeout_s: float = 8.0) -> bool:
        if not client.wait_for_service(timeout_sec=timeout_s):
            self.get_logger().error(f"[SEQ] Service unavailable: {name}")
            return False

        fut = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_s)
        if not fut.done():
            self.get_logger().error(f"[SEQ] Timed out calling {name}")
            return False

        try:
            resp = fut.result()
        except Exception as exc:
            self.get_logger().error(f"[SEQ] Service call failed for {name}: {exc}")
            return False

        ok = bool(resp.success)
        self.get_logger().info(f"[SEQ] {name}: success={ok} msg='{resp.message}'")
        time.sleep(0.25)
        return ok

    def _settle_base(self, settle_s: float) -> None:
        end_t = time.time() + max(0.0, settle_s)
        while time.time() < end_t and rclpy.ok():
            self.motion.stop()
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

    def _send_pose(self, joint_names: List[str], positions: List[float], duration_s: float = 2.0) -> bool:
        if len(joint_names) != len(positions):
            self.get_logger().error("[SEQ] joint_names and positions length mismatch")
            return False

        if not self._traj_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"[SEQ] Trajectory action unavailable: {self.traj_action}")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)
        goal.trajectory.header.stamp.sec = 0
        goal.trajectory.header.stamp.nanosec = 0

        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        sec_i = int(duration_s)
        nsec_i = int((duration_s - sec_i) * 1e9)
        pt.time_from_start = Duration(sec=sec_i, nanosec=nsec_i)
        goal.trajectory.points = [pt]

        self.get_logger().info(f"[SEQ] Sending pose {dict(zip(joint_names, positions))}")
        send_fut = self._traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=10.0)
        if not send_fut.done():
            self.get_logger().error("[SEQ] send_goal timed out")
            return False

        goal_handle = send_fut.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("[SEQ] Goal rejected")
            return False

        result_fut = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_fut, timeout_sec=max(15.0, duration_s + 10.0))
        if not result_fut.done():
            self.get_logger().error("[SEQ] result timed out")
            return False

        result = result_fut.result().result
        ok = int(result.error_code) == 0
        self.get_logger().info(
            f"[SEQ] Trajectory result error_code={result.error_code} error_string='{getattr(result, 'error_string', '')}'"
        )
        return ok

    def _open_gripper(self) -> bool:
        return self._send_pose(
            ["joint_gripper_finger_left"],
            [self.gripper_open_pos],
            duration_s=2.0,
        )

    def _close_gripper(self) -> bool:
        return self._send_pose(
            ["joint_gripper_finger_left"],
            [self.gripper_closed_pos],
            duration_s=2.0,
        )

    def run(self) -> bool:
        if not self._wait_for_joint_state():
            return False

        self.get_logger().info("[SEQ] Step 1: rotate left by 90 degrees")
        if not self._call_trigger(self._nav_mode_client, "switch_to_navigation_mode"):
            return False
        self.motion.turn_angle(self.turn_left_rad, timeout_s=self.turn_timeout_s)
        self._settle_base(self.post_turn_settle_s)

        self.get_logger().info("[SEQ] Step 2: raise lift to pre-pick height")
        if not self._call_trigger(self._pos_mode_client, "switch_to_position_mode"):
            return False
        if not self._send_pose(["joint_lift"], [self.lift_pre_pick_m], duration_s=2.5):
            return False

        self.get_logger().info("[SEQ] Step 3: set wrist to neutral")
        if not self._send_pose(
            ["joint_wrist_yaw", "joint_wrist_pitch", "joint_wrist_roll"],
            [self.wrist_yaw_neutral_rad, self.wrist_pitch_neutral_rad, self.wrist_roll_neutral_rad],
            duration_s=2.0,
        ):
            return False

        self.get_logger().info("[SEQ] Step 4: open gripper to max")
        if not self._open_gripper():
            return False

        self.get_logger().info("[SEQ] Step 5: extend arm to target reach pose while keeping current lift")
        current = self._joint_map()
        current_lift = float(current.get("joint_lift", self.lift_pre_pick_m))
        if not self._send_pose(
            [
                "joint_lift",
                "wrist_extension",
                "joint_wrist_yaw",
                "joint_head_pan",
                "joint_head_tilt",
                "joint_wrist_pitch",
                "joint_wrist_roll",
            ],
            [
                current_lift,
                self.wrist_extension_goal_m,
                self.wrist_yaw_neutral_rad,
                self.head_pan_goal_rad,
                self.head_tilt_goal_rad,
                self.wrist_pitch_neutral_rad,
                self.wrist_roll_neutral_rad,
            ],
            duration_s=3.0,
        ):
            return False

        self.get_logger().info("[SEQ] Step 6: close gripper to smallest")
        if not self._close_gripper():
            return False

        self.get_logger().info("[SEQ] Step 7: move forward to drag the grasped item")
        if not self._call_trigger(self._nav_mode_client, "switch_to_navigation_mode"):
            return False
        self._settle_base(self.pre_drive_settle_s)
        self.motion.drive_distance(self.drive_forward_m, timeout_s=self.drive_timeout_s)
        self._settle_base(0.5)

        self.get_logger().info("[SEQ] Step 8: open gripper to max to drop")
        if not self._call_trigger(self._pos_mode_client, "switch_to_position_mode"):
            return False
        if not self._open_gripper():
            return False

        self.get_logger().info("[SEQ] Step 9: retract arm, close gripper, and rotate wrist inward for safety")
        current = self._joint_map()
        current_lift = float(current.get("joint_lift", self.lift_pre_pick_m))
        current_head_pan = float(current.get("joint_head_pan", self.head_pan_goal_rad))
        current_head_tilt = float(current.get("joint_head_tilt", self.head_tilt_goal_rad))

        if not self._send_pose(
            [
                "joint_lift",
                "wrist_extension",
                "joint_wrist_yaw",
                "joint_head_pan",
                "joint_head_tilt",
                "joint_wrist_pitch",
                "joint_wrist_roll",
                "joint_gripper_finger_left",
            ],
            [
                current_lift,
                0.0,
                self.wrist_yaw_stowed_rad,
                current_head_pan,
                current_head_tilt,
                self.wrist_pitch_neutral_rad,
                self.wrist_roll_neutral_rad,
                self.gripper_closed_pos,
            ],
            duration_s=3.0,
        ):
            return False

        self.get_logger().info("[SEQ] Sequence complete")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = PickupDropSequence()
    ok = False
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
