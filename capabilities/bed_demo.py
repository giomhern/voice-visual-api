#!/usr/bin/env python3
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger


TRAJ_ACTION = "/stretch_controller/follow_joint_trajectory"
CMD_VEL_TOPIC = "/stretch/cmd_vel"

SWITCH_TO_TRAJ_SRV = "/switch_to_trajectory_mode"
SWITCH_TO_NAV_SRV = "/switch_to_navigation_mode"


def duration_msg(t: float) -> Duration:
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


@dataclass
class Pose:
    wrist_extension: float = 0.20
    joint_lift: float = 0.35
    joint_wrist_yaw: float = 0.0
    joint_wrist_pitch: float = -0.30
    joint_wrist_roll: float = 0.0
    joint_gripper_finger_left: float = 0.05  # open-ish


class AirGraspDemo(Node):
    def __init__(self):
        super().__init__("air_grasp_demo_modes_two_waypoints")

        self.traj_client = ActionClient(self, FollowJointTrajectory, TRAJ_ACTION)
        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        self.srv_traj = self.create_client(Trigger, SWITCH_TO_TRAJ_SRV)
        self.srv_nav = self.create_client(Trigger, SWITCH_TO_NAV_SRV)

        self.joint_names = [
            "wrist_extension",
            "joint_lift",
            "joint_wrist_yaw",
            "joint_wrist_pitch",
            "joint_wrist_roll",
            "joint_gripper_finger_left",
        ]

    # ---------- services ----------
    def _call_trigger(self, client, name: str, wait_s: float = 8.0, timeout_s: float = 12.0):
        t0 = time.time()
        while not client.wait_for_service(timeout_sec=0.5):
            if time.time() - t0 > wait_s:
                raise RuntimeError(f"Timed out waiting for service: {name}")
            self.get_logger().info(f"Waiting for {name}...")

        fut = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_s)
        resp = fut.result()
        if resp is None:
            raise RuntimeError(f"Service call failed/timeout: {name}")
        if not resp.success:
            raise RuntimeError(f"{name} returned success=False: {resp.message}")
        time.sleep(0.25)

    def switch_to_trajectory_mode(self):
        self.get_logger().info("[MODE] switching to trajectory")
        self._call_trigger(self.srv_traj, SWITCH_TO_TRAJ_SRV)

    def switch_to_navigation_mode(self):
        self.get_logger().info("[MODE] switching to navigation")
        self._call_trigger(self.srv_nav, SWITCH_TO_NAV_SRV)

    # ---------- motion ----------
    def wait_for_traj(self, timeout_s: float = 8.0):
        if not self.traj_client.wait_for_server(timeout_sec=timeout_s):
            raise RuntimeError(f"Trajectory action not available: {TRAJ_ACTION}")

    def _send_traj_two_points(self, positions_start: List[float], positions_goal: List[float], move_time_s: float):
        if len(positions_start) != len(self.joint_names) or len(positions_goal) != len(self.joint_names):
            raise ValueError("positions length must match joint_names length")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.joint_names)

        # IMPORTANT: don't set a future stamp
        goal.trajectory.header.stamp.sec = 0
        goal.trajectory.header.stamp.nanosec = 0

        p0 = JointTrajectoryPoint()
        p0.positions = list(positions_start)
        p0.time_from_start = duration_msg(0.0)

        p1 = JointTrajectoryPoint()
        p1.positions = list(positions_goal)
        p1.time_from_start = duration_msg(move_time_s)

        goal.trajectory.points = [p0, p1]

        send_fut = self.traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=20.0)
        gh = send_fut.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("Trajectory goal rejected")

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=40.0)

        result = res_fut.result().result
        ok = int(result.error_code) == 0
        err_str = getattr(result, "error_string", "")
        if not ok:
            raise RuntimeError(f"Trajectory failed: code={result.error_code} err='{err_str}'")
        time.sleep(0.20)

    def send_pose(self, current: Pose, target: Pose, move_time_s: float = 1.2) -> Pose:
        start = [
            current.wrist_extension,
            current.joint_lift,
            current.joint_wrist_yaw,
            current.joint_wrist_pitch,
            current.joint_wrist_roll,
            current.joint_gripper_finger_left,
        ]
        goal = [
            target.wrist_extension,
            target.joint_lift,
            target.joint_wrist_yaw,
            target.joint_wrist_pitch,
            target.joint_wrist_roll,
            target.joint_gripper_finger_left,
        ]
        self._send_traj_two_points(start, goal, move_time_s)
        return target

    def drive_forward(self, v: float, duration_s: float):
        if duration_s <= 0:
            return
        msg = Twist()
        msg.linear.x = float(v)
        t0 = time.time()
        while time.time() - t0 < duration_s:
            self.cmd_pub.publish(msg)
            time.sleep(0.05)
        self.cmd_pub.publish(Twist())
        time.sleep(0.15)

    # ---------- demo ----------
    def run(self):
        self.wait_for_traj()

        # Tune
        forward_each_step_s = 1.0   # set 0 to disable base motion
        forward_speed = 0.05

        grip_open = 0.05
        grip_closed = 0.01

        yaw_right = +0.70
        yaw_mid = 0.00
        yaw_left = -0.70

        pose = Pose(
            wrist_extension=0.20,
            joint_lift=0.35,
            joint_wrist_yaw=0.0,
            joint_wrist_pitch=-0.30,
            joint_wrist_roll=0.0,
            joint_gripper_finger_left=grip_open,
        )

        # RIGHT (start): open -> close
        self.switch_to_trajectory_mode()
        self.get_logger().info("[DEMO] Go RIGHT open")
        pose = self.send_pose(pose, Pose(**{**pose.__dict__, "joint_wrist_yaw": yaw_right, "joint_gripper_finger_left": grip_open}), 1.5)

        self.get_logger().info("[DEMO] RIGHT close")
        pose = self.send_pose(pose, Pose(**{**pose.__dict__, "joint_wrist_yaw": yaw_right, "joint_gripper_finger_left": grip_closed}), 0.8)

        # Drive to MIDDLE while holding closed
        self.switch_to_navigation_mode()
        self.get_logger().info("[DEMO] Drive forward (step 1)")
        self.drive_forward(forward_speed, forward_each_step_s)

        # MIDDLE: (still closed) -> open -> close
        self.switch_to_trajectory_mode()
        self.get_logger().info("[DEMO] Move to MIDDLE while closed")
        pose = self.send_pose(pose, Pose(**{**pose.__dict__, "joint_wrist_yaw": yaw_mid, "joint_gripper_finger_left": grip_closed}), 1.2)

        self.get_logger().info("[DEMO] MIDDLE release (open)")
        pose = self.send_pose(pose, Pose(**{**pose.__dict__, "joint_wrist_yaw": yaw_mid, "joint_gripper_finger_left": grip_open}), 0.6)

        self.get_logger().info("[DEMO] MIDDLE close")
        pose = self.send_pose(pose, Pose(**{**pose.__dict__, "joint_wrist_yaw": yaw_mid, "joint_gripper_finger_left": grip_closed}), 0.8)

        # Drive to LEFT while holding closed
        self.switch_to_navigation_mode()
        self.get_logger().info("[DEMO] Drive forward (step 2)")
        self.drive_forward(forward_speed, forward_each_step_s)

        # LEFT: (still closed) -> open
        self.switch_to_trajectory_mode()
        self.get_logger().info("[DEMO] Move to LEFT while closed")
        pose = self.send_pose(pose, Pose(**{**pose.__dict__, "joint_wrist_yaw": yaw_left, "joint_gripper_finger_left": grip_closed}), 1.2)

        self.get_logger().info("[DEMO] LEFT release (open)")
        pose = self.send_pose(pose, Pose(**{**pose.__dict__, "joint_wrist_yaw": yaw_left, "joint_gripper_finger_left": grip_open}), 0.6)

        # Return neutral
        self.get_logger().info("[DEMO] Return neutral (open)")
        pose = self.send_pose(pose, Pose(**{**pose.__dict__, "joint_wrist_yaw": 0.0, "joint_gripper_finger_left": grip_open}), 1.2)

        self.get_logger().info("[DEMO] Done")


def main():
    rclpy.init()
    node = AirGraspDemo()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()