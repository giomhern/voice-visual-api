#!/usr/bin/env python3
from __future__ import annotations

import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger


# ---- Adjust if your system differs ----
TRAJ_ACTION = "/stretch_controller/follow_joint_trajectory"
CMD_VEL_TOPIC = "/stretch/cmd_vel"

SWITCH_TO_TRAJ_SRV = "/switch_to_trajectory_mode"
SWITCH_TO_NAV_SRV = "/switch_to_navigation_mode"


def dur(t: float) -> Duration:
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


@dataclass
class Pose:
    # Joints you confirmed exist
    wrist_extension: float = 0.20
    joint_lift: float = 0.35
    joint_wrist_yaw: float = 0.0
    joint_wrist_pitch: float = -0.30
    joint_wrist_roll: float = 0.0
    # Only ONE finger joint (per your requirement)
    joint_gripper_finger_left: float = 0.05  # open-ish


class AirGraspDemo(Node):
    """
    Air-grasp demo:
      Start at RIGHT -> close gripper
      Switch to NAV -> drive forward
      Switch to TRAJ -> go to MIDDLE (still closed) -> open
      Close again
      Switch to NAV -> drive forward
      Switch to TRAJ -> go to LEFT (still closed) -> open
      Return yaw to neutral

    Handles mode switching using:
      /switch_to_trajectory_mode (Trigger)
      /switch_to_navigation_mode (Trigger)
    """

    def __init__(self):
        super().__init__("air_grasp_demo_modes_one_finger")

        self.traj_client = ActionClient(self, FollowJointTrajectory, TRAJ_ACTION)
        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        self.switch_traj = self.create_client(Trigger, SWITCH_TO_TRAJ_SRV)
        self.switch_nav = self.create_client(Trigger, SWITCH_TO_NAV_SRV)

    # -------------------------
    # Mode switching utilities
    # -------------------------
    def _call_trigger(self, client, name: str, timeout_s: float = 10.0):
        t0 = time.time()
        while not client.wait_for_service(timeout_sec=0.5):
            if time.time() - t0 > timeout_s:
                raise RuntimeError(f"Timed out waiting for service: {name}")
            self.get_logger().info(f"Waiting for service {name}...")

        fut = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_s)
        resp = fut.result()
        if resp is None:
            raise RuntimeError(f"Service call failed/timeout: {name}")
        if not resp.success:
            raise RuntimeError(f"{name} returned success=False: {resp.message}")

    def switch_to_trajectory_mode(self):
        self.get_logger().info("Switching to TRAJECTORY mode...")
        self._call_trigger(self.switch_traj, SWITCH_TO_TRAJ_SRV)
        # small settle
        time.sleep(0.2)

    def switch_to_navigation_mode(self):
        self.get_logger().info("Switching to NAVIGATION mode...")
        self._call_trigger(self.switch_nav, SWITCH_TO_NAV_SRV)
        # small settle
        time.sleep(0.2)

    # -------------------------
    # Trajectory utilities
    # -------------------------
    def wait_for_traj_action(self, timeout_s: float = 10.0) -> bool:
        return self.traj_client.wait_for_server(timeout_sec=timeout_s)

    def send_pose(self, pose: Pose, move_time_s: float = 1.2):
        """
        Sends a short single-point trajectory.
        Many Stretch controllers accept this, but if yours prefers 2 points,
        I can modify it to include a "hold" point at t=0.0.
        """
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            "wrist_extension",
            "joint_lift",
            "joint_wrist_yaw",
            "joint_wrist_pitch",
            "joint_wrist_roll",
            "joint_gripper_finger_left",
        ]

        p = JointTrajectoryPoint()
        p.positions = [
            pose.wrist_extension,
            pose.joint_lift,
            pose.joint_wrist_yaw,
            pose.joint_wrist_pitch,
            pose.joint_wrist_roll,
            pose.joint_gripper_finger_left,
        ]
        p.time_from_start = dur(move_time_s)
        goal.trajectory.points = [p]

        send_fut = self.traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut)
        gh = send_fut.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("Trajectory goal rejected / not accepted")

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)

    # -------------------------
    # Base driving
    # -------------------------
    def drive_forward(self, v: float = 0.05, duration_s: float = 1.0):
        if duration_s <= 0:
            return

        msg = Twist()
        msg.linear.x = float(v)

        t0 = time.time()
        while time.time() - t0 < duration_s:
            self.cmd_pub.publish(msg)
            time.sleep(0.05)

        self.cmd_pub.publish(Twist())
        time.sleep(0.1)

    # -------------------------
    # Demo routine
    # -------------------------
    def run(self):
        if not self.wait_for_traj_action():
            raise RuntimeError(f"Trajectory action server not available: {TRAJ_ACTION}")

        # ---- Tune these ----
        forward_each_step_s = 1.0   # set 0.0 to disable base forward motion
        forward_speed = 0.05

        # One-finger gripper values (tune if needed)
        grip_open = 0.05
        grip_closed = 0.01  # safer than 0.00 on many setups

        # Right/middle/left via wrist yaw
        yaw_right = +0.70
        yaw_mid = 0.00
        yaw_left = -0.70

        common = Pose(
            wrist_extension=0.20,
            joint_lift=0.35,
            joint_wrist_pitch=-0.30,
            joint_wrist_roll=0.0,
            joint_wrist_yaw=0.0,
            joint_gripper_finger_left=grip_open,
        )

        # ---- RIGHT (start) ----
        self.switch_to_trajectory_mode()
        self.get_logger().info("Go to RIGHT (open).")
        self.send_pose(Pose(**{**common.__dict__, "joint_wrist_yaw": yaw_right, "joint_gripper_finger_left": grip_open}), 1.5)

        self.get_logger().info("RIGHT: close (air grasp).")
        self.send_pose(Pose(**{**common.__dict__, "joint_wrist_yaw": yaw_right, "joint_gripper_finger_left": grip_closed}), 0.8)

        # ---- Drive forward ----
        self.switch_to_navigation_mode()
        self.get_logger().info("Drive forward (step 1).")
        self.drive_forward(v=forward_speed, duration_s=forward_each_step_s)

        # ---- MIDDLE ----
        self.switch_to_trajectory_mode()
        self.get_logger().info("Move to MIDDLE while holding closed.")
        self.send_pose(Pose(**{**common.__dict__, "joint_wrist_yaw": yaw_mid, "joint_gripper_finger_left": grip_closed}), 1.2)

        self.get_logger().info("MIDDLE: release (open).")
        self.send_pose(Pose(**{**common.__dict__, "joint_wrist_yaw": yaw_mid, "joint_gripper_finger_left": grip_open}), 0.6)

        self.get_logger().info("MIDDLE: close (air grasp).")
        self.send_pose(Pose(**{**common.__dict__, "joint_wrist_yaw": yaw_mid, "joint_gripper_finger_left": grip_closed}), 0.8)

        # ---- Drive forward ----
        self.switch_to_navigation_mode()
        self.get_logger().info("Drive forward (step 2).")
        self.drive_forward(v=forward_speed, duration_s=forward_each_step_s)

        # ---- LEFT ----
        self.switch_to_trajectory_mode()
        self.get_logger().info("Move to LEFT while holding closed.")
        self.send_pose(Pose(**{**common.__dict__, "joint_wrist_yaw": yaw_left, "joint_gripper_finger_left": grip_closed}), 1.2)

        self.get_logger().info("LEFT: release (open).")
        self.send_pose(Pose(**{**common.__dict__, "joint_wrist_yaw": yaw_left, "joint_gripper_finger_left": grip_open}), 0.6)

        # ---- Return neutral ----
        self.get_logger().info("Return yaw to neutral (open).")
        self.send_pose(Pose(**{**common.__dict__, "joint_wrist_yaw": 0.0, "joint_gripper_finger_left": grip_open}), 1.2)

        self.get_logger().info("Air grasp demo complete.")


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