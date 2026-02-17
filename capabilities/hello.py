#!/usr/bin/env python3
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# 🔧 EDIT if your action name differs:
# ros2 action list | grep FollowJointTrajectory
TRAJ_ACTION_NAME = "/stretch_controller/follow_joint_trajectory"

JOINTS = [
    "joint_lift",
    "wrist_extension",
    "joint_wrist_yaw",
    "joint_wrist_pitch",
    "joint_wrist_roll",
    "joint_gripper_finger_left",
    "joint_gripper_finger_right",
]


def dur(t: float) -> Duration:
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


class GreetFlourishClap(Node):
    def __init__(self):
        super().__init__("greet_flourish_clap")
        self.client = ActionClient(self, FollowJointTrajectory, TRAJ_ACTION_NAME)

        self.get_logger().info(f"Waiting for action server: {TRAJ_ACTION_NAME}")
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                "FollowJointTrajectory server not available. "
                "Check: ros2 action list | grep FollowJointTrajectory"
            )
            raise RuntimeError("No trajectory action server")

        self.get_logger().info("Connected. Sending flourish + clap...")
        self.send_motion()

    def pt(self, t_s, lift, wrist_ext, yaw, pitch, roll, grip_l, grip_r):
        p = JointTrajectoryPoint()
        p.positions = [
            float(lift),
            float(wrist_ext),
            float(yaw),
            float(pitch),
            float(roll),
            float(grip_l),
            float(grip_r),
        ]
        p.time_from_start = dur(t_s)
        return p

    def send_motion(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS

        pts = []
        t = 0.0

        # ---- Pose setup (conservative) ----
        lift_up = 0.55
        wrist_ext = 0.06

        # base angles
        base_pitch = -0.65    # wrist bent down a bit
        base_roll  = 0.10     # tiny roll makes it feel like a "twist"

        # gripper values: these depend on your calibration; start small/safe.
        # If it doesn't move, increase range a bit.
        grip_open = 0.045
        grip_closed = 0.005

        # Move into greeting pose (open gripper)
        t += 0.9
        pts.append(self.pt(t, lift_up, wrist_ext, 0.0, base_pitch, base_roll, grip_open, grip_open))

        # ---- Flourish: upside-down parabola arc ----
        # We'll sweep yaw from -amp to +amp and shape pitch to make an ∩ arc.
        amp = 0.75            # yaw amplitude (bigger looks more like flourish than "no")
        curve = 0.35          # how much pitch rises at center (controls the "∩" height)
        roll_amp = 0.25       # twist vibe; roll follows yaw smoothly

        n = 9                 # number of points in the arc (higher = smoother)
        arc_duration = 1.8    # seconds total for flourish

        for i in range(n):
            # s in [-1, +1]
            s = -1.0 + 2.0 * (i / (n - 1))
            yaw = amp * s

            # upside-down parabola: peak at s=0, low at s=±1
            pitch = base_pitch + curve * (1.0 - s * s)

            # smooth twist: roll proportional to s
            roll = base_roll + roll_amp * s

            # Clap timing: close/open at a couple points during the flourish
            # (two "claps" total)
            if i in (2, 6):
                gl = gr = grip_closed
            else:
                gl = gr = grip_open

            t += arc_duration / (n - 1)
            pts.append(self.pt(t, lift_up, wrist_ext, yaw, pitch, roll, gl, gr))

        # ---- Thank-you clap: 3 quick claps in place ----
        # Keep pose steady, just open/close
        clap_count = 3
        clap_step = 0.28  # seconds per half-cycle

        for _ in range(clap_count):
            t += clap_step
            pts.append(self.pt(t, lift_up, wrist_ext, 0.0, base_pitch, base_roll, grip_closed, grip_closed))
            t += clap_step
            pts.append(self.pt(t, lift_up, wrist_ext, 0.0, base_pitch, base_roll, grip_open, grip_open))

        # Return neutral
        t += 1.0
        pts.append(self.pt(t, 0.45, 0.0, 0.0, 0.0, 0.0, grip_open, grip_open))

        goal.trajectory.points = pts
        self.client.send_goal_async(goal)
        self.get_logger().info("Flourish + clap goal sent.")


def main():
    rclpy.init()
    node = GreetFlourishClap()
    rclpy.spin_once(node, timeout_sec=1.0)
    time.sleep(10.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()