#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# 🔧 EDIT THIS after running:
# ros2 action list | grep FollowJointTrajectory
TRAJ_ACTION_NAME = "/stretch_controller/follow_joint_trajectory"

JOINTS = [
    "joint_lift",
    "wrist_extension",
    "joint_wrist_pitch",
    "joint_wrist_yaw",
]


def dur(t: float) -> Duration:
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


class WaveGreeting(Node):
    def __init__(self):
        super().__init__("wave_greeting")
        self.client = ActionClient(self, FollowJointTrajectory, TRAJ_ACTION_NAME)

        self.get_logger().info(f"Waiting for action server: {TRAJ_ACTION_NAME}")
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                "FollowJointTrajectory server not available.\n"
                "Run: ros2 action list | grep FollowJointTrajectory\n"
                "Then set TRAJ_ACTION_NAME to one of those."
            )
            raise RuntimeError("No trajectory action server")

        self.get_logger().info("Connected. Sending wave trajectory...")
        self.send_wave()

    def make_point(self, t_s: float, lift: float, wrist_ext: float, wrist_pitch: float, wrist_yaw: float):
        p = JointTrajectoryPoint()
        p.positions = [float(lift), float(wrist_ext), float(wrist_pitch), float(wrist_yaw)]
        p.time_from_start = dur(t_s)
        return p

    def send_wave(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS

        pts = []
        t = 0.0

        # Conservative pose values (tune later)
        lift_up = 0.55
        wrist_ext = 0.05
        wrist_pitch = -0.6

        # 1) Move into greeting pose
        t += 0.8
        pts.append(self.make_point(t, lift_up, wrist_ext, wrist_pitch, 0.0))

        # 2) Wave left-right a few times using wrist yaw
        amp = 0.55   # radians (reduce if too wide)
        step = 0.45  # seconds between peaks (increase to slow down)

        for yaw in (+amp, -amp, +amp, -amp, +amp):
            t += step
            pts.append(self.make_point(t, lift_up, wrist_ext, wrist_pitch, yaw))

        # 3) Return to neutral
        t += 0.8
        pts.append(self.make_point(t, 0.45, 0.0, 0.0, 0.0))

        goal.trajectory.points = pts

        # Send (fire-and-forget)
        self.client.send_goal_async(goal)
        self.get_logger().info("Wave goal sent.")


def main():
    rclpy.init()
    node = WaveGreeting()
    # Let it stay alive long enough to send the goal and start motion
    rclpy.spin_once(node, timeout_sec=1.0)
    time.sleep(7.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()