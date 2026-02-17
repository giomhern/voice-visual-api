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
                "FollowJointTrajectory server not available. "
                "Double-check TRAJ_ACTION_NAME with: ros2 action list | grep FollowJointTrajectory"
            )
            raise RuntimeError("No trajectory action server")

        self.get_logger().info("Connected. Sending wave...")
        self.send_wave()

    def send_wave(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS

        pts = []

        # 0) Neutral-ish starting pose (small lift, slight reach, wrist bent)
        # NOTE: these are conservative values; tune if you want bigger motion.
        t = 0.0
        pts.append(self.pt(
            t += 0.8,
            lift=0.55,
            wrist_ext=0.05,
            wrist_pitch=-0.6,
            wrist_yaw=0.0
        ))

        # Wave oscillations (yaw left-right-left-right-left)
        amp = 0.55  # radians (~31 degrees). Reduce if too wide.
        step = 0.45 # seconds between peaks (speed of wave)

        pts.append(self.pt(t += step, lift=0.55, wrist_ext=0.05, wrist_pitch=-0.6, wrist_yaw=+amp))
        pts.append(self.pt(t += step, lift=0.55, wrist_ext=0.05, wrist_pitch=-0.6, wrist_yaw=-amp))
        pts.append(self.pt(t += step, lift=0.55, wrist_ext=0.05, wrist_pitch=-0.6, wrist_yaw=+amp))
        pts.append(self.pt(t += step, lift=0.55, wrist_ext=0.05, wrist_pitch=-0.6, wrist_yaw=-amp))
        pts.append(self.pt(t += step, lift=0.55, wrist_ext=0.05, wrist_pitch=-0.6, wrist_yaw=+amp))

        # Return to neutral
        pts.append(self.pt(
            t += 0.8,
            lift=0.45,
            wrist_ext=0.0,
            wrist_pitch=0.0,
            wrist_yaw=0.0
        ))

        goal.trajectory.points = pts

        # Send goal (fire-and-forget)
        self.client.send_goal_async(goal)
        self.get_logger().info("Wave goal sent.")

    def pt(self, t, lift, wrist_ext, wrist_pitch, wrist_yaw):
        p = JointTrajectoryPoint()
        p.positions = [float(lift), float(wrist_ext), float(wrist_pitch), float(wrist_yaw)]
        p.time_from_start = dur(t)
        return p


def main():
    rclpy.init()
    node = WaveGreeting()
    # Let it run long enough to send + execute the trajectory
    rclpy.spin_once(node, timeout_sec=6.0)
    time.sleep(2.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()