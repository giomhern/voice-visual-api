#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


TRAJ_ACTION_NAME = "/stretch_controller/follow_joint_trajectory"

JOINTS = [
    "joint_lift",
    "wrist_extension",
    "joint_wrist_yaw",     # keep ~0 (no left/right)
    "joint_wrist_pitch",   # slight bend for readability
    "joint_wrist_roll",    # twist happens here
    "joint_gripper_finger_left",
]


def dur(t: float) -> Duration:
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


class TwistClapGreeting(Node):
    def __init__(self):
        super().__init__("twist_clap_greeting")
        self.client = ActionClient(self, FollowJointTrajectory, TRAJ_ACTION_NAME)

        self.get_logger().info(f"Waiting for action server: {TRAJ_ACTION_NAME}")
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                "FollowJointTrajectory server not available. "
                "Double-check TRAJ_ACTION_NAME with: ros2 action list | grep FollowJointTrajectory"
            )
            raise RuntimeError("No trajectory action server")

        self.get_logger().info("Connected. Sending twist + clap greeting...")
        self.send_motion()

    def pt(self, t_s, lift, wrist_ext, yaw, pitch, roll, grip_l, grip_r):
        p = JointTrajectoryPoint()
        p.positions = [float(lift), float(wrist_ext), float(yaw), float(pitch), float(roll), float(grip_l), float(grip_r)]
        p.time_from_start = dur(t_s)
        return p

    def send_motion(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS

        pts = []
        t = 0.0

        # Pose
        lift_up = 0.55
        wrist_ext = 0.06
        yaw_hold = 0.0
        pitch_hold = -0.55   # slight bend
        roll_center = 0.0

        # Gripper (tune if needed)
        grip_open = 0.045
        grip_closed = 0.005

        # 1) Move into greeting pose
        t += 0.9
        pts.append(self.pt(t, lift_up, wrist_ext, yaw_hold, pitch_hold, roll_center, grip_open, grip_open))

        # 2) Wrist twist flourish (in-place roll oscillation)
        # This should read as "twist" not "no"
        roll_amp = 1.0       # radians (~57°). Try 0.6 if too much.
        step = 0.35          # seconds per twist point

        # sequence: center -> + -> center -> - -> center -> +
        rolls = [0.0, +roll_amp, 0.0, -roll_amp, 0.0, +roll_amp, 0.0]
        for r in rolls:
            t += step
            pts.append(self.pt(t, lift_up, wrist_ext, yaw_hold, pitch_hold, r, grip_open, grip_open))

        # 3) "Thank you" claps (stay still, just gripper open/close)
        clap_count = 3
        clap_step = 0.25
        for _ in range(clap_count):
            t += clap_step
            pts.append(self.pt(t, lift_up, wrist_ext, yaw_hold, pitch_hold, 0.0, grip_closed, grip_closed))
            t += clap_step
            pts.append(self.pt(t, lift_up, wrist_ext, yaw_hold, pitch_hold, 0.0, grip_open, grip_open))

        # 4) Return neutral
        t += 1.0
        pts.append(self.pt(t, 0.45, 0.0, 0.0, 0.0, 0.0, grip_open, grip_open))

        goal.trajectory.points = pts
        self.client.send_goal_async(goal)
        self.get_logger().info("Twist + clap goal sent.")


def main():
    rclpy.init()
    node = TwistClapGreeting()
    rclpy.spin_once(node, timeout_sec=1.0)
    time.sleep(9.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()