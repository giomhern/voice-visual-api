#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


TRAJ_ACTION_NAME = "/stretch_controller/follow_joint_trajectory"


def dur(t: float) -> Duration:
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


class TwoGoalTwistClap(Node):
    def __init__(self):
        super().__init__("two_goal_twist_clap")
        self.client = ActionClient(self, FollowJointTrajectory, TRAJ_ACTION_NAME)

        self.get_logger().info(f"Waiting for action server: {TRAJ_ACTION_NAME}")
        if not self.client.wait_for_server(timeout_sec=8.0):
            raise RuntimeError("No trajectory action server")

        self.run()

    def send_goal_and_wait(self, joint_names, points, timeout_sec=12.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names
        goal.trajectory.points = points

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=3.0)
        gh = send_future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error("Goal rejected.")
            return False

        res_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_future, timeout_sec=timeout_sec)
        return True

    def run(self):
        # -------------------------
        # GOAL A: twist + reset
        # (NO GRIPPER to avoid timeout)
        # Keep it short (<10s)
        # -------------------------
        arm_joints = [
            "joint_lift",
            "wrist_extension",
            "joint_wrist_yaw",
            "joint_wrist_pitch",
            "joint_wrist_roll",
        ]

        lift_up = 0.55
        wrist_ext = 0.06
        yaw_hold = 0.0
        pitch_hold = -0.55

        roll_amp = 1.0
        step = 0.5

        t = 0.0
        ptsA = []

        def ptA(t_s, lift, ext, yaw, pitch, roll):
            p = JointTrajectoryPoint()
            p.positions = [float(lift), float(ext), float(yaw), float(pitch), float(roll)]
            p.time_from_start = dur(t_s)
            return p

        t += 0.8
        ptsA.append(ptA(t, lift_up, wrist_ext, yaw_hold, pitch_hold, 0.0))

        for r in (+roll_amp, -roll_amp, +roll_amp, -roll_amp):
            t += step
            ptsA.append(ptA(t, lift_up, wrist_ext, yaw_hold, pitch_hold, r))

        # reset
        t += 0.8
        ptsA.append(ptA(t, lift_up, wrist_ext, 0.0, pitch_hold, 0.0))

        self.get_logger().info("Sending Goal A (twist+reset)...")
        okA = self.send_goal_and_wait(arm_joints, ptsA, timeout_sec=10.0)
        if not okA:
            self.get_logger().error("Goal A failed/timeout.")
            return

        # -------------------------
        # GOAL B: claps only (gripper joint only)
        # Short (<10s)
        # -------------------------
        grip_joint = ["joint_gripper_finger_left"]

        grip_open = 0.045
        grip_closed = 0.005

        t = 0.0
        ptsB = []

        def ptB(t_s, grip):
            p = JointTrajectoryPoint()
            p.positions = [float(grip)]
            p.time_from_start = dur(t_s)
            return p

        # 3 claps: close/open repeated
        clap_step = 0.35
        for _ in range(3):
            t += clap_step
            ptsB.append(ptB(t, grip_closed))
            t += clap_step
            ptsB.append(ptB(t, grip_open))

        self.get_logger().info("Sending Goal B (3 claps)...")
        okB = self.send_goal_and_wait(grip_joint, ptsB, timeout_sec=10.0)
        if not okB:
            self.get_logger().error(
                "Goal B failed/timeout. This strongly suggests the gripper should NOT be controlled via FollowJointTrajectory on your setup."
            )
            return

        self.get_logger().info("Done.")


def main():
    rclpy.init()
    node = TwoGoalTwistClap()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()