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


class GreetTwistClap(Node):
    def __init__(self):
        super().__init__("greet_twist_clap")
        self.client = ActionClient(self, FollowJointTrajectory, TRAJ_ACTION_NAME)

        self.get_logger().info(f"Waiting for action server: {TRAJ_ACTION_NAME}")
        if not self.client.wait_for_server(timeout_sec=8.0):
            raise RuntimeError("No FollowJointTrajectory action server. Check TRAJ_ACTION_NAME.")

        self.run()

    def send_goal_and_wait(self, joint_names, points, timeout_sec: float) -> bool:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)
        goal.trajectory.points = list(points)

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=3.0)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Goal rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)

        if not result_future.done():
            self.get_logger().error("Goal result timed out waiting.")
            return False

        return True

    def run(self):
        # -------------------------
        # GOAL A: Look at user + arm to greeting pose + wrist twists + reset
        # (NO GRIPPER HERE to avoid gripper-related timeouts)
        # Keep total time < 10s
        # -------------------------
        jointsA = [
            "joint_head_pan",
            "joint_head_tilt",
            "joint_lift",
            "wrist_extension",
            "joint_wrist_yaw",
            "joint_wrist_pitch",
            "joint_wrist_roll",
        ]

        # "Eye-level-ish" pose
        head_pan = 0.0
        head_tilt = -0.20       # negative often looks slightly down; tweak if needed

        lift_up = 0.60
        wrist_ext = 0.06

        yaw_hold = 0.0          # no left-right wave
        pitch_hold = -0.55      # readable wrist pose
        roll_amp = 1.00         # twist amount (rad)

        ptsA = []
        t = 0.0

        def ptA(t_s, hp, ht, lift, ext, yaw, pitch, roll):
            p = JointTrajectoryPoint()
            p.positions = [float(hp), float(ht), float(lift), float(ext), float(yaw), float(pitch), float(roll)]
            p.time_from_start = dur(t_s)
            return p

        # 1) Move to greeting posture
        t = 1.2
        ptsA.append(ptA(t, head_pan, head_tilt, lift_up, wrist_ext, yaw_hold, pitch_hold, 0.0))

        # 2) Two twists: +, -, +, -
        step = 0.55
        for r in (+roll_amp, -roll_amp, +roll_amp, -roll_amp):
            t += step
            ptsA.append(ptA(t, head_pan, head_tilt, lift_up, wrist_ext, yaw_hold, pitch_hold, r))

        # 3) Reset wrist to default before claps
        t += 0.80
        ptsA.append(ptA(t, head_pan, head_tilt, lift_up, wrist_ext, 0.0, pitch_hold, 0.0))

        self.get_logger().info("Sending Goal A (look + pose + twists + reset)...")
        okA = self.send_goal_and_wait(jointsA, ptsA, timeout_sec=10.0)
        if not okA:
            self.get_logger().error("Goal A failed (rejected or timeout).")
            return

        # -------------------------
        # GOAL B: 3 claps (GRIPPER ONLY)
        # Keep total time < 10s
        # -------------------------
        jointsB = ["joint_gripper_finger_left"]

        grip_open = 0.045
        grip_closed = 0.005

        ptsB = []
        t = 0.0

        def ptB(t_s, grip):
            p = JointTrajectoryPoint()
            p.positions = [float(grip)]
            p.time_from_start = dur(t_s)
            return p

        # Start at open
        t = 0.4
        ptsB.append(ptB(t, grip_open))

        clap_step = 0.35
        for _ in range(3):
            t += clap_step
            ptsB.append(ptB(t, grip_closed))
            t += clap_step
            ptsB.append(ptB(t, grip_open))

        self.get_logger().info("Sending Goal B (3 claps)...")
        okB = self.send_goal_and_wait(jointsB, ptsB, timeout_sec=10.0)
        if not okB:
            self.get_logger().error(
                "Goal B failed (rejected or timeout). If this happens, your gripper likely needs a different control interface than FollowJointTrajectory."
            )
            return

        # -------------------------
        # GOAL C: Return to neutral (optional but nice)
        # -------------------------
        jointsC = [
            "joint_head_pan",
            "joint_head_tilt",
            "joint_lift",
            "wrist_extension",
            "joint_wrist_yaw",
            "joint_wrist_pitch",
            "joint_wrist_roll",
        ]

        ptsC = []
        t = 0.0

        def ptC(t_s, hp, ht, lift, ext, yaw, pitch, roll):
            p = JointTrajectoryPoint()
            p.positions = [float(hp), float(ht), float(lift), float(ext), float(yaw), float(pitch), float(roll)]
            p.time_from_start = dur(t_s)
            return p

        t = 1.5
        ptsC.append(ptC(t, 0.0, 0.0, 0.45, 0.0, 0.0, 0.0, 0.0))

        self.get_logger().info("Sending Goal C (return neutral)...")
        _ = self.send_goal_and_wait(jointsC, ptsC, timeout_sec=6.0)

        self.get_logger().info("Done.")


def main():
    rclpy.init()
    node = GreetTwistClap()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()