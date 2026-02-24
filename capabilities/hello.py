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


class HelloGreeting(Node):
    def __init__(self):
        super().__init__("hello_greeting")
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

    def pt(self, t_s, *positions):
        p = JointTrajectoryPoint()
        p.positions = [float(x) for x in positions]
        p.time_from_start = dur(t_s)
        return p

    def run(self):

        # =========================
        # GOAL A: Twist flourish
        # =========================
        arm_joints = [
            "joint_lift",
            "wrist_extension",
            "joint_wrist_yaw",
            "joint_wrist_pitch",
            "joint_wrist_roll",
        ]

        lift_up = 0.85
        wrist_ext = 0.06
        yaw_hold = 0.0
        pitch_hold = -0.55
        roll_amp = 1.0

        t = 0.0
        ptsA = []

        t += 0.8
        ptsA.append(self.pt(t, lift_up, wrist_ext, yaw_hold, pitch_hold, 0.0))

        for r in (+roll_amp, -roll_amp, +roll_amp, -roll_amp):
            t += 0.5
            ptsA.append(self.pt(t, lift_up, wrist_ext, yaw_hold, pitch_hold, r))

        # Reset wrist roll
        t += 0.8
        ptsA.append(self.pt(t, lift_up, wrist_ext, yaw_hold, pitch_hold, 0.0))

        self.get_logger().info("Sending twist flourish...")
        if not self.send_goal_and_wait(arm_joints, ptsA, timeout_sec=10.0):
            return

        # =========================
        # GOAL B: 3 Claps
        # =========================
        grip_joint = ["joint_gripper_finger_left"]

        grip_open = 0.045
        grip_closed = 0.005

        t = 0.0
        ptsB = []

        for _ in range(3):
            t += 0.35
            ptsB.append(self.pt(t, grip_closed))
            t += 0.35
            ptsB.append(self.pt(t, grip_open))

        self.get_logger().info("Sending claps...")
        self.send_goal_and_wait(grip_joint, ptsB, timeout_sec=10.0)

        self.get_logger().info("Hello gesture complete.")


def main():
    rclpy.init()
    node = HelloGreeting()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()