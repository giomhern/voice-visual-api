#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState


TRAJ_ACTION_NAME = "/stretch_controller/follow_joint_trajectory"


def dur(t: float) -> Duration:
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


class TwoGoalTwistClapFull(Node):
    def __init__(self):
        super().__init__("two_goal_twist_clap_full")
        self.client = ActionClient(self, FollowJointTrajectory, TRAJ_ACTION_NAME)

        # Grab one /joint_states snapshot for neutral offsets
        self.js_msg = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.get_logger().info("Waiting for one /joint_states message...")
        while rclpy.ok() and self.js_msg is None:
            rclpy.spin_once(self, timeout_sec=0.2)

        if self.js_msg is None:
            raise RuntimeError("Did not receive /joint_states")

        self.roll_neutral = self._pos("joint_wrist_roll")
        self.yaw_neutral = self._pos("joint_wrist_yaw")
        self.pitch_neutral = self._pos("joint_wrist_pitch")

        self.get_logger().info(
            f"Neutral captured: yaw={self.yaw_neutral:.3f}, pitch={self.pitch_neutral:.3f}, roll={self.roll_neutral:.3f}"
        )

        self.get_logger().info(f"Waiting for action server: {TRAJ_ACTION_NAME}")
        if not self.client.wait_for_server(timeout_sec=8.0):
            raise RuntimeError("No trajectory action server. Check TRAJ_ACTION_NAME.")

        self.run()

    def _on_js(self, msg: JointState):
        self.js_msg = msg

    def _pos(self, joint_name: str) -> float:
        idx = self.js_msg.name.index(joint_name)
        return float(self.js_msg.position[idx])

    def send_goal_and_wait(self, joint_names, points, timeout_sec=12.0) -> bool:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)
        goal.trajectory.points = list(points)

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=3.0)
        gh = send_future.result()

        if gh is None or not gh.accepted:
            self.get_logger().error("Goal rejected.")
            return False

        res_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_future, timeout_sec=timeout_sec)
        if not res_future.done():
            self.get_logger().error("Goal result wait timed out.")
            return False

        return True

    def run(self):
        # -------------------------
        # GOAL A: move to pose + twist around *neutral roll* + reset
        # -------------------------
        arm_joints = [
            "joint_lift",
            "joint_arm_l0",
            "joint_arm_l1",
            "joint_arm_l2",
            "joint_arm_l3",
            "joint_wrist_yaw",
            "joint_wrist_pitch",
            "joint_wrist_roll",
        ]

        # You said this height is good:
        lift_up = 0.85

        # Use current extension as baseline; you can also set these explicitly.
        # Keeping them modest avoids surprises.
        arm_l = 0.014180634514987614  # from your sample; change if you want more/less reach

        yaw_hold = self.yaw_neutral
        pitch_hold = self.pitch_neutral
        roll0 = self.roll_neutral

        roll_amp = 1.0   # twist amount
        step = 0.5        # time between flips

        t = 0.0
        ptsA = []

        def ptA(t_s, lift, arm0, arm1, arm2, arm3, yaw, pitch, roll):
            p = JointTrajectoryPoint()
            p.positions = [float(lift), float(arm0), float(arm1), float(arm2), float(arm3),
                           float(yaw), float(pitch), float(roll)]
            p.time_from_start = dur(t_s)
            return p

        # Move into pose at neutral roll (prevents ending slanted)
        t += 0.9
        ptsA.append(ptA(t, lift_up, arm_l, arm_l, arm_l, arm_l, yaw_hold, pitch_hold, roll0))

        # Twists: +, -, +, - around roll0 (no intermediate)
        for r in (roll0 + roll_amp, roll0 - roll_amp, roll0 + roll_amp, roll0 - roll_amp):
            t += step
            ptsA.append(ptA(t, lift_up, arm_l, arm_l, arm_l, arm_l, yaw_hold, pitch_hold, r))

        # Reset to neutral roll BEFORE claps
        t += 0.8
        ptsA.append(ptA(t, lift_up, arm_l, arm_l, arm_l, arm_l, yaw_hold, pitch_hold, roll0))

        self.get_logger().info("Sending Goal A (pose + twist fixed + reset)...")
        okA = self.send_goal_and_wait(arm_joints, ptsA, timeout_sec=10.0)
        if not okA:
            self.get_logger().error("Goal A failed/timeout.")
            return

        # -------------------------
        # GOAL B: claps only (gripper joint only) - same as your working clap
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
            self.get_logger().error("Goal B failed/timeout.")
            return

        # -------------------------
        # Optional: return wrist roll to 0.0 (true zero) OR keep neutral
        # I recommend keeping neutral to avoid slant surprises.
        # -------------------------
        self.get_logger().info("Done (twist ends at your neutral roll; claps completed).")


def main():
    rclpy.init()
    node = TwoGoalTwistClapFull()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()