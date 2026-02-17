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
    "joint_wrist_yaw",        # keep ~0 (no left/right)
    "joint_wrist_pitch",      # slight bend for readability
    "joint_wrist_roll",       # twist happens here
    "joint_gripper_finger_left",
    "joint_gripper_finger_right",
]


def dur(t: float) -> Duration:
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


class TwistThenClap(Node):
    def __init__(self):
        super().__init__("twist_then_clap")
        self.client = ActionClient(self, FollowJointTrajectory, TRAJ_ACTION_NAME)

        self.get_logger().info(f"Waiting for action server: {TRAJ_ACTION_NAME}")
        if not self.client.wait_for_server(timeout_sec=8.0):
            self.get_logger().error(
                "FollowJointTrajectory server not available.\n"
                "Check: ros2 action list | grep FollowJointTrajectory\n"
                "Then set TRAJ_ACTION_NAME accordingly."
            )
            raise RuntimeError("No trajectory action server")

        self.get_logger().info("Connected. Sending twist-then-clap...")
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

        # --- Pose (tune if you want) ---
        lift_up = 0.55
        wrist_ext = 0.06
        yaw_hold = 0.0
        pitch_hold = -0.55

        # Gripper (tune if needed)
        grip_open = 0.045
        grip_closed = 0.005

        # 1) Move into greeting pose
        t += 0.8
        pts.append(self.pt(t, lift_up, wrist_ext, yaw_hold, pitch_hold, 0.0, grip_open, grip_open))

        # 2) Twist in place: just left/right (no intermediate points), twice
        roll_amp = 1.0   # radians
        step = 0.45      # time between flips (increase if too fast)

        # sequence: +, -, +, -
        for r in (+roll_amp, -roll_amp, +roll_amp, -roll_amp):
            t += step
            pts.append(self.pt(t, lift_up, wrist_ext, yaw_hold, pitch_hold, r, grip_open, grip_open))

        # 3) Center the wrist before clapping
        t += 0.35
        pts.append(self.pt(t, lift_up, wrist_ext, yaw_hold, pitch_hold, 0.0, grip_open, grip_open))

        # 4) Claps (open/close) — do them clearly
        clap_count = 3
        clap_step = 0.28
        for _ in range(clap_count):
            t += clap_step
            pts.append(self.pt(t, lift_up, wrist_ext, yaw_hold, pitch_hold, 0.0, grip_closed, grip_closed))
            t += clap_step
            pts.append(self.pt(t, lift_up, wrist_ext, yaw_hold, pitch_hold, 0.0, grip_open, grip_open))

        # 5) Return neutral
        t += 0.9
        pts.append(self.pt(t, 0.45, 0.0, 0.0, 0.0, 0.0, grip_open, grip_open))

        goal.trajectory.points = pts

        # --- Send goal and keep the node alive until it finishes ---
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=2.0)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected.")
            return

        self.get_logger().info(f"Goal accepted. Letting it run for ~{t:.1f}s...")
        result_future = goal_handle.get_result_async()

        # Wait for completion (with a bit of padding)
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=t + 3.0)
        self.get_logger().info("Done.")


def main():
    rclpy.init()
    node = TwistThenClap()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()