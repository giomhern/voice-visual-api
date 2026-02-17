#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


TRAJ_ACTION_NAME = "/stretch_controller/follow_joint_trajectory"

# IMPORTANT: only include the ONE gripper joint your controller accepts
JOINTS = [
    "joint_lift",
    "wrist_extension",
    "joint_wrist_yaw",        # hold at 0 (no left/right)
    "joint_wrist_pitch",      # slight bend for readability
    "joint_wrist_roll",       # twist happens here
    "joint_gripper_finger_left",
]


def dur(t: float) -> Duration:
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


class TwistResetClap(Node):
    def __init__(self):
        super().__init__("twist_reset_clap")
        self.client = ActionClient(self, FollowJointTrajectory, TRAJ_ACTION_NAME)

        self.get_logger().info(f"Waiting for action server: {TRAJ_ACTION_NAME}")
        if not self.client.wait_for_server(timeout_sec=8.0):
            self.get_logger().error(
                "FollowJointTrajectory server not available.\n"
                "Check: ros2 action list | grep FollowJointTrajectory\n"
                "Then set TRAJ_ACTION_NAME accordingly."
            )
            raise RuntimeError("No trajectory action server")

        self.get_logger().info("Connected. Sending twist → reset → 3 claps...")
        self.send_motion()

    def pt(self, t_s, lift, wrist_ext, yaw, pitch, roll, grip_left):
        p = JointTrajectoryPoint()
        p.positions = [
            float(lift),
            float(wrist_ext),
            float(yaw),
            float(pitch),
            float(roll),
            float(grip_left),
        ]
        p.time_from_start = dur(t_s)
        return p

    def send_motion(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS

        pts = []
        t = 0.0

        # --- Greeting pose ---
        lift_up = 0.55
        wrist_ext = 0.06
        yaw_hold = 0.0
        pitch_hold = -0.55

        # --- Gripper values (these worked for you before; keep them) ---
        grip_open = 0.045
        grip_closed = 0.005

        # 1) Move into greeting pose (open gripper)
        t += 0.9
        pts.append(self.pt(t, lift_up, wrist_ext, yaw_hold, pitch_hold, 0.0, grip_open))

        # 2) Twist: ONLY left/right (no intermediate), twice
        roll_amp = 1.0   # radians
        step = 0.45      # seconds between flips
        for r in (+roll_amp, -roll_amp, +roll_amp, -roll_amp):
            t += step
            pts.append(self.pt(t, lift_up, wrist_ext, yaw_hold, pitch_hold, r, grip_open))

        # 3) Reset to default pose BEFORE clapping (give it time to settle)
        # This is the key change you asked for.
        t += 0.8
        pts.append(self.pt(t, lift_up, wrist_ext, 0.0, pitch_hold, 0.0, grip_open))

        # 4) 3 claps (close/open) while holding reset pose
        clap_count = 3
        clap_step = 0.30
        for _ in range(clap_count):
            t += clap_step
            pts.append(self.pt(t, lift_up, wrist_ext, 0.0, pitch_hold, 0.0, grip_closed))
            t += clap_step
            pts.append(self.pt(t, lift_up, wrist_ext, 0.0, pitch_hold, 0.0, grip_open))

        # 5) Return neutral
        t += 1.0
        pts.append(self.pt(t, 0.45, 0.0, 0.0, 0.0, 0.0, grip_open))

        goal.trajectory.points = pts

        # --- Send goal and wait so it doesn't exit early ---
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=3.0)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected.")
            return

        self.get_logger().info(f"Goal accepted. Waiting for completion (~{t:.1f}s)...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=t + 5.0)
        self.get_logger().info("Done.")


def main():
    rclpy.init()
    node = TwistResetClap()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()