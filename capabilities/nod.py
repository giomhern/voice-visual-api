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


class HeadNodYes(Node):
    def __init__(self):
        super().__init__("head_nod_yes")

        self.client = ActionClient(self, FollowJointTrajectory, TRAJ_ACTION_NAME)

        self.js = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

        self.get_logger().info("Waiting for /joint_states...")
        while rclpy.ok() and self.js is None:
            rclpy.spin_once(self, timeout_sec=0.2)

        self.pan0 = self._pos("joint_head_pan")
        self.tilt0 = self._pos("joint_head_tilt")

        self.get_logger().info(
            f"Starting at pan={self.pan0:.3f}, tilt={self.tilt0:.3f}"
        )

        self.client.wait_for_server()
        self.send_nod()

    def _on_js(self, msg):
        self.js = msg

    def _pos(self, name):
        idx = self.js.name.index(name)
        return float(self.js.position[idx])

    def send_nod(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["joint_head_pan", "joint_head_tilt"]

        pts = []
        t = 0.0

        NOD_AMOUNT = 0.40   # increase for stronger nod

        # Neutral
        t += 0.5
        pts.append(self.pt(t, self.pan0, self.tilt0))

        # Down
        t += 0.7
        pts.append(self.pt(t, self.pan0, self.tilt0 + NOD_AMOUNT))

        # Up (past neutral slightly for realism)
        t += 0.7
        pts.append(self.pt(t, self.pan0, self.tilt0 - 0.20))

        # Down again
        t += 0.7
        pts.append(self.pt(t, self.pan0, self.tilt0 + NOD_AMOUNT * 0.7))

        # Return to neutral
        t += 0.7
        pts.append(self.pt(t, self.pan0, self.tilt0))

        goal.trajectory.points = pts

        self.get_logger().info("Nodding yes...")
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        gh = send_future.result()
        if gh and gh.accepted:
            result_future = gh.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=6.0)

        self.get_logger().info("Done.")

    def pt(self, t_s, pan, tilt):
        p = JointTrajectoryPoint()
        p.positions = [float(pan), float(tilt)]
        p.time_from_start = dur(t_s)
        return p


def main():
    rclpy.init()
    node = HeadNodYes()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()