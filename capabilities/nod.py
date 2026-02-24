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


class HeadNod(Node):
    def __init__(self):
        super().__init__("head_nod")

        self.client = ActionClient(self, FollowJointTrajectory, TRAJ_ACTION_NAME)

        # Get current head position
        self.js = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

        self.get_logger().info("Waiting for /joint_states...")
        while rclpy.ok() and self.js is None:
            rclpy.spin_once(self, timeout_sec=0.2)

        self.pan0 = self._pos("joint_head_pan")
        self.tilt0 = self._pos("joint_head_tilt")

        self.get_logger().info(
            f"Neutral head position: pan={self.pan0:.3f}, tilt={self.tilt0:.3f}"
        )

        self.get_logger().info(f"Waiting for action server: {TRAJ_ACTION_NAME}")
        if not self.client.wait_for_server(timeout_sec=8.0):
            raise RuntimeError("No trajectory action server.")

        self.send_nod()

    def _on_js(self, msg: JointState):
        self.js = msg

    def _pos(self, name):
        idx = self.js.name.index(name)
        return float(self.js.position[idx])

    def send_nod(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["joint_head_pan", "joint_head_tilt"]

        points = []
        t = 0.0

        # Nod amplitude (adjust if needed)
        NOD_AMOUNT = 0.35  # radians (~20 degrees)

        # Start at neutral
        p0 = JointTrajectoryPoint()
        p0.positions = [self.pan0, self.tilt0]
        p0.time_from_start = dur(0.5)
        points.append(p0)

        # Nod down
        p1 = JointTrajectoryPoint()
        p1.positions = [self.pan0, self.tilt0 + NOD_AMOUNT]
        p1.time_from_start = dur(1.2)
        points.append(p1)

        # Nod up (back past neutral slightly for natural feel)
        p2 = JointTrajectoryPoint()
        p2.positions = [self.pan0, self.tilt0 - 0.15]
        p2.time_from_start = dur(2.0)
        points.append(p2)

        # Return to neutral
        p3 = JointTrajectoryPoint()
        p3.positions = [self.pan0, self.tilt0]
        p3.time_from_start = dur(2.8)
        points.append(p3)

        goal.trajectory.points = points

        self.get_logger().info("Sending nod...")
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        gh = send_future.result()
        if gh and gh.accepted:
            result_future = gh.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=6.0)

        self.get_logger().info("Nod complete.")


def main():
    rclpy.init()
    node = HeadNod()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()