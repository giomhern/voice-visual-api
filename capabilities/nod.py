#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState


ACTION_NAME = "/stretch_controller/follow_joint_trajectory"


def dur(t: float) -> Duration:
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


class HeadNodWorking(Node):
    def __init__(self):
        super().__init__("head_nod_working")

        # Action client
        self.client = ActionClient(self, FollowJointTrajectory, ACTION_NAME)

        # Joint states
        self.js = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

        self.get_logger().info("Waiting for /joint_states...")
        while rclpy.ok() and self.js is None:
            rclpy.spin_once(self, timeout_sec=0.2)

        self.pan0 = self._pos("joint_head_pan")
        self.tilt0 = self._pos("joint_head_tilt")
        self.get_logger().info(f"Start: pan={self.pan0:.3f}, tilt={self.tilt0:.3f}")

        self.get_logger().info(f"Waiting for action server: {ACTION_NAME}")
        if not self.client.wait_for_server(timeout_sec=8.0):
            raise RuntimeError(f"No action server at {ACTION_NAME}")

        self.send_nod()

    def _on_js(self, msg: JointState):
        self.js = msg

    def _pos(self, name: str) -> float:
        idx = self.js.name.index(name)
        return float(self.js.position[idx])

    def pt(self, t_s: float, pan: float, tilt: float) -> JointTrajectoryPoint:
        p = JointTrajectoryPoint()
        p.positions = [float(pan), float(tilt)]
        p.time_from_start = dur(t_s)
        return p

    def send_nod(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["joint_head_pan", "joint_head_tilt"]

        # Use a very explicit “start hold” point first
        # Then a clear up/down nod relative to the current tilt
        NOD_DOWN = 0.40
        NOD_UP = -0.25

        pts = [
            self.pt(0.10, self.pan0, self.tilt0),                 # start hold
            self.pt(1.00, self.pan0, self.tilt0),                 # settle
            self.pt(2.00, self.pan0, self.tilt0 + NOD_DOWN),      # down
            self.pt(3.00, self.pan0, self.tilt0 + NOD_UP),        # up
            self.pt(4.00, self.pan0, self.tilt0 + NOD_DOWN * 0.6),# down smaller
            self.pt(5.00, self.pan0, self.tilt0),                 # back neutral
        ]

        goal.trajectory.points = pts

        self.get_logger().info("Sending nod goal...")
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=3.0)

        goal_handle = send_future.result()
        if goal_handle is None:
            self.get_logger().error("No goal handle returned (send failed).")
            return
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected.")
            return

        self.get_logger().info("Goal accepted. Waiting for result (and spinning)...")
        result_future = goal_handle.get_result_async()

        # IMPORTANT: keep spinning while the motion is executing
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)

        if not result_future.done():
            self.get_logger().error("Timed out waiting for result.")
            return

        res = result_future.result().result
        status = result_future.result().status
        self.get_logger().info(
            f"Result: status={status}, error_code={res.error_code}, error_string='{res.error_string}'"
        )


def main():
    rclpy.init()
    node = HeadNodWorking()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()