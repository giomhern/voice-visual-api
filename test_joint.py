#!/usr/bin/env python3
"""
Standalone Stretch arm pre-pose script.

Fixes the two common controller rejections:
1) "trajectory mode does not currently allow execution of goal with start time in the past"
   -> stamp trajectory start slightly in the FUTURE

2) "no trajectory in goal contains enough waypoints"
   -> send >= 2 points (many controllers require this)
"""

import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


def duration_msg(seconds: float) -> Duration:
    sec_i = int(seconds)
    nsec_i = int((seconds - sec_i) * 1e9)
    return Duration(sec=sec_i, nanosec=nsec_i)


class StretchJointMover(Node):
    def __init__(self, action_name: str = "/stretch_controller/follow_joint_trajectory"):
        super().__init__("stretch_joint_mover")
        self._action_name = action_name
        self._action_client = ActionClient(self, FollowJointTrajectory, self._action_name)

    def send_goal(
        self,
        joint_names,
        positions,
        duration_sec: float = 2.0,
        lead_time_sec: float = 0.75,     # start trajectory this far in the future
        first_point_offset_sec: float = 0.50,  # first point after start time
        wait_server_sec: float = 10.0,
        wait_result_sec: float = 30.0,
    ) -> bool:
        if len(joint_names) != len(positions):
            self.get_logger().error("joint_names and positions must have same length")
            return False

        self.get_logger().info(f"Waiting for action server: {self._action_name}")
        if not self._action_client.wait_for_server(timeout_sec=float(wait_server_sec)):
            self.get_logger().error("Action server not available")
            return False

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = list(joint_names)

        # ---- KEY: stamp trajectory start slightly in the future ----
        now = self.get_clock().now()
        start_time = now + rclpy.duration.Duration(seconds=float(lead_time_sec))
        goal_msg.trajectory.header.stamp = start_time.to_msg()

        # ---- KEY: provide >= 2 points ----
        pt1 = JointTrajectoryPoint()
        pt1.positions = list(positions)
        pt1.time_from_start = duration_msg(float(first_point_offset_sec))

        pt2 = JointTrajectoryPoint()
        pt2.positions = list(positions)
        pt2.time_from_start = duration_msg(float(duration_sec))

        # Ensure pt2 is strictly after pt1
        if (pt2.time_from_start.sec < pt1.time_from_start.sec) or (
            pt2.time_from_start.sec == pt1.time_from_start.sec
            and pt2.time_from_start.nanosec <= pt1.time_from_start.nanosec
        ):
            pt2.time_from_start = duration_msg(pt1.time_from_start.sec + 1.0)

        goal_msg.trajectory.points = [pt1, pt2]

        self.get_logger().info(f"Sending goal: {joint_names} -> {positions}")
        send_future = self._action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, send_future, timeout_sec=float(wait_result_sec))
        if not send_future.done():
            self.get_logger().error("send_goal timed out")
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return False

        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=float(wait_result_sec))

        if not result_future.done():
            self.get_logger().error("Result timed out")
            return False

        result = result_future.result().result
        err_str = getattr(result, "error_string", "")
        self.get_logger().info(f"Result: error_code={result.error_code} error_string='{err_str}'")

        return int(result.error_code) == 0


def main():
    action_name = "/stretch_controller/follow_joint_trajectory"
    if len(sys.argv) > 1:
        action_name = sys.argv[1]

    rclpy.init()
    node = StretchJointMover(action_name=action_name)

    ok = node.send_goal(
        joint_names=["joint_lift", "wrist_extension", "joint_wrist_yaw"],
        positions = [0.906026669779699, -8.377152024940286e-06, 0.0051132692929521375],
        duration_sec=2.0,
        lead_time_sec=0.75,
        first_point_offset_sec=0.50,
        wait_server_sec=10.0,
        wait_result_sec=30.0,
    )

    node.get_logger().info(f"Done. success={ok}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()