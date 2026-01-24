from __future__ import annotations

import time
from typing import List, Tuple, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class ArmCommander:
    """Simple FollowJointTrajectory helper with joint_states lookup."""

    def __init__(self, node: Node, action_name: str, joint_states_topic: str = "/stretch/joint_states"):
        self.node = node
        self.action_name = action_name
        self.joint_states_topic = joint_states_topic

        self._js: Optional[JointState] = None
        self._js_sub = node.create_subscription(JointState, joint_states_topic, self._on_js, 10)
        self._client = ActionClient(node, FollowJointTrajectory, action_name)

    def _on_js(self, msg: JointState):
        self._js = msg

    def wait_ready(self, timeout_s: float = 5.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout_s and rclpy.ok():
            if self._js is not None:
                break
            rclpy.spin_once(self.node, timeout_sec=0.1)

        if self._js is None:
            self.node.get_logger().error(f"[ARM] No joint_states on {self.joint_states_topic}")
            return False

        if not self._client.wait_for_server(timeout_sec=timeout_s):
            self.node.get_logger().error(f"[ARM] Action server not available: {self.action_name}")
            return False

        return True

    def _indices(self, joint_names: List[str]) -> Optional[List[int]]:
        if self._js is None or not self._js.name:
            return None
        idx = []
        for j in joint_names:
            if j not in self._js.name:
                self.node.get_logger().error(f"[ARM] Joint '{j}' not in joint_states.")
                return None
            idx.append(self._js.name.index(j))
        return idx

    def send_pose(self, joint_names: List[str], positions: List[float], duration_s: float = 2.0, wait: bool = True) -> bool:
        idx = self._indices(joint_names)
        if idx is None:
            return False

        # Current positions for start point
        start = JointTrajectoryPoint()
        start.time_from_start = Duration(seconds=0.0).to_msg()
        start.positions = [float(self._js.position[i]) for i in idx]  # type: ignore

        goal_pt = JointTrajectoryPoint()
        goal_pt.time_from_start = Duration(seconds=float(duration_s)).to_msg()
        goal_pt.positions = [float(p) for p in positions]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)
        goal.trajectory.points = [start, goal_pt]

        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        gh = future.result()
        if gh is None or not gh.accepted:
            self.node.get_logger().error("[ARM] goal rejected")
            return False

        if not wait:
            return True

        res_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res_future, timeout_sec=float(duration_s) + 10.0)
        return True

    def send_trajectory(self, joint_names: List[str], points: List[Tuple[List[float], float]], wait: bool = True) -> bool:
        idx = self._indices(joint_names)
        if idx is None:
            return False

        start = JointTrajectoryPoint()
        start.time_from_start = Duration(seconds=0.0).to_msg()
        start.positions = [float(self._js.position[i]) for i in idx]  # type: ignore

        traj_points = [start]
        for pose, t in points:
            pt = JointTrajectoryPoint()
            pt.time_from_start = Duration(seconds=float(t)).to_msg()
            pt.positions = [float(p) for p in pose]
            traj_points.append(pt)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)
        goal.trajectory.points = traj_points

        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        gh = future.result()
        if gh is None or not gh.accepted:
            self.node.get_logger().error("[ARM] trajectory goal rejected")
            return False

        if not wait:
            return True

        res_future = gh.get_result_async()
        total = points[-1][1] if points else 2.0
        rclpy.spin_until_future_complete(self.node, res_future, timeout_sec=float(total) + 10.0)
        return True
