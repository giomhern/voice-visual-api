from __future__ import annotations

import time
from typing import List, Optional, Sequence

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState


class ArmCommander:
    """
    Minimal action-based arm commander for Stretch using FollowJointTrajectory.

    - Subscribes to /joint_states to read current positions
    - Sends 2-point trajectories: (current -> target) over T seconds
    """

    def __init__(self, node: Node, action_name: str, joint_states_topic: str = "/joint_states"):
        self.node = node
        self._joint_state: JointState = JointState()
        self._have_joint_state = False

        self._js_sub = node.create_subscription(JointState, joint_states_topic, self._on_js, 10)
        self._client = ActionClient(node, FollowJointTrajectory, action_name)

    def _on_js(self, msg: JointState):
        self._joint_state = msg
        self._have_joint_state = True

    def wait_ready(self, timeout_s: float = 5.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self._have_joint_state and self._joint_state.name:
                break
        if not (self._have_joint_state and self._joint_state.name):
            self.node.get_logger().error("[ARM] No joint_states received.")
            return False

        if not self._client.wait_for_server(timeout_sec=timeout_s):
            self.node.get_logger().error(f"[ARM] Trajectory action server not available: {self._client._action_name}")
            return False

        return True

    def _current_positions(self, joint_names: Sequence[str]) -> Optional[List[float]]:
        js = self._joint_state
        if not js.name:
            return None
        out = []
        for j in joint_names:
            if j not in js.name:
                self.node.get_logger().error(f"[ARM] Joint '{j}' not in joint_states.")
                return None
            idx = js.name.index(j)
            out.append(js.position[idx])
        return out

    def send_pose(
        self,
        joint_names: Sequence[str],
        target_positions: Sequence[float],
        duration_s: float = 2.0,
        wait: bool = True,
    ) -> bool:
        """
        Send a simple 2-point trajectory: current -> target.
        If wait=True, block until goal result or timeout.
        """
        if len(joint_names) != len(target_positions):
            raise ValueError("joint_names and target_positions must have same length")

        cur = self._current_positions(joint_names)
        if cur is None:
            return False

        p0 = JointTrajectoryPoint()
        p1 = JointTrajectoryPoint()

        p0.time_from_start = Duration(seconds=0.0).to_msg()
        p1.time_from_start = Duration(seconds=float(duration_s)).to_msg()

        p0.positions = list(cur)
        p1.positions = list(target_positions)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)
        goal.trajectory.points = [p0, p1]

        self.node.get_logger().info(f"[ARM] Sending pose {list(joint_names)} -> {list(target_positions)} over {duration_s}s")

        future = self._client.send_goal_async(goal)
        if not wait:
            return True

        # Wait for goal handle
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error("[ARM] Goal rejected.")
            return False

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=max(5.0, duration_s + 3.0))
        res = result_future.result()
        if res is None:
            self.node.get_logger().warn("[ARM] No result received (timeout).")
            return False

        self.node.get_logger().info("[ARM] Goal complete.")
        return True
