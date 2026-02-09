#!/usr/bin/env python3
from __future__ import annotations

import time
from typing import Dict

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
import rclpy.duration


class CleanSurfaceExec(Node):
    """
    Execution-only clean surface node.

    ASSUMPTIONS:
    - funmap is already running
    - robot is already positioned at the desk
    - arm is already in preprocessing pose
    """

    def __init__(self):
        super().__init__("clean_surface_exec")

        # Trajectory action client
        self._traj_action = "/stretch_controller/follow_joint_trajectory"
        self._traj_client = ActionClient(
            self, FollowJointTrajectory, self._traj_action
        )

        # Trigger service
        self._srv = self.create_service(
            Trigger,
            "/clean_surface/trigger_clean_surface",
            self._on_trigger,
        )

        self.get_logger().info(
            "[CLEAN_SURFACE_EXEC] Ready. Waiting for trigger_clean_surface."
        )

    # -----------------------------
    # Service callback
    # -----------------------------
    def _on_trigger(self, req: Trigger.Request, resp: Trigger.Response):
        self.get_logger().info("[CLEAN_SURFACE_EXEC] Trigger received")

        # Defensive wait for trajectory controller
        if not self._traj_client.wait_for_server(timeout_sec=5.0):
            resp.success = False
            resp.message = "Trajectory action server not available"
            return resp

        ok = self._execute_wipe_motion()

        resp.success = bool(ok)
        resp.message = "Clean surface executed" if ok else "Clean surface failed"
        return resp

    # -----------------------------
    # Wipe execution
    # -----------------------------
    def _execute_wipe_motion(self) -> bool:
        """
        Very simple deterministic wipe:
        - extend wrist forward
        - retract
        Repeatable & safe.
        """

        # Example wipe joints (adjust as needed)
        joint_names = [
            "joint_lift",
            "wrist_extension",
            "joint_wrist_yaw",
        ]

        # Two-phase wipe
        wipe_positions = [
            [0.90,  0.20, 0.0],   # forward
            [0.90, -0.02, 0.0],   # retract
        ]

        for idx, pos in enumerate(wipe_positions):
            self.get_logger().info(f"[CLEAN_SURFACE_EXEC] Wipe phase {idx+1}")
            if not self._send_traj(joint_names, pos, duration_sec=2.0):
                return False

            time.sleep(0.25)

        return True

    # -----------------------------
    # Trajectory helper
    # -----------------------------
    def _send_traj(
        self,
        joint_names,
        positions,
        duration_sec: float,
        timeout_sec: float = 30.0,
    ) -> bool:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)

        # IMPORTANT: future-dated stamp
        start = self.get_clock().now() + rclpy.duration.Duration(seconds=0.5)
        goal.trajectory.header.stamp = start.to_msg()

        pt1 = JointTrajectoryPoint()
        pt1.positions = list(positions)
        pt1.time_from_start = Duration(sec=0, nanosec=int(0.5 * 1e9))

        pt2 = JointTrajectoryPoint()
        pt2.positions = list(positions)
        pt2.time_from_start = Duration(sec=int(duration_sec))

        goal.trajectory.points = [pt1, pt2]

        send_fut = self._traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=timeout_sec)

        if not send_fut.done():
            self.get_logger().error("[CLEAN_SURFACE_EXEC] send_goal timed out")
            return False

        gh = send_fut.result()
        if not gh.accepted:
            self.get_logger().error("[CLEAN_SURFACE_EXEC] Goal rejected")
            return False

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=timeout_sec)

        if not res_fut.done():
            self.get_logger().error("[CLEAN_SURFACE_EXEC] result timed out")
            return False

        result = res_fut.result().result
        self.get_logger().info(
            f"[CLEAN_SURFACE_EXEC] Result error_code={result.error_code}"
        )
        return int(result.error_code) == 0


def main():
    rclpy.init()
    node = CleanSurfaceExec()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()