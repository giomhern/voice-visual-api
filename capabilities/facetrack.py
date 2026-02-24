#!/usr/bin/env python3
"""
Stretch 3: Face scan (head pan) until MarkerArray is non-empty.

Behavior:
- Subscribe to MarkerArray face detections.
- Sweep head pan left<->right continuously.
- As soon as markers array is non-empty, STOP sending motion and print FOUND.

Key notes:
- Uses the same action server that you confirmed works:
    /stretch_controller/follow_joint_trajectory
- Sends a 2-waypoint trajectory each step (controller often requires >=2 points).
- Uses /joint_states to read current head pan (and optional tilt hold).
"""

import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


def dur(t: float) -> Duration:
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


class FaceScanPan(Node):
    def __init__(self):
        super().__init__("stretch_face_scan_pan")

        # ---- Parameters (easy to override via --ros-args -p ...) ----
        self.declare_parameter("marker_topic", "/face_detections/markers")
        self.declare_parameter("traj_action", "/stretch_controller/follow_joint_trajectory")

        self.declare_parameter("pan_joint", "joint_head_pan")
        self.declare_parameter("tilt_joint", "joint_head_tilt")

        self.declare_parameter("left_limit", -1.6)     # rad
        self.declare_parameter("right_limit", 1.6)     # rad
        self.declare_parameter("step", 0.12)           # rad per move
        self.declare_parameter("dwell_s", 0.20)        # wait between steps (seconds)

        # Timing for the tiny trajectories we send each step
        self.declare_parameter("t_start_s", 0.05)       # start waypoint time_from_start
        self.declare_parameter("t_goal_s", 0.35)        # goal waypoint time_from_start
        self.declare_parameter("goal_timeout_s", 3.0)   # wait for action result

        # Hold tilt constant (optional)
        self.declare_parameter("hold_tilt", True)       # keep current tilt during scan

        self.marker_topic: str = str(self.get_parameter("marker_topic").value)
        self.traj_action: str = str(self.get_parameter("traj_action").value)
        self.pan_joint: str = str(self.get_parameter("pan_joint").value)
        self.tilt_joint: str = str(self.get_parameter("tilt_joint").value)

        self.left_limit: float = float(self.get_parameter("left_limit").value)
        self.right_limit: float = float(self.get_parameter("right_limit").value)
        self.step: float = float(self.get_parameter("step").value)
        self.dwell_s: float = float(self.get_parameter("dwell_s").value)

        self.t_start_s: float = float(self.get_parameter("t_start_s").value)
        self.t_goal_s: float = float(self.get_parameter("t_goal_s").value)
        self.goal_timeout_s: float = float(self.get_parameter("goal_timeout_s").value)

        self.hold_tilt: bool = bool(self.get_parameter("hold_tilt").value)

        if self.left_limit > self.right_limit:
            self.left_limit, self.right_limit = self.right_limit, self.left_limit

        # ---- State ----
        self._found: bool = False
        self._latest_marker_count: int = 0

        self._js: Optional[JointState] = None
        self._start_pan: Optional[float] = None
        self._start_tilt: Optional[float] = None

        self._direction: int = +1  # +1 -> moving right, -1 -> moving left
        self._target_pan: float = self.left_limit

        # ---- Subscriptions ----
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.create_subscription(MarkerArray, self.marker_topic, self._on_markers, 10)

        # ---- Action client ----
        self.client = ActionClient(self, FollowJointTrajectory, self.traj_action)

        # ---- Wait for joint states so we can seed "current pan" properly ----
        self.get_logger().info("Waiting for /joint_states...")
        while rclpy.ok() and self._js is None:
            rclpy.spin_once(self, timeout_sec=0.2)

        self._start_pan = self._pos(self.pan_joint)
        self._start_tilt = self._pos(self.tilt_joint)

        self.get_logger().info(
            f"Start head pose: pan={self._start_pan:.3f}, tilt={self._start_tilt:.3f}"
        )

        # ---- Wait for trajectory server ----
        self.get_logger().info(f"Waiting for trajectory action server: {self.traj_action}")
        if not self.client.wait_for_server(timeout_sec=8.0):
            raise RuntimeError(f"No trajectory action server at {self.traj_action}")

        self.get_logger().info(f"Listening for face markers on: {self.marker_topic}")
        self.get_logger().info(
            f"Scan limits: [{self.left_limit:.2f}, {self.right_limit:.2f}] rad | "
            f"step={self.step:.2f} | dwell={self.dwell_s:.2f}s | hold_tilt={self.hold_tilt}"
        )

    # -------------------------
    # Callbacks / helpers
    # -------------------------
    def _on_js(self, msg: JointState):
        self._js = msg

    def _on_markers(self, msg: MarkerArray):
        self._latest_marker_count = len(msg.markers)
        if (not self._found) and self._latest_marker_count > 0:
            self._found = True
            self.get_logger().warn("FOUND SOMEONE (MarkerArray is non-empty). Stopping scan.")

    def _pos(self, name: str) -> float:
        assert self._js is not None
        idx = self._js.name.index(name)
        return float(self._js.position[idx])

    def _current_pan_tilt(self) -> Tuple[float, float]:
        # Use latest joint state; fallback to start values if needed
        pan = self._start_pan if self._start_pan is not None else 0.0
        tilt = self._start_tilt if self._start_tilt is not None else 0.0

        if self._js is not None:
            try:
                pan = self._pos(self.pan_joint)
            except Exception:
                pass
            try:
                tilt = self._pos(self.tilt_joint)
            except Exception:
                pass
        return float(pan), float(tilt)

    def _pt(self, t_s: float, pan: float, tilt: float) -> JointTrajectoryPoint:
        p = JointTrajectoryPoint()
        p.positions = [float(pan), float(tilt)]
        p.time_from_start = dur(t_s)
        return p

    def _send_pan_step(self, target_pan: float) -> bool:
        """
        Send a 2-waypoint trajectory for head pan (and tilt if hold_tilt=True):
        - waypoint 1: current position at t_start_s
        - waypoint 2: target position at t_goal_s

        This avoids: "no trajectory in goal contains enough waypoints".
        """
        cur_pan, cur_tilt = self._current_pan_tilt()
        tgt_tilt = cur_tilt if self.hold_tilt else cur_tilt  # currently always holding; kept for future extension

        joints = [self.pan_joint, self.tilt_joint]

        pts = []
        pts.append(self._pt(self.t_start_s, cur_pan, cur_tilt))
        pts.append(self._pt(self.t_goal_s, float(target_pan), float(tgt_tilt)))

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joints
        goal.trajectory.points = pts

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=3.0)
        gh = send_future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error("Trajectory goal rejected.")
            return False

        res_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_future, timeout_sec=float(self.goal_timeout_s))
        return True

    # -------------------------
    # Main scan loop
    # -------------------------
    def run(self):
        # Start sweeping from left limit, but seed target near current pan to avoid huge jump.
        cur_pan, _ = self._current_pan_tilt()
        self._target_pan = max(self.left_limit, min(self.right_limit, cur_pan))

        # First: go to left limit so scan is deterministic (optional).
        if not self._found:
            self.get_logger().info(f"Moving to left limit: {self.left_limit:.2f} rad")
            self._send_pan_step(self.left_limit)
            self._target_pan = self.left_limit
            self._direction = +1

        while rclpy.ok() and not self._found:
            # process incoming markers/joint states
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._found:
                break

            # compute next target
            next_pan = self._target_pan + self._direction * self.step

            # bounce at limits
            if next_pan >= self.right_limit:
                next_pan = self.right_limit
                self._direction = -1
            elif next_pan <= self.left_limit:
                next_pan = self.left_limit
                self._direction = +1

            # send motion
            ok = self._send_pan_step(next_pan)
            if ok:
                self._target_pan = next_pan
                self.get_logger().info(
                    f"Scanning... pan={self._target_pan:.2f} rad | markers={self._latest_marker_count}"
                )
            else:
                # don't spam; give time to recover if controller hiccups
                time.sleep(0.3)

            # dwell (but keep spinning so detection can stop us immediately)
            t_end = time.time() + self.dwell_s
            while time.time() < t_end and rclpy.ok() and not self._found:
                rclpy.spin_once(self, timeout_sec=0.05)

        if self._found:
            self.get_logger().warn("Scan stopped: face detected.")
        else:
            self.get_logger().info("Scan ended (ROS shutting down).")


def main():
    rclpy.init()
    node = FaceScanPan()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()