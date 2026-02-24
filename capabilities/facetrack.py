#!/usr/bin/env python3
"""
Stretch 3: simple face-scan head pan
- Pan head from left_limit -> right_limit -> left_limit ...
- Subscribe to a MarkerArray topic from deep perception face detection
- When MarkerArray.markers is non-empty, stop scanning and print to console

Run:
  ros2 launch stretch_deep_perception face_detection.launch.py
  chmod +x face_scan_pan.py
  ros2 run <your_pkg> face_scan_pan.py --ros-args -p marker_topic:=/YOUR/FACE/MARKERS

Typical params you may want:
  -p left_limit:=-1.6 -p right_limit:=1.6 -p step:=0.12 -p dwell_s:=0.25
"""

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from visualization_msgs.msg import MarkerArray
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class FaceScanPan(Node):
    def __init__(self):
        super().__init__("face_scan_pan")

        # --- Parameters ---
        self.declare_parameter("marker_topic", "/face_detections/markers")
        self.declare_parameter("traj_action", "/stretch_controller/follow_joint_trajectory")
        self.declare_parameter("head_pan_joint", "joint_head_pan")

        self.declare_parameter("left_limit", -1.6)   # radians
        self.declare_parameter("right_limit", 1.6)   # radians
        self.declare_parameter("step", 0.12)         # radians per tick
        self.declare_parameter("dwell_s", 0.25)      # seconds to wait between moves
        self.declare_parameter("goal_time_s", 0.35)  # time_from_start for each pan micro-move

        self.marker_topic = self.get_parameter("marker_topic").value
        self.traj_action = self.get_parameter("traj_action").value
        self.head_pan_joint = self.get_parameter("head_pan_joint").value

        self.left_limit = float(self.get_parameter("left_limit").value)
        self.right_limit = float(self.get_parameter("right_limit").value)
        self.step = float(self.get_parameter("step").value)
        self.dwell_s = float(self.get_parameter("dwell_s").value)
        self.goal_time_s = float(self.get_parameter("goal_time_s").value)

        if self.left_limit > self.right_limit:
            self.left_limit, self.right_limit = self.right_limit, self.left_limit

        # --- State ---
        self._found = False
        self._latest_marker_count = 0
        self._current_pan: float = self.left_limit
        self._direction: int = +1  # +1 => moving right, -1 => moving left

        # --- Subscriber: MarkerArray ---
        self._sub = self.create_subscription(
            MarkerArray,
            self.marker_topic,
            self._on_markers,
            10,
        )

        # --- Action client: FollowJointTrajectory ---
        self._traj_client = ActionClient(self, FollowJointTrajectory, self.traj_action)

        self.get_logger().info(f"Listening for face markers on: {self.marker_topic}")
        self.get_logger().info(f"Head pan joint: {self.head_pan_joint}")
        self.get_logger().info(
            f"Scan limits: [{self.left_limit:.2f}, {self.right_limit:.2f}] rad | "
            f"step={self.step:.2f} | dwell={self.dwell_s:.2f}s"
        )

    def _on_markers(self, msg: MarkerArray):
        self._latest_marker_count = len(msg.markers)
        if (not self._found) and self._latest_marker_count > 0:
            self._found = True
            self.get_logger().warn("FOUND SOMEONE (MarkerArray is non-empty). Stopping scan.")

    def _send_pan_goal(self, pan_rad: float) -> bool:
        """Send a small trajectory goal to set head pan to pan_rad."""
        if not self._traj_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                f"Trajectory action server not available: {self.traj_action} "
                f"(is Stretch in trajectory mode?)"
            )
            return False

        traj = JointTrajectory()
        traj.joint_names = [self.head_pan_joint]

        pt = JointTrajectoryPoint()
        pt.positions = [float(pan_rad)]
        pt.time_from_start = Duration(
            sec=int(self.goal_time_s),
            nanosec=int((self.goal_time_s - int(self.goal_time_s)) * 1e9),
        )
        traj.points = [pt]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        send_future = self._traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=3.0)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected.")
            return False

        # We don't strictly need to wait for full completion, but it makes motion smoother/predictable
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=3.0)

        return True

    def spin_scan_loop(self):
        """Main scanning loop: sweep head pan until markers appear."""
        # Initialize at left limit
        self._current_pan = self.left_limit
        self._direction = +1

        # Make one initial move so we're in a known pose
        self._send_pan_goal(self._current_pan)

        while rclpy.ok() and not self._found:
            rclpy.spin_once(self, timeout_sec=0.05)

            # If a face appears between moves, stop immediately
            if self._found:
                break

            # Next pan target
            next_pan = self._current_pan + self._direction * self.step

            # Bounce at limits
            if next_pan >= self.right_limit:
                next_pan = self.right_limit
                self._direction = -1
            elif next_pan <= self.left_limit:
                next_pan = self.left_limit
                self._direction = +1

            # Command motion
            ok = self._send_pan_goal(next_pan)
            if not ok:
                # If trajectory server isn't available, don't hammer it
                time.sleep(0.5)
                continue

            self._current_pan = next_pan

            # Optional: debug print every step
            self.get_logger().info(
                f"Scanning... pan={self._current_pan:.2f} rad | markers={self._latest_marker_count}"
            )

            # Dwell between steps
            t_end = time.time() + self.dwell_s
            while time.time() < t_end and rclpy.ok() and not self._found:
                rclpy.spin_once(self, timeout_sec=0.05)

        if self._found:
            self.get_logger().warn("Scan stopped: face detected.")
        else:
            self.get_logger().info("Scan loop ended (ROS shutdown).")


def main():
    rclpy.init()
    node = FaceScanPan()
    try:
        node.spin_scan_loop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()