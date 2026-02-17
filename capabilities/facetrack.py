#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration as RclpyDuration

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# MarkerArray is a very common “face exists” output in this stack
from visualization_msgs.msg import MarkerArray


def dur(t: float) -> Duration:
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


class FaceScanAndHold(Node):
    """
    Scans head pan left-right until a face marker arrives, then stops and holds.
    Requires: stretch_driver running + face detector running.
    """

    def __init__(self):
        super().__init__("face_scan_and_hold")

        # --- params you may need to adjust ---
        self.declare_parameter("traj_action", "/stretch_controller/follow_joint_trajectory")
        self.declare_parameter("face_markers_topic", "/faces/markers")  # change if your topic differs
        self.declare_parameter("pan_min", -1.4)     # radians
        self.declare_parameter("pan_max",  1.4)     # radians
        self.declare_parameter("tilt_hold", -0.25)  # radians (negative often looks slightly down)
        self.declare_parameter("step", 0.35)        # radians per scan step
        self.declare_parameter("move_time", 0.7)    # seconds per step
        self.declare_parameter("settle_time", 0.2)  # seconds pause after each move
        self.declare_parameter("confirm_msgs", 2)   # require N marker msgs before declaring “found”

        self.traj_action = self.get_parameter("traj_action").value
        self.face_topic = self.get_parameter("face_markers_topic").value

        self.pan_min = float(self.get_parameter("pan_min").value)
        self.pan_max = float(self.get_parameter("pan_max").value)
        self.tilt_hold = float(self.get_parameter("tilt_hold").value)
        self.step = float(self.get_parameter("step").value)
        self.move_time = float(self.get_parameter("move_time").value)
        self.settle_time = float(self.get_parameter("settle_time").value)
        self.confirm_msgs = int(self.get_parameter("confirm_msgs").value)

        # --- state ---
        self.face_msg_count = 0
        self.face_found = False
        self.last_face_time = None

        # --- subscribers ---
        self.sub = self.create_subscription(MarkerArray, self.face_topic, self._on_faces, 10)

        # --- action client for head motion ---
        self.client = ActionClient(self, FollowJointTrajectory, self.traj_action)
        self.get_logger().info(f"Waiting for trajectory action: {self.traj_action}")
        if not self.client.wait_for_server(timeout_sec=8.0):
            raise RuntimeError(
                f"FollowJointTrajectory server not available at {self.traj_action}. "
                "Verify with: ros2 action list | grep FollowJointTrajectory"
            )

        self.get_logger().info(f"Listening for faces on: {self.face_topic}")
        self.get_logger().info("Starting head scan...")
        self.scan_loop()

    def _on_faces(self, msg: MarkerArray):
        # Many implementations publish empty arrays sometimes; treat “non-empty” as detection.
        if msg.markers and len(msg.markers) > 0:
            self.face_msg_count += 1
            self.last_face_time = self.get_clock().now()
            if self.face_msg_count >= self.confirm_msgs:
                self.face_found = True

    def send_head_point(self, pan: float, tilt: float, t_from_start: float) -> bool:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["joint_head_pan", "joint_head_tilt"]

        p = JointTrajectoryPoint()
        p.positions = [float(pan), float(tilt)]
        p.time_from_start = dur(t_from_start)

        goal.trajectory.points = [p]

        fut = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().warn("Head goal rejected.")
            return False

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=max(3.0, t_from_start + 2.0))
        return True

    def scan_loop(self):
        # Build a simple scan sequence: pan_min -> pan_max -> pan_min ...
        pan = self.pan_min
        direction = +1.0

        # Start at center-ish first (nicer behavior)
        self.send_head_point(0.0, self.tilt_hold, t_from_start=1.0)
        time.sleep(0.2)

        while rclpy.ok() and not self.face_found:
            # Move one step
            pan_next = pan + direction * self.step
            if pan_next > self.pan_max:
                pan_next = self.pan_max
                direction = -1.0
            elif pan_next < self.pan_min:
                pan_next = self.pan_min
                direction = +1.0

            ok = self.send_head_point(pan_next, self.tilt_hold, t_from_start=self.move_time)
            if not ok:
                # if rejected, slow down a bit and retry next loop
                time.sleep(0.5)

            # allow callbacks to process face markers
            t_end = time.time() + self.settle_time
            while time.time() < t_end and rclpy.ok() and not self.face_found:
                rclpy.spin_once(self, timeout_sec=0.05)

            pan = pan_next

        if self.face_found:
            self.get_logger().info("Face found. Holding head position.")
            # “Hold” by simply not sending more scan commands.
            # Optionally, send a final point to “lock in” current pose:
            self.send_head_point(pan, self.tilt_hold, t_from_start=0.5)


def main():
    rclpy.init()
    node = FaceScanAndHold()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()