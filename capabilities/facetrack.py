#!/usr/bin/env python3
"""
Stretch 3: Scan for a face then follow it with head pan (assume only one face).

Phase 1 (SCAN):
- Sweep head pan left<->right until MarkerArray.markers is non-empty.

Phase 2 (FOLLOW):
- Use the first marker pose to compute a horizontal angle error:
    angle = atan2(x, z)
- Update target pan with a P controller:
    target_pan = current_pan + Kp * angle
- Send small 2-point trajectories at a fixed rate.

Notes:
- Uses /stretch_controller/follow_joint_trajectory (same as your working script)
- Marker topic defaults to /faces/marker_array
- Uses sensor-data QoS (BEST_EFFORT) for perception topics
"""

import math
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data

from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import JointState

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


TRAJ_ACTION_NAME = "/stretch_controller/follow_joint_trajectory"

PAN_JOINT = "joint_head_pan"
TILT_JOINT = "joint_head_tilt"

MARKER_TOPIC_DEFAULT = "/faces/marker_array"

# Pan scan params
LEFT_LIMIT = -1.6
RIGHT_LIMIT = 1.6
SCAN_STEP = 0.18
SCAN_DWELL_S = 0.15

# Follow params
FOLLOW_HZ = 8.0
KP = 1.2                  # proportional gain on angle error
MAX_STEP_PER_TICK = 0.18  # rad; limits how fast we move per timer tick
DEADZONE_RAD = 0.02       # rad; ignore tiny error to reduce jitter

# Trajectory timing (2 points)
T_START_S = 0.05
T_GOAL_S = 0.25
GOAL_SEND_TIMEOUT_S = 2.0

# Hold tilt constant (keep current tilt)
HOLD_TILT = True


def dur(t: float) -> Duration:
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def clip_mag(x: float, max_mag: float) -> float:
    if x > max_mag:
        return max_mag
    if x < -max_mag:
        return -max_mag
    return x


class FaceScanAndFollow(Node):
    def __init__(self):
        super().__init__("face_scan_and_follow")

        self.declare_parameter("marker_topic", MARKER_TOPIC_DEFAULT)
        self.marker_topic = str(self.get_parameter("marker_topic").value)

        self.client = ActionClient(self, FollowJointTrajectory, TRAJ_ACTION_NAME)

        # State
        self.js: Optional[JointState] = None
        self.latest_marker_count = 0

        self.mode = "SCAN"  # "SCAN" or "FOLLOW"
        self._scan_target = LEFT_LIMIT
        self._scan_dir = +1

        # Track one face (assumed single)
        self._have_face_pose = False
        self._face_x = 0.0
        self._face_z = 1.0

        # Avoid overlapping action goals
        self._goal_in_flight = False
        self._last_cmd_pan: Optional[float] = None

        # Subs
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.create_subscription(
            MarkerArray,
            self.marker_topic,
            self._on_markers,
            qos_profile_sensor_data,
        )

        # Wait for joint states
        self.get_logger().info("Waiting for /joint_states...")
        while rclpy.ok() and self.js is None:
            rclpy.spin_once(self, timeout_sec=0.2)

        pan0 = self._pos(PAN_JOINT)
        tilt0 = self._pos(TILT_JOINT)
        self.get_logger().info(f"Start head pose: pan={pan0:.3f}, tilt={tilt0:.3f}")

        # Wait for trajectory server
        self.get_logger().info(f"Waiting for trajectory action server: {TRAJ_ACTION_NAME}")
        if not self.client.wait_for_server(timeout_sec=8.0):
            raise RuntimeError("No trajectory action server")

        self.get_logger().info(f"Listening for face markers on: {self.marker_topic}")

        # Main control loop timer
        period = 1.0 / float(FOLLOW_HZ)
        self.create_timer(period, self._control_tick)

        # Move to left limit to start scan deterministically (non-blocking by design)
        self._queue_pan_goal(LEFT_LIMIT)

    # -------------------------
    # Callbacks / helpers
    # -------------------------
    def _on_js(self, msg: JointState):
        self.js = msg

    def _on_markers(self, msg: MarkerArray):
        self.latest_marker_count = len(msg.markers)

        if self.latest_marker_count <= 0:
            self._have_face_pose = False
            return

        # assume only one face: use the first marker
        m = msg.markers[0]
        x = float(m.pose.position.x)
        z = float(m.pose.position.z)

        # guard against weird/zero depth
        if abs(z) < 1e-6:
            self._have_face_pose = False
            return

        self._face_x = x
        self._face_z = z
        self._have_face_pose = True

        if self.mode == "SCAN":
            self.mode = "FOLLOW"
            self.get_logger().warn("FOUND face. Switching from SCAN -> FOLLOW.")

    def _pos(self, name: str) -> float:
        assert self.js is not None
        idx = self.js.name.index(name)
        return float(self.js.position[idx])

    def _pt(self, t_s: float, pan: float, tilt: float) -> JointTrajectoryPoint:
        p = JointTrajectoryPoint()
        p.positions = [float(pan), float(tilt)]
        p.time_from_start = dur(t_s)
        return p

    def _queue_pan_goal(self, target_pan: float) -> None:
        """
        Send a 2-point joint trajectory goal (current -> target) without blocking,
        and prevent sending another while one is in flight.
        """
        if self._goal_in_flight:
            return
        if self.js is None:
            return

        cur_pan = self._pos(PAN_JOINT)
        cur_tilt = self._pos(TILT_JOINT)
        tgt_pan = float(target_pan)
        tgt_tilt = float(cur_tilt) if HOLD_TILT else float(cur_tilt)

        joints = [PAN_JOINT, TILT_JOINT]
        pts = [
            self._pt(T_START_S, cur_pan, cur_tilt),
            self._pt(T_GOAL_S, tgt_pan, tgt_tilt),
        ]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joints
        goal.trajectory.points = pts

        self._goal_in_flight = True
        self._last_cmd_pan = tgt_pan

        send_future = self.client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, fut):
        try:
            gh = fut.result()
        except Exception as e:
            self.get_logger().error(f"send_goal_async failed: {e}")
            self._goal_in_flight = False
            return

        if gh is None or not gh.accepted:
            self.get_logger().error("Goal rejected.")
            self._goal_in_flight = False
            return

        res_future = gh.get_result_async()
        res_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, fut):
        # We don't need the result content for control; just clear in-flight.
        self._goal_in_flight = False

    # -------------------------
    # Control loop
    # -------------------------
    def _control_tick(self):
        if not rclpy.ok() or self.js is None:
            return

        # Keep callbacks flowing
        # (Timer runs inside executor; no need to spin here.)

        if self.mode == "SCAN":
            # Only step scan when no goal is currently moving
            if self._goal_in_flight:
                return

            # next scan target
            next_pan = self._scan_target + self._scan_dir * SCAN_STEP
            if next_pan >= RIGHT_LIMIT:
                next_pan = RIGHT_LIMIT
                self._scan_dir = -1
            elif next_pan <= LEFT_LIMIT:
                next_pan = LEFT_LIMIT
                self._scan_dir = +1

            self._scan_target = next_pan
            self._queue_pan_goal(self._scan_target)
            # optional debug
            # self.get_logger().info(f"Scanning... pan={self._scan_target:.2f} markers={self.latest_marker_count}")
            return

        # FOLLOW mode
        if self.mode == "FOLLOW":
            if not self._have_face_pose:
                # If face momentarily missing, just hold position (no new commands)
                return
            if self._goal_in_flight:
                return

            # Horizontal angle to face in the marker frame:
            # if frame is optical: x is right, z is forward => atan2(x, z)
            angle_err = math.atan2(self._face_x, self._face_z)

            # deadzone to reduce jitter
            if abs(angle_err) < DEADZONE_RAD:
                return

            cur_pan = self._pos(PAN_JOINT)

            delta = KP * angle_err
            delta = clip_mag(delta, MAX_STEP_PER_TICK)

            target_pan = clamp(cur_pan + delta, LEFT_LIMIT, RIGHT_LIMIT)

            self._queue_pan_goal(target_pan)
            # optional debug
            # self.get_logger().info(f"Follow: err={angle_err:.3f} cur={cur_pan:.2f} tgt={target_pan:.2f}")


def main():
    rclpy.init()
    node = FaceScanAndFollow()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()