#!/usr/bin/env python3
"""
Stretch 3: Face scan (head pan) until MarkerArray is non-empty.

Behavior:
- Subscribe to MarkerArray face detections (default: /face_detections/markers)
- Sweep head pan left<->right continuously
- As soon as MarkerArray.markers is non-empty:
    - print FOUND
    - stop sending new trajectories

Motion:
- Uses /stretch_controller/follow_joint_trajectory (same as your working nod script)
- Sends 2-point trajectories (controller-friendly)
- Holds head tilt at current value unless you change HOLD_TILT
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import JointState

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


TRAJ_ACTION_NAME = "/stretch_controller/follow_joint_trajectory"

PAN_JOINT = "joint_head_pan"
TILT_JOINT = "joint_head_tilt"

MARKER_TOPIC_DEFAULT = "/faces/marker_array"

# Scan params
LEFT_LIMIT = -3
RIGHT_LIMIT = 3
STEP = 0.12
DWELL_S = 0.20

# Trajectory timing (2 points)
T_START_S = 0.05
T_GOAL_S = 0.35
GOAL_TIMEOUT_S = 3.0

HOLD_TILT = True  # keep tilt fixed at current value during scan


def dur(t: float) -> Duration:
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


class FaceScanPan(Node):
    def __init__(self):
        super().__init__("face_scan_pan")

        # Allow overriding marker topic without editing code
        self.declare_parameter("marker_topic", MARKER_TOPIC_DEFAULT)
        self.marker_topic = str(self.get_parameter("marker_topic").value)

        self.client = ActionClient(self, FollowJointTrajectory, TRAJ_ACTION_NAME)

        # Joint state
        self.js = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

        # Marker array
        self.latest_marker_count = 0
        self.found = False
        self.create_subscription(MarkerArray, self.marker_topic, self._on_markers, 10)

        # Wait for joint states
        self.get_logger().info("Waiting for /joint_states...")
        while rclpy.ok() and self.js is None:
            rclpy.spin_once(self, timeout_sec=0.2)

        # Record start pose
        self.pan0 = self._pos(PAN_JOINT)
        self.tilt0 = self._pos(TILT_JOINT)
        self.get_logger().info(f"Start head pose: pan={self.pan0:.3f}, tilt={self.tilt0:.3f}")

        # Wait for trajectory server
        self.get_logger().info(f"Waiting for trajectory action server: {TRAJ_ACTION_NAME}")
        if not self.client.wait_for_server(timeout_sec=8.0):
            raise RuntimeError("No trajectory action server")

        self.get_logger().info(f"Listening for face markers on: {self.marker_topic}")
        self.get_logger().info(
            f"Scan limits: [{LEFT_LIMIT:.2f}, {RIGHT_LIMIT:.2f}] rad | "
            f"step={STEP:.2f} | dwell={DWELL_S:.2f}s"
        )

    def _on_js(self, msg: JointState):
        self.js = msg

    def _on_markers(self, msg: MarkerArray):
        self.latest_marker_count = len(msg.markers)
        if (not self.found) and self.latest_marker_count > 0:
            self.found = True
            self.get_logger().warn("FOUND SOMEONE (MarkerArray is non-empty). Stopping scan.")

    def _pos(self, name: str) -> float:
        idx = self.js.name.index(name)
        return float(self.js.position[idx])

    def pt(self, t_s: float, pan: float, tilt: float) -> JointTrajectoryPoint:
        p = JointTrajectoryPoint()
        p.positions = [float(pan), float(tilt)]
        p.time_from_start = dur(t_s)
        return p

    def send_goal_and_wait(self, joint_names, points, timeout_sec=GOAL_TIMEOUT_S) -> bool:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names
        goal.trajectory.points = points

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=3.0)
        gh = send_future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error("Goal rejected.")
            return False

        res_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_future, timeout_sec=timeout_sec)
        return True

    def send_pan_step(self, target_pan: float) -> bool:
        """
        2-point trajectory:
          - point 1: current (or last-known) pose at T_START_S
          - point 2: target pan at T_GOAL_S
        """
        # Make sure we have fresh joint states
        rclpy.spin_once(self, timeout_sec=0.01)

        cur_pan = self._pos(PAN_JOINT)
        cur_tilt = 0.0
        tgt_tilt = cur_tilt if HOLD_TILT else cur_tilt

        joints = [PAN_JOINT, TILT_JOINT]
        pts = [
            self.pt(T_START_S, cur_pan, cur_tilt),
            self.pt(T_GOAL_S, float(target_pan), float(tgt_tilt)),
        ]
        return self.send_goal_and_wait(joints, pts, timeout_sec=GOAL_TIMEOUT_S)

    def run(self):
        # Deterministic scan start: go to left limit first
        target = LEFT_LIMIT
        direction = +1

        self.get_logger().info(f"Moving to left limit: {LEFT_LIMIT:.2f} rad")
        self.send_pan_step(LEFT_LIMIT)
        time.sleep(0.1)

        while rclpy.ok() and not self.found:
            # process markers
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.found:
                break

            # step target
            next_pan = target + direction * STEP

            # bounce at limits
            if next_pan >= RIGHT_LIMIT:
                next_pan = RIGHT_LIMIT
                direction = -1
            elif next_pan <= LEFT_LIMIT:
                next_pan = LEFT_LIMIT
                direction = +1

            ok = self.send_pan_step(next_pan)
            if ok:
                target = next_pan
                self.get_logger().info(
                    f"Scanning... pan={target:.2f} rad | markers={self.latest_marker_count}"
                )
            else:
                self.get_logger().warn("Scan step failed; retrying after brief pause.")
                time.sleep(0.3)

            # dwell while still checking markers
            t_end = time.time() + DWELL_S
            while time.time() < t_end and rclpy.ok() and not self.found:
                rclpy.spin_once(self, timeout_sec=0.05)

        if self.found:
            self.get_logger().warn("Stopped: face detected.")
        else:
            self.get_logger().info("Stopped: ROS shutdown.")


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