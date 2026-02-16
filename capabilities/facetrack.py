#!/usr/bin/env python3
import math
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class StretchFaceMarkerHeadTrack(Node):
    """
    Tracks the closest detected face using the 3D CUBE marker published by
    stretch_deep_perception's detect_faces pipeline on /faces/marker_array.
    """

    def __init__(self):
        super().__init__("stretch_face_marker_headtrack")

        # -------- Params you may need to change --------
        self.declare_parameter("marker_array_topic", "/faces/marker_array")
        self.declare_parameter("joint_states_topic", "/joint_states")

        # Verify with: ros2 action list | grep -i trajectory
        self.declare_parameter("traj_action", "/stretch_controller/follow_joint_trajectory")

        # Verify with: ros2 topic echo /joint_states --once
        self.declare_parameter("head_pan_joint", "joint_head_pan")
        self.declare_parameter("head_tilt_joint", "joint_head_tilt")

        # Control
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("kp_yaw", 1.0)
        self.declare_parameter("kp_pitch", 1.0)
        self.declare_parameter("max_step_rad", 0.15)

        # Sign flips if your pan/tilt goes the wrong way
        self.declare_parameter("yaw_sign", 1.0)
        self.declare_parameter("pitch_sign", 1.0)

        self.declare_parameter("pan_limits", [-2.8, 2.8])
        self.declare_parameter("tilt_limits", [-1.5, 1.2])

        # Only accept markers whose text contains this (their code sets marker.text = label)
        self.declare_parameter("label_contains", "face")

        # Drop detections if we haven't seen a face recently
        self.declare_parameter("stale_s", 0.7)

        # -------- State --------
        self.pan = 0.0
        self.tilt = 0.0
        self.have_joints = False

        self.last_face_xyz: Optional[Tuple[float, float, float]] = None
        self.last_face_time = 0.0

        # -------- ROS I/O --------
        self.create_subscription(
            MarkerArray,
            self.get_parameter("marker_array_topic").value,
            self.on_markers,
            10,
        )
        self.create_subscription(
            JointState,
            self.get_parameter("joint_states_topic").value,
            self.on_joint_state,
            50,
        )

        self.traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.get_parameter("traj_action").value,
        )

        rate = float(self.get_parameter("control_rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate, 1e-6), self.control_tick)

    def on_joint_state(self, msg: JointState):
        pan_name = self.get_parameter("head_pan_joint").value
        tilt_name = self.get_parameter("head_tilt_joint").value
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        if pan_name in name_to_idx and tilt_name in name_to_idx:
            self.pan = msg.position[name_to_idx[pan_name]]
            self.tilt = msg.position[name_to_idx[tilt_name]]
            self.have_joints = True

    def on_markers(self, msg: MarkerArray):
        """
        In Hello Robot's detection_ros_markers, the main detection box is a CUBE Marker,
        and marker.text is set to the detection label (e.g., 'face'). Landmarks are POINTS markers.
        We'll pick the closest CUBE with matching label.
        """
        want = self.get_parameter("label_contains").value.lower()

        best = None
        best_dist = None

        for m in msg.markers:
            if m.type != Marker.CUBE:
                continue
            if want and (m.text or "").lower().find(want) == -1:
                continue

            x = float(m.pose.position.x)
            y = float(m.pose.position.y)
            z = float(m.pose.position.z)

            # In camera optical frame, z is typically forward; choose closest in depth
            dist = z  # or math.sqrt(x*x + y*y + z*z)

            if best is None or dist < best_dist:
                best = (x, y, z)
                best_dist = dist

        if best is not None:
            self.last_face_xyz = best
            self.last_face_time = time.time()

    @staticmethod
    def clamp(x, lo, hi):
        return max(lo, min(hi, x))

    def control_tick(self):
        if not self.have_joints:
            return
        if self.last_face_xyz is None:
            return

        stale_s = float(self.get_parameter("stale_s").value)
        if time.time() - self.last_face_time > stale_s:
            return

        x, y, z = self.last_face_xyz

        # Camera optical frame convention is typically: x right, y down, z forward.
        # Pan (yaw) error: angle to rotate so face goes to center => atan2(x, z)
        # Tilt (pitch) error: angle to rotate so face goes to center => atan2(y, z)
        # Signs are configurable.
        yaw_err = math.atan2(x, z) * float(self.get_parameter("yaw_sign").value)
        pitch_err = math.atan2(y, z) * float(self.get_parameter("pitch_sign").value)

        kp_yaw = float(self.get_parameter("kp_yaw").value)
        kp_pitch = float(self.get_parameter("kp_pitch").value)
        max_step = float(self.get_parameter("max_step_rad").value)

        pan_step = self.clamp(kp_yaw * yaw_err, -max_step, max_step)
        tilt_step = self.clamp(kp_pitch * pitch_err, -max_step, max_step)

        pan_lo, pan_hi = self.get_parameter("pan_limits").value
        tilt_lo, tilt_hi = self.get_parameter("tilt_limits").value

        pan_goal = self.clamp(self.pan + pan_step, float(pan_lo), float(pan_hi))
        tilt_goal = self.clamp(self.tilt + tilt_step, float(tilt_lo), float(tilt_hi))

        self.send_head_trajectory(pan_goal, tilt_goal)

    def send_head_trajectory(self, pan_goal: float, tilt_goal: float):
        if not self.traj_client.wait_for_server(timeout_sec=0.0):
            return

        pan_name = self.get_parameter("head_pan_joint").value
        tilt_name = self.get_parameter("head_tilt_joint").value

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [pan_name, tilt_name]

        pt = JointTrajectoryPoint()
        pt.positions = [float(pan_goal), float(tilt_goal)]
        pt.time_from_start = Duration(sec=0, nanosec=250_000_000)  # 0.25s
        goal.trajectory.points = [pt]

        self.traj_client.send_goal_async(goal)


def main():
    rclpy.init()
    node = StretchFaceMarkerHeadTrack()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()