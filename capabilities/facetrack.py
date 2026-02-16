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


class TrackFaceOpticalStable(Node):
    def __init__(self):
        super().__init__("track_face_optical_stable")

        # Topics / controller
        self.declare_parameter("marker_array_topic", "/faces/marker_array")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("traj_action", "/stretch_controller/follow_joint_trajectory")
        self.declare_parameter("head_pan_joint", "joint_head_pan")
        self.declare_parameter("head_tilt_joint", "joint_head_tilt")

        # Face selection
        self.declare_parameter("label_contains", "face")
        self.declare_parameter("stale_s", 0.6)

        # Control (start gentle)
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("kp_yaw", 0.6)
        self.declare_parameter("kp_pitch", 0.6)
        self.declare_parameter("max_step_rad", 0.08)   # smaller = less wild
        self.declare_parameter("deadband_rad", 0.02)   # ignore tiny errors
        self.declare_parameter("ema_alpha", 0.35)      # smoothing

        # Flip these ONLY if needed
        self.declare_parameter("yaw_sign", 1.0)
        self.declare_parameter("pitch_sign", 1.0)

        # Limits
        self.declare_parameter("pan_limits", [-2.8, 2.8])
        self.declare_parameter("tilt_limits", [-1.5, 1.2])

        # Trajectory timing
        self.declare_parameter("point_dt_s", 0.35)
        self.declare_parameter("min_goal_period_s", 0.12)

        # State
        self.pan = 0.0
        self.tilt = 0.0
        self.have_joints = False

        self.last_xyz: Optional[Tuple[float, float, float]] = None
        self.last_t = 0.0

        self.yaw_f = 0.0
        self.pitch_f = 0.0
        self.last_goal_sent = 0.0

        # ROS
        self.create_subscription(MarkerArray, self.get_parameter("marker_array_topic").value, self.on_markers, 10)
        self.create_subscription(JointState, self.get_parameter("joint_states_topic").value, self.on_joint_states, 50)
        self.traj = ActionClient(self, FollowJointTrajectory, self.get_parameter("traj_action").value)

        rate = float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate, 1e-6), self.tick)

        self.get_logger().info("TrackFaceOpticalStable running.")
        self.get_logger().info("Expected marker frame: camera_color_optical_frame (x right, y down, z forward)")

    def on_joint_states(self, msg: JointState):
        pan_name = self.get_parameter("head_pan_joint").value
        tilt_name = self.get_parameter("head_tilt_joint").value
        idx = {n: i for i, n in enumerate(msg.name)}
        if pan_name in idx and tilt_name in idx:
            self.pan = msg.position[idx[pan_name]]
            self.tilt = msg.position[idx[tilt_name]]
            self.have_joints = True

    def on_markers(self, msg: MarkerArray):
        want = self.get_parameter("label_contains").value.lower()

        best = None
        best_z = None
        for m in msg.markers:
            if m.type != Marker.CUBE:
                continue
            if want and want not in (m.text or "").lower():
                continue

            x = float(m.pose.position.x)
            y = float(m.pose.position.y)
            z = float(m.pose.position.z)
            if z <= 1e-3:
                continue

            # closest in depth
            if best is None or z < best_z:
                best = (x, y, z)
                best_z = z

        if best is not None:
            self.last_xyz = best
            self.last_t = time.time()

    @staticmethod
    def clamp(x, lo, hi):
        return max(lo, min(hi, x))

    def tick(self):
        if not self.have_joints or self.last_xyz is None:
            return

        if time.time() - self.last_t > float(self.get_parameter("stale_s").value):
            return

        x, y, z = self.last_xyz

        # OPTICAL FRAME RULE:
        # yaw error: face right (x>0) => yaw positive
        yaw_err = math.atan2(x, z)

        # pitch: face up => y<0, so use -y to make "up" => positive pitch_err
        pitch_err = math.atan2(-y, z)

        yaw_err *= float(self.get_parameter("yaw_sign").value)
        pitch_err *= float(self.get_parameter("pitch_sign").value)

        # deadband
        dead = float(self.get_parameter("deadband_rad").value)
        if abs(yaw_err) < dead:
            yaw_err = 0.0
        if abs(pitch_err) < dead:
            pitch_err = 0.0

        # smooth
        a = float(self.get_parameter("ema_alpha").value)
        self.yaw_f = (1 - a) * self.yaw_f + a * yaw_err
        self.pitch_f = (1 - a) * self.pitch_f + a * pitch_err

        kp_yaw = float(self.get_parameter("kp_yaw").value)
        kp_pitch = float(self.get_parameter("kp_pitch").value)
        max_step = float(self.get_parameter("max_step_rad").value)

        pan_step = self.clamp(kp_yaw * self.yaw_f, -max_step, max_step)
        tilt_step = self.clamp(kp_pitch * self.pitch_f, -max_step, max_step)

        pan_lo, pan_hi = self.get_parameter("pan_limits").value
        tilt_lo, tilt_hi = self.get_parameter("tilt_limits").value

        pan_goal = self.clamp(self.pan + pan_step, float(pan_lo), float(pan_hi))
        tilt_goal = self.clamp(self.tilt + tilt_step, float(tilt_lo), float(tilt_hi))

        now = time.time()
        if now - self.last_goal_sent < float(self.get_parameter("min_goal_period_s").value):
            return
        self.last_goal_sent = now

        self.send_traj(pan_goal, tilt_goal)

    def send_traj(self, pan_goal: float, tilt_goal: float):
        if not self.traj.wait_for_server(timeout_sec=0.0):
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            self.get_parameter("head_pan_joint").value,
            self.get_parameter("head_tilt_joint").value,
        ]

        pt = JointTrajectoryPoint()
        pt.positions = [float(pan_goal), float(tilt_goal)]
        dt = float(self.get_parameter("point_dt_s").value)
        sec = int(dt)
        nsec = int((dt - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nsec)

        goal.trajectory.points = [pt]
        self.traj.send_goal_async(goal)


def main():
    rclpy.init()
    rclpy.spin(TrackFaceOpticalStable())
    rclpy.shutdown()


if __name__ == "__main__":
    main()