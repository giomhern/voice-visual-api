#!/usr/bin/env python3
import math
import time
from typing import Optional, Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class FaceTrack(Node):
    def __init__(self):
        super().__init__("facetrack")

        self.declare_parameter("marker_array_topic", "/faces/marker_array")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("traj_action", "/stretch_controller/follow_joint_trajectory")
        self.declare_parameter("head_pan_joint", "joint_head_pan")
        self.declare_parameter("head_tilt_joint", "joint_head_tilt")
        self.declare_parameter("label_contains", "face")

        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("kp_yaw", 0.6)
        self.declare_parameter("kp_pitch", 0.6)
        self.declare_parameter("max_step_rad", 0.08)
        self.declare_parameter("deadband_rad", 0.02)
        self.declare_parameter("ema_alpha", 0.35)

        self.declare_parameter("yaw_sign", 1.0)
        self.declare_parameter("pitch_sign", 1.0)

        self.declare_parameter("pan_limits", [-2.8, 2.8])
        self.declare_parameter("tilt_limits", [-1.5, 1.2])

        self.declare_parameter("point_dt_s", 0.35)
        self.declare_parameter("min_goal_period_s", 0.12)

        self.declare_parameter("lock_timeout_s", 1.0)
        self.declare_parameter("lock_gate_m", 0.25)
        self.declare_parameter("lock_prefer_id", True)

        self.declare_parameter("search_enabled", True)
        self.declare_parameter("search_tilt_rad", -0.35)
        self.declare_parameter("search_pan_min", -1.6)
        self.declare_parameter("search_pan_max", 1.6)
        self.declare_parameter("search_speed_rad_s", 0.6)
        self.declare_parameter("search_step_rad", 0.08)
        self.declare_parameter("search_hold_s", 0.3)

        self.pan = 0.0
        self.tilt = 0.0
        self.have_joints = False

        self.locked = False
        self.lock_id: Optional[int] = None
        self.lock_xyz: Optional[Tuple[float, float, float]] = None
        self.lock_last_seen = 0.0

        self.yaw_f = 0.0
        self.pitch_f = 0.0

        self.last_goal_sent = 0.0

        self.search_dir = 1.0
        self.search_next_hold_until = 0.0
        self.search_target_pan: Optional[float] = None

        self.create_subscription(MarkerArray, self.get_parameter("marker_array_topic").value, self.on_markers, 10)
        self.create_subscription(JointState, self.get_parameter("joint_states_topic").value, self.on_joint_states, 50)
        self.traj = ActionClient(self, FollowJointTrajectory, self.get_parameter("traj_action").value)

        rate = float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate, 1e-6), self.tick)

    def on_joint_states(self, msg: JointState):
        pan_name = self.get_parameter("head_pan_joint").value
        tilt_name = self.get_parameter("head_tilt_joint").value
        idx = {n: i for i, n in enumerate(msg.name)}
        if pan_name in idx and tilt_name in idx:
            self.pan = msg.position[idx[pan_name]]
            self.tilt = msg.position[idx[tilt_name]]
            self.have_joints = True

    @staticmethod
    def _dist(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        dz = a[2] - b[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def on_markers(self, msg: MarkerArray):
        want = self.get_parameter("label_contains").value.lower()
        now = time.time()

        candidates: List[Tuple[int, float, float, float]] = []
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
            candidates.append((m.id, x, y, z))

        if not candidates:
            return

        lock_timeout = float(self.get_parameter("lock_timeout_s").value)
        gate = float(self.get_parameter("lock_gate_m").value)
        prefer_id = bool(self.get_parameter("lock_prefer_id").value)

        if self.locked and self.lock_xyz is not None and (now - self.lock_last_seen) <= lock_timeout:
            chosen = None

            if prefer_id and self.lock_id is not None:
                for cid, x, y, z in candidates:
                    if cid == self.lock_id:
                        chosen = (cid, x, y, z)
                        break

            if chosen is None:
                lx, ly, lz = self.lock_xyz
                best = None
                best_d = None
                for cid, x, y, z in candidates:
                    d = self._dist((x, y, z), (lx, ly, lz))
                    if best is None or d < best_d:
                        best = (cid, x, y, z)
                        best_d = d
                if best is not None and best_d is not None and best_d <= gate:
                    chosen = best

            if chosen is not None:
                cid, x, y, z = chosen
                self.lock_id = cid
                self.lock_xyz = (x, y, z)
                self.lock_last_seen = now
            return

        chosen = min(candidates, key=lambda t: t[3])
        cid, x, y, z = chosen
        self.locked = True
        self.lock_id = cid
        self.lock_xyz = (x, y, z)
        self.lock_last_seen = now

    @staticmethod
    def clamp(x, lo, hi):
        return max(lo, min(hi, x))

    def tick(self):
        if not self.have_joints:
            return

        now = time.time()
        lock_timeout = float(self.get_parameter("lock_timeout_s").value)

        face_fresh = self.locked and self.lock_xyz is not None and (now - self.lock_last_seen) <= lock_timeout

        if face_fresh:
            self.track_step(now)
            return

        self.locked = False
        self.lock_id = None
        self.lock_xyz = None

        if bool(self.get_parameter("search_enabled").value):
            self.search_step(now)

    def track_step(self, now: float):
        if self.lock_xyz is None:
            return

        x, y, z = self.lock_xyz
        if z <= 1e-3:
            return

        yaw_err = math.atan2(x, z)
        pitch_err = math.atan2(-y, z)

        yaw_err *= float(self.get_parameter("yaw_sign").value)
        pitch_err *= float(self.get_parameter("pitch_sign").value)

        dead = float(self.get_parameter("deadband_rad").value)
        if abs(yaw_err) < dead:
            yaw_err = 0.0
        if abs(pitch_err) < dead:
            pitch_err = 0.0

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

        min_period = float(self.get_parameter("min_goal_period_s").value)
        if now - self.last_goal_sent < min_period:
            return
        self.last_goal_sent = now

        self.send_traj(pan_goal, tilt_goal)

    def search_step(self, now: float):
        if now < self.search_next_hold_until:
            return

        pan_lo, pan_hi = self.get_parameter("pan_limits").value
        tilt_lo, tilt_hi = self.get_parameter("tilt_limits").value

        s_pan_min = float(self.get_parameter("search_pan_min").value)
        s_pan_max = float(self.get_parameter("search_pan_max").value)
        s_pan_min = self.clamp(s_pan_min, float(pan_lo), float(pan_hi))
        s_pan_max = self.clamp(s_pan_max, float(pan_lo), float(pan_hi))

        s_tilt = self.clamp(float(self.get_parameter("search_tilt_rad").value), float(tilt_lo), float(tilt_hi))

        step = float(self.get_parameter("search_step_rad").value)
        speed = float(self.get_parameter("search_speed_rad_s").value)
        if speed > 0:
            step = max(step, speed * (1.0 / float(self.get_parameter("rate_hz").value)))

        if self.search_target_pan is None:
            self.search_target_pan = self.pan

        target = self.search_target_pan + self.search_dir * step

        if target >= s_pan_max:
            target = s_pan_max
            self.search_dir = -1.0
            self.search_next_hold_until = now + float(self.get_parameter("search_hold_s").value)
        elif target <= s_pan_min:
            target = s_pan_min
            self.search_dir = 1.0
            self.search_next_hold_until = now + float(self.get_parameter("search_hold_s").value)

        self.search_target_pan = target

        min_period = float(self.get_parameter("min_goal_period_s").value)
        if now - self.last_goal_sent < min_period:
            return
        self.last_goal_sent = now

        self.send_traj(self.search_target_pan, s_tilt)

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
    rclpy.spin(FaceTrack())
    rclpy.shutdown()


if __name__ == "__main__":
    main()