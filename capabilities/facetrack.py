#!/usr/bin/env python3
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class FaceScanUntilFound(Node):
    def __init__(self):
        super().__init__("facetrack")

        self.declare_parameter("marker_array_topic", "/faces/marker_array")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("traj_action", "/stretch_controller/follow_joint_trajectory")
        self.declare_parameter("head_pan_joint", "joint_head_pan")
        self.declare_parameter("head_tilt_joint", "joint_head_tilt")
        self.declare_parameter("label_contains", "face")

        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("min_goal_period_s", 0.12)
        self.declare_parameter("point_dt_s", 0.35)

        self.declare_parameter("search_tilt_rad", -0.35)
        self.declare_parameter("search_pan_min", -1.6)
        self.declare_parameter("search_pan_max", 1.6)
        self.declare_parameter("search_step_rad", 0.08)
        self.declare_parameter("search_hold_s", 0.25)

        self.pan = 0.0
        self.tilt = 0.0
        self.have_joints = False

        self.found = False
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

    def on_markers(self, msg: MarkerArray):
        if self.found:
            return
        want = self.get_parameter("label_contains").value.lower()
        for m in msg.markers:
            if m.type != Marker.CUBE:
                continue
            if want and want not in (m.text or "").lower():
                continue
            z = float(m.pose.position.z)
            if z <= 1e-3:
                continue
            self.found = True
            self.get_logger().info("Face found. Stopping scan.")
            return

    @staticmethod
    def clamp(x, lo, hi):
        return max(lo, min(hi, x))

    def tick(self):
        if not self.have_joints:
            return
        if self.found:
            return

        now = time.time()
        if now < self.search_next_hold_until:
            return

        pan_lo, pan_hi = self.get_parameter("search_pan_min").value, self.get_parameter("search_pan_max").value
        tilt_goal = float(self.get_parameter("search_tilt_rad").value)

        if self.search_target_pan is None:
            self.search_target_pan = self.pan

        step = float(self.get_parameter("search_step_rad").value)
        target = self.search_target_pan + self.search_dir * step

        if target >= float(pan_hi):
            target = float(pan_hi)
            self.search_dir = -1.0
            self.search_next_hold_until = now + float(self.get_parameter("search_hold_s").value)
        elif target <= float(pan_lo):
            target = float(pan_lo)
            self.search_dir = 1.0
            self.search_next_hold_until = now + float(self.get_parameter("search_hold_s").value)

        self.search_target_pan = target

        if now - self.last_goal_sent < float(self.get_parameter("min_goal_period_s").value):
            return
        self.last_goal_sent = now

        self.send_traj(self.search_target_pan, tilt_goal)

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
    rclpy.spin(FaceScanUntilFound())
    rclpy.shutdown()


if __name__ == "__main__":
    main()