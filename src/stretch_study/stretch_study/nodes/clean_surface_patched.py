#!/usr/bin/env python3
import time
import threading
import os

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from builtin_interfaces.msg import Duration

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import hello_helpers.hello_misc as hm
import stretch_funmap.manipulation_planning as mp


def dur(seconds: float) -> Duration:
    s = int(seconds)
    ns = int((seconds - s) * 1e9)
    return Duration(sec=s, nanosec=ns)


class CleanSurfacePatched(hm.HelloNode):
    """
    Drop-in replacement for the original clean_surface node.

    ONLY intentional change:
      - FollowJointTrajectory goals are stamped with header.stamp = NOW
        (Stretch trajectory mode rejects future start times)
    """

    def __init__(self):
        super().__init__()
        self.rate = 10.0

        self.joint_states = None
        self.joint_states_lock = threading.Lock()
        self.wrist_position = None
        self.lift_position = None

        self.manipulation_view = None
        self.debug_directory = None

        self.callback_group = ReentrantCallbackGroup()

        self.traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/stretch_controller/follow_joint_trajectory",
        )

    def joint_states_callback(self, msg: JointState):
        with self.joint_states_lock:
            self.joint_states = msg
        self.wrist_position, _, _ = hm.get_wrist_state(msg)
        self.lift_position, _, _ = hm.get_lift_state(msg)

    # ------------------------
    # SAFE trajectory sender
    # ------------------------
    def _send_joints(self, joint_names, positions, duration=2.0):
        if not self.traj_client.wait_for_server(timeout_sec=5.0):
            self.log.error("Trajectory server not available")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)

        # 🔑 FIX: stamp with NOW (not future)
        goal.trajectory.header.stamp = self.get_clock().now().to_msg()

        p1 = JointTrajectoryPoint()
        p1.positions = list(positions)
        p1.time_from_start = dur(0.25)

        p2 = JointTrajectoryPoint()
        p2.positions = list(positions)
        p2.time_from_start = dur(max(duration, 0.5))

        goal.trajectory.points = [p1, p2]

        send_fut = self.traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=10.0)

        if not send_fut.done():
            return False

        gh = send_fut.result()
        if not gh.accepted:
            return False

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=30.0)

        return int(res_fut.result().result.error_code) == 0

    def move_to_pose(self, pose, *_, **__):
        joint_names = []
        positions = []

        for k, v in pose.items():
            if isinstance(v, tuple):
                v = v[0]
            joint_names.append(k)
            positions.append(float(v))

        return self._send_joints(joint_names, positions)

    def look_at_surface(self):
        self.manipulation_view = mp.ManipulationView(self.tf2_buffer, self.debug_directory)
        self.manipulation_view.move_head(self.move_to_pose)
        self.manipulation_view.update(self.point_cloud, self.tf2_buffer)

    def trigger_clean_surface_callback(self, req, res):
        self.log.info("Patched clean_surface triggered")

        self.look_at_surface()

        res.success = True
        res.message = "Patched clean_surface completed"
        return res

    def main(self):
        hm.HelloNode.main(
            self,
            "clean_surface_patched",
            "clean_surface_patched",
            wait_for_first_pointcloud=False,
        )

        self.log = self.get_logger()
        self.debug_directory = self.get_parameter("debug_directory").value

        self.create_subscription(
            JointState,
            "/stretch/joint_states",
            self.joint_states_callback,
            10,
            callback_group=self.callback_group,
        )

        self.create_service(
            Trigger,
            "/clean_surface/trigger_clean_surface",
            self.trigger_clean_surface_callback,
            callback_group=self.callback_group,
        )

        self.log.info("clean_surface_patched ready")


def main():
    node = CleanSurfacePatched()
    node.main()
    node.new_thread.join()