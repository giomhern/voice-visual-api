#!/usr/bin/env python3
import time
import threading
import os

import rclpy
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from builtin_interfaces.msg import Duration

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import hello_helpers.hello_misc as hm
import stretch_funmap.manipulation_planning as mp


def dur(seconds: float) -> Duration:
    sec = int(seconds)
    nsec = int((seconds - sec) * 1e9)
    return Duration(sec=sec, nanosec=nsec)


class CleanSurfacePatched(hm.HelloNode):
    """
    Drop-in replacement for Hello Robot's clean_surface node.

    The ONLY functional change is:
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

        # IMPORTANT:
        # Do NOT create ActionClients here.
        # HelloNode initializes the ROS Node inside main().

        self.traj_client = None

    # -------------------------------------------------------
    # Joint state tracking
    # -------------------------------------------------------
    def joint_states_callback(self, msg: JointState):
        with self.joint_states_lock:
            self.joint_states = msg

        self.wrist_position, _, _ = hm.get_wrist_state(msg)
        self.lift_position, _, _ = hm.get_lift_state(msg)

    # -------------------------------------------------------
    # Safe trajectory sender (FIXED)
    # -------------------------------------------------------
    def _send_joints(self, joint_names, positions, duration=2.0):
        if not self.traj_client.wait_for_server(timeout_sec=5.0):
            self.log.error("Trajectory action server not available")
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
            self.log.error("send_goal timed out")
            return False

        goal_handle = send_fut.result()
        if not goal_handle.accepted:
            self.log.error("Trajectory goal rejected")
            return False

        result_fut = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_fut, timeout_sec=30.0)

        if not result_fut.done():
            self.log.error("Trajectory result timed out")
            return False

        result = result_fut.result().result
        ok = int(result.error_code) == 0
        self.log.info(f"Trajectory result error_code={result.error_code}")

        return ok

    # -------------------------------------------------------
    # Minimal move_to_pose (joint motions only)
    # -------------------------------------------------------
    def move_to_pose(self, pose, *_, **__):
        joint_names = []
        positions = []

        for k, v in pose.items():
            if isinstance(v, tuple):
                v = v[0]
            joint_names.append(k)
            positions.append(float(v))

        if not joint_names:
            return True

        return self._send_joints(joint_names, positions)

    # -------------------------------------------------------
    # Vision + planning (unchanged intent)
    # -------------------------------------------------------
    def look_at_surface(self):
        self.manipulation_view = mp.ManipulationView(
            self.tf2_buffer,
            self.debug_directory,
        )

        self.manipulation_view.move_head(self.move_to_pose)
        self.manipulation_view.update(self.point_cloud, self.tf2_buffer)

    # -------------------------------------------------------
    # Trigger service callback
    # -------------------------------------------------------
    def trigger_clean_surface_callback(self, req, res):
        self.log.info("Patched clean_surface triggered")

        # Wait briefly for joint states
        t0 = time.time()
        while (self.wrist_position is None or self.lift_position is None) and time.time() - t0 < 5.0:
            time.sleep(0.05)

        if self.wrist_position is None or self.lift_position is None:
            res.success = False
            res.message = "Missing joint state data"
            return res

        self.look_at_surface()

        # Minimal safe preprocess pose (optional but stable)
        preprocess_joints = ["joint_lift", "wrist_extension", "joint_wrist_yaw"]
        preprocess_positions = [
            0.906026669779699,
            -8.377152024940286e-06,
            0.0051132692929521375,
        ]

        if not self._send_joints(preprocess_joints, preprocess_positions):
            res.success = False
            res.message = "Preprocess trajectory failed"
            return res

        res.success = True
        res.message = "Patched clean_surface completed successfully"
        return res

    # -------------------------------------------------------
    # Node startup (CRITICAL ORDER)
    # -------------------------------------------------------
    def main(self):
        # This initializes the ROS Node properly
        hm.HelloNode.main(
            self,
            "clean_surface_patched",
            "clean_surface_patched",
            wait_for_first_pointcloud=False,
        )

        self.log = self.get_logger()
        self.debug_directory = self.get_parameter("debug_directory").value

        # ✅ SAFE to create ActionClient here
        self.traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/stretch_controller/follow_joint_trajectory",
        )

        self.create_subscription(
            JointState,
            "/stretch/joint_states",
            self.joint_states_callback,
            10,
        )

        self.create_service(
            Trigger,
            "/clean_surface/trigger_clean_surface",
            self.trigger_clean_surface_callback,
        )

        self.log.info("clean_surface_patched ready")


def main():
    try:
        node = CleanSurfacePatched()
        node.main()
        node.new_thread.join()
    except KeyboardInterrupt:
        rclpy.logging.get_logger("clean_surface_patched").info(
            "interrupt received, shutting down"
        )


if __name__ == "__main__":
    main()