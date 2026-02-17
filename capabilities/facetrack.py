#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray


def dur(t: float) -> Duration:
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


class FaceScanAndHold(Node):
    """
    Scans head pan left-right until a face marker arrives, then stops and holds.
    Fixes Stretch error: "no trajectory in goal contains enough waypoints"
    by always sending 2 waypoints (start + end).
    """

    def __init__(self):
        super().__init__("face_scan_and_hold")

        # Params
        self.declare_parameter("traj_action", "/stretch_controller/follow_joint_trajectory")
        self.declare_parameter("face_markers_topic", "/faces/markers")  # change if needed

        self.declare_parameter("pan_min", -1.4)
        self.declare_parameter("pan_max",  1.4)
        self.declare_parameter("tilt_hold", -0.25)

        self.declare_parameter("step", 0.35)
        self.declare_parameter("move_time", 0.7)
        self.declare_parameter("settle_time", 0.2)
        self.declare_parameter("confirm_msgs", 2)

        self.traj_action = self.get_parameter("traj_action").value
        self.face_topic = self.get_parameter("face_markers_topic").value

        self.pan_min = float(self.get_parameter("pan_min").value)
        self.pan_max = float(self.get_parameter("pan_max").value)
        self.tilt_hold = float(self.get_parameter("tilt_hold").value)

        self.step = float(self.get_parameter("step").value)
        self.move_time = float(self.get_parameter("move_time").value)
        self.settle_time = float(self.get_parameter("settle_time").value)
        self.confirm_msgs = int(self.get_parameter("confirm_msgs").value)

        # State
        self.face_msg_count = 0
        self.face_found = False

        self.cur_head_pan = None
        self.cur_head_tilt = None

        # Subs
        self.create_subscription(MarkerArray, self.face_topic, self._on_faces, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 50)

        # Action client
        self.client = ActionClient(self, FollowJointTrajectory, self.traj_action)
        self.get_logger().info(f"Waiting for trajectory action: {self.traj_action}")
        if not self.client.wait_for_server(timeout_sec=8.0):
            raise RuntimeError(
                f"FollowJointTrajectory not available at {self.traj_action}. "
                "Check: ros2 action list | grep FollowJointTrajectory"
            )

        self.get_logger().info(f"Listening for faces on: {self.face_topic}")
        self.get_logger().info("Waiting for head joint_states...")
        while rclpy.ok() and (self.cur_head_pan is None or self.cur_head_tilt is None):
            rclpy.spin_once(self, timeout_sec=0.2)

        self.get_logger().info("Starting head scan...")
        self.scan_loop()

    def _on_faces(self, msg: MarkerArray):
        if msg.markers and len(msg.markers) > 0:
            self.face_msg_count += 1
            if self.face_msg_count >= self.confirm_msgs:
                self.face_found = True

    def _on_joint_states(self, msg: JointState):
        # Update current head pan/tilt if present
        try:
            i_pan = msg.name.index("joint_head_pan")
            i_tilt = msg.name.index("joint_head_tilt")
        except ValueError:
            return
        self.cur_head_pan = float(msg.position[i_pan])
        self.cur_head_tilt = float(msg.position[i_tilt])

    def send_head(self, pan_target: float, tilt_target: float, move_time: float) -> bool:
        """
        Send a trajectory with 2 points:
        - Point 1: hold current position (0.1s)
        - Point 2: target position (move_time)
        This avoids: "no trajectory ... contains enough waypoints"
        """
        if self.cur_head_pan is None or self.cur_head_tilt is None:
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["joint_head_pan", "joint_head_tilt"]

        p0 = JointTrajectoryPoint()
        p0.positions = [self.cur_head_pan, self.cur_head_tilt]
        p0.time_from_start = dur(0.1)

        p1 = JointTrajectoryPoint()
        p1.positions = [float(pan_target), float(tilt_target)]
        p1.time_from_start = dur(max(0.2, move_time))

        goal.trajectory.points = [p0, p1]

        fut = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        gh = fut.result()

        if gh is None or not gh.accepted:
            self.get_logger().warn("Head goal rejected.")
            return False

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=max(3.0, move_time + 3.0))
        return True

    def scan_loop(self):
        # Start centered
        self.send_head(0.0, self.tilt_hold, move_time=1.0)
        time.sleep(0.2)

        pan = self.pan_min
        direction = +1.0

        while rclpy.ok() and not self.face_found:
            pan_next = pan + direction * self.step
            if pan_next > self.pan_max:
                pan_next = self.pan_max
                direction = -1.0
            elif pan_next < self.pan_min:
                pan_next = self.pan_min
                direction = +1.0

            self.send_head(pan_next, self.tilt_hold, move_time=self.move_time)

            # Let callbacks run
            t_end = time.time() + self.settle_time
            while time.time() < t_end and rclpy.ok() and not self.face_found:
                rclpy.spin_once(self, timeout_sec=0.05)

            pan = pan_next

        if self.face_found:
            self.get_logger().info("Face found. Holding position (stopping scan).")
            # Optional: re-send current pose as a "hold" command
            self.send_head(self.cur_head_pan, self.cur_head_tilt, move_time=0.5)


def main():
    rclpy.init()
    node = FaceScanAndHold()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()