#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState


TRAJ_ACTION_NAME = "/stretch_controller/follow_joint_trajectory"


def dur(t: float) -> Duration:
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


class HeadNodLikeWorking(Node):
    def __init__(self):
        super().__init__("head_nod_like_working")
        self.client = ActionClient(self, FollowJointTrajectory, TRAJ_ACTION_NAME)

        # Joint state capture (so we nod relative to current pose)
        self.js = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

        self.get_logger().info("Waiting for /joint_states...")
        while rclpy.ok() and self.js is None:
            rclpy.spin_once(self, timeout_sec=0.2)

        self.pan0 = self._pos("joint_head_pan")
        self.tilt0 = self._pos("joint_head_tilt")
        self.get_logger().info(f"Start head pose: pan={self.pan0:.3f}, tilt={self.tilt0:.3f}")

        self.get_logger().info(f"Waiting for action server: {TRAJ_ACTION_NAME}")
        if not self.client.wait_for_server(timeout_sec=8.0):
            raise RuntimeError("No trajectory action server")

        self.run()

    def _on_js(self, msg: JointState):
        self.js = msg

    def _pos(self, name: str) -> float:
        idx = self.js.name.index(name)
        return float(self.js.position[idx])

    def send_goal_and_wait(self, joint_names, points, timeout_sec=6.0):
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
        if not res_future.done():
            self.get_logger().error("Timed out waiting for goal result.")
            return False

        res = res_future.result().result
        status = res_future.result().status
        self.get_logger().info(f"Result status={status}, error_code={res.error_code}, error_string='{res.error_string}'")
        return True

    def pt(self, t_s, pan, tilt):
        p = JointTrajectoryPoint()
        p.positions = [float(pan), float(tilt)]
        p.time_from_start = dur(t_s)
        return p

    def run(self):
        joints = ["joint_head_pan", "joint_head_tilt"]

        # Make nod obvious
        down = self.tilt0 + 0.40
        up = self.tilt0 - 0.25
        neutral = self.tilt0

        # -------------------------
        # GOAL 1: go DOWN and HOLD there (end at DOWN)
        # -------------------------
        t = 0.0
        pts = []
        t += 0.4
        pts.append(self.pt(t, self.pan0, neutral))
        t += 1.0
        pts.append(self.pt(t, self.pan0, down))

        self.get_logger().info("Goal 1: head down")
        if not self.send_goal_and_wait(joints, pts, timeout_sec=4.0):
            return
        time.sleep(0.4)  # give you time to *see* it

        # -------------------------
        # GOAL 2: go UP and HOLD there
        # -------------------------
        t = 0.0
        pts = []
        t += 0.4
        pts.append(self.pt(t, self.pan0, down))
        t += 1.0
        pts.append(self.pt(t, self.pan0, up))

        self.get_logger().info("Goal 2: head up")
        if not self.send_goal_and_wait(joints, pts, timeout_sec=4.0):
            return
        time.sleep(0.4)

        # -------------------------
        # GOAL 3: return to NEUTRAL
        # -------------------------
        t = 0.0
        pts = []
        t += 0.4
        pts.append(self.pt(t, self.pan0, up))
        t += 1.0
        pts.append(self.pt(t, self.pan0, neutral))

        self.get_logger().info("Goal 3: back to neutral")
        self.send_goal_and_wait(joints, pts, timeout_sec=4.0)

        self.get_logger().info("Done.")


def main():
    rclpy.init()
    node = HeadNodLikeWorking()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()