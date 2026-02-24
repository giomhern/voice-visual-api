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


class HeadNod(Node):
    def __init__(self):
        super().__init__("head_big_nods")
        self.client = ActionClient(self, FollowJointTrajectory, TRAJ_ACTION_NAME)

        self.js = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

        self.get_logger().info("Waiting for /joint_states...")
        while rclpy.ok() and self.js is None:
            rclpy.spin_once(self, timeout_sec=0.2)

        self.pan0 = self._pos("joint_head_pan")
        self.tilt0 = self._pos("joint_head_tilt")
        self.get_logger().info(f"Start head pose: pan={self.pan0:.3f}, tilt={self.tilt0:.3f}")

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
        return True

    def pt(self, t_s, pan, tilt):
        p = JointTrajectoryPoint()
        p.positions = [float(pan), float(tilt)]
        p.time_from_start = dur(t_s)
        return p

    def run(self):
        joints = ["joint_head_pan", "joint_head_tilt"]

        # 🔥 More pronounced nod
        NOD_COUNT = 3
        DOWN_AMOUNT = 0.65   # bigger downward motion
        UP_AMOUNT = -0.25    # less extreme upward
        neutral = self.tilt0

        down = self.tilt0 + DOWN_AMOUNT
        up = self.tilt0 + UP_AMOUNT

        for i in range(NOD_COUNT):
            self.get_logger().info(f"Nod {i+1}")

            # ----- DOWN -----
            t = 0.0
            pts = []
            t += 0.4
            pts.append(self.pt(t, self.pan0, neutral))
            t += 0.8
            pts.append(self.pt(t, self.pan0, down))

            if not self.send_goal_and_wait(joints, pts, timeout_sec=4.0):
                return
            time.sleep(0.2)

            # ----- UP -----
            t = 0.0
            pts = []
            t += 0.4
            pts.append(self.pt(t, self.pan0, down))
            t += 0.8
            pts.append(self.pt(t, self.pan0, up))

            if not self.send_goal_and_wait(joints, pts, timeout_sec=4.0):
                return
            time.sleep(0.2)

        # ----- FINAL RETURN TO NEUTRAL -----
        t = 0.0
        pts = []
        t += 0.4
        pts.append(self.pt(t, self.pan0, up))
        t += 0.8
        pts.append(self.pt(t, self.pan0, neutral))

        self.send_goal_and_wait(joints, pts, timeout_sec=4.0)

        self.get_logger().info("Big nod sequence complete.")


def main():
    rclpy.init()
    node = HeadNod()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()