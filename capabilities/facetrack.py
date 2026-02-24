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

PAN_JOINT = "joint_head_pan"
TILT_JOINT = "joint_head_tilt"

LEFT_LIMIT = -1.6
RIGHT_LIMIT = 1.6
STEP = 0.2
DWELL_S = 0.2


def dur(t: float) -> Duration:
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


class HeadSweep(Node):
    def __init__(self):
        super().__init__("head_sweep")

        self.client = ActionClient(self, FollowJointTrajectory, TRAJ_ACTION_NAME)

        self.js = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

        self.get_logger().info("Waiting for /joint_states...")
        while rclpy.ok() and self.js is None:
            rclpy.spin_once(self, timeout_sec=0.2)

        self.pan0 = self._pos(PAN_JOINT)
        self.tilt0 = self._pos(TILT_JOINT)

        self.get_logger().info(
            f"Start pose: pan={self.pan0:.3f}, tilt={self.tilt0:.3f}"
        )

        if not self.client.wait_for_server(timeout_sec=8.0):
            raise RuntimeError("No trajectory action server")

        self.run()

    def _on_js(self, msg: JointState):
        self.js = msg

    def _pos(self, name: str) -> float:
        idx = self.js.name.index(name)
        return float(self.js.position[idx])

    def pt(self, t_s, pan, tilt):
        p = JointTrajectoryPoint()
        p.positions = [float(pan), float(tilt)]
        p.time_from_start = dur(t_s)
        return p

    def send_pan(self, target_pan):
        joints = [PAN_JOINT, TILT_JOINT]

        # read current
        rclpy.spin_once(self, timeout_sec=0.01)
        cur_pan = self._pos(PAN_JOINT)
        cur_tilt = self._pos(TILT_JOINT)

        pts = [
            self.pt(0.05, cur_pan, cur_tilt),
            self.pt(0.4, target_pan, cur_tilt),
        ]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joints
        goal.trajectory.points = pts

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=3.0)
        gh = send_future.result()

        if gh is None or not gh.accepted:
            self.get_logger().error("Goal rejected.")
            return False

        res_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_future, timeout_sec=3.0)
        return True

    def run(self):
        target = LEFT_LIMIT
        direction = +1

        self.get_logger().info("Starting head sweep...")

        while rclpy.ok():
            ok = self.send_pan(target)
            if not ok:
                time.sleep(0.5)
                continue

            self.get_logger().info(f"Pan: {target:.2f} rad")

            # compute next
            target += direction * STEP

            if target >= RIGHT_LIMIT:
                target = RIGHT_LIMIT
                direction = -1
            elif target <= LEFT_LIMIT:
                target = LEFT_LIMIT
                direction = +1

            time.sleep(DWELL_S)


def main():
    rclpy.init()
    node = HeadSweep()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()