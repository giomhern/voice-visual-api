#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# -----------------------------
# EDIT THESE IF NEEDED
# -----------------------------
TRAJ_ACTION_NAME = "/stretch_controller/follow_joint_trajectory"

JOINTS = [
    "joint_lift",
    "joint_arm_l0",
    "joint_wrist_yaw",
    "joint_wrist_pitch",
    "joint_wrist_roll",
]


class WaveGreeting(Node):
    def __init__(self):
        super().__init__("wave_greeting")

        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            TRAJ_ACTION_NAME,
        )

        self.get_logger().info("Waiting for trajectory action server...")
        self.client.wait_for_server()
        self.get_logger().info("Connected.")

        self.perform_wave()

    def perform_wave(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS

        points = []

        # 1️⃣ Raise and extend arm
        p1 = JointTrajectoryPoint()
        p1.positions = [
            0.7,    # lift
            0.2,    # arm extension
            0.0,    # wrist yaw
            -0.5,   # wrist pitch
            0.0     # wrist roll
        ]
        p1.time_from_start = Duration(sec=2)
        points.append(p1)

        # 2️⃣ Wave left
        p2 = JointTrajectoryPoint()
        p2.positions = [
            0.7,
            0.2,
            0.6,    # wrist yaw left
            -0.5,
            0.0
        ]
        p2.time_from_start = Duration(sec=3)
        points.append(p2)

        # 3️⃣ Wave right
        p3 = JointTrajectoryPoint()
        p3.positions = [
            0.7,
            0.2,
            -0.6,   # wrist yaw right
            -0.5,
            0.0
        ]
        p3.time_from_start = Duration(sec=4)
        points.append(p3)

        # 4️⃣ Wave left again
        p4 = JointTrajectoryPoint()
        p4.positions = [
            0.7,
            0.2,
            0.6,
            -0.5,
            0.0
        ]
        p4.time_from_start = Duration(sec=5)
        points.append(p4)

        # 5️⃣ Return neutral
        p5 = JointTrajectoryPoint()
        p5.positions = [
            0.5,
            0.0,
            0.0,
            0.0,
            0.0
        ]
        p5.time_from_start = Duration(sec=7)
        points.append(p5)

        goal.trajectory.points = points

        self.get_logger().info("Sending wave trajectory...")
        self.client.send_goal_async(goal)


def main():
    rclpy.init()
    node = WaveGreeting()
    rclpy.spin_once(node, timeout_sec=8)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()