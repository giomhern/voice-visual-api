#!/usr/bin/env python3
"""
Standalone Stretch script:
  1) switch to navigation mode
  2) turn left ~90 degrees (cmd_vel)
  3) switch to trajectory mode
  4) move arm using FollowJointTrajectory

Run with:
  python3 stretch_turn_and_pose.py
"""

import time
import math

import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.action import ActionClient

from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


def duration_msg(seconds: float) -> Duration:
    sec_i = int(seconds)
    nsec_i = int((seconds - sec_i) * 1e9)
    return Duration(sec=sec_i, nanosec=nsec_i)


class StretchTurnAndPose(Node):
    def __init__(self):
        super().__init__("stretch_turn_and_pose")

        # Publishers / clients
        self.cmd_vel_pub = self.create_publisher(Twist, "/stretch/cmd_vel", 10)

        self.traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/stretch_controller/follow_joint_trajectory",
        )

        self.srv_nav = self.create_client(Trigger, "/switch_to_navigation_mode")
        self.srv_traj = self.create_client(Trigger, "/switch_to_trajectory_mode")

    # -------------------------
    # Helpers
    # -------------------------
    def call_trigger(self, client, name: str, timeout=10.0) -> bool:
        self.get_logger().info(f"Waiting for service {name}...")
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f"Service {name} not available")
            return False

        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

        if not future.done():
            self.get_logger().error(f"{name} call timed out")
            return False

        resp = future.result()
        self.get_logger().info(f"{name}: success={resp.success} msg='{resp.message}'")
        time.sleep(0.3)  # controller settle time
        return bool(resp.success)

    def stop_base(self):
        msg = Twist()
        for _ in range(6):
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.05)
        time.sleep(0.3)

    def turn_left_90(self, angular_speed=0.35):
        """
        Time-based 90 degree turn.
        """
        angle = math.pi / 2.0
        duration = angle / angular_speed

        twist = Twist()
        twist.angular.z = angular_speed

        self.get_logger().info("Turning left 90 degrees...")
        start = time.time()
        while time.time() - start < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)

        self.stop_base()
        self.get_logger().info("Turn complete")

    # -------------------------
    # Arm trajectory
    # -------------------------
    def send_arm_pose(self, joint_names, positions):
        self.get_logger().info("Waiting for trajectory action server...")
        if not self.traj_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Trajectory action server not available")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)

        # ---- KEY: start slightly in the future ----
        start_time = self.get_clock().now() + rclpy.duration.Duration(seconds=1.0)
        goal.trajectory.header.stamp = start_time.to_msg()

        pt1 = JointTrajectoryPoint()
        pt1.positions = list(positions)
        pt1.time_from_start = duration_msg(0.5)

        pt2 = JointTrajectoryPoint()
        pt2.positions = list(positions)
        pt2.time_from_start = duration_msg(2.0)

        goal.trajectory.points = [pt1, pt2]

        self.get_logger().info("Sending arm trajectory...")
        send_future = self.traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)

        if not send_future.done():
            self.get_logger().error("send_goal timed out")
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)

        result = result_future.result().result
        self.get_logger().info(
            f"Arm result: error_code={result.error_code} error_string='{getattr(result, 'error_string', '')}'"
        )

        return int(result.error_code) == 0


def main():
    rclpy.init()
    node = StretchTurnAndPose()

    # 1) Switch to navigation mode
    if not node.call_trigger(node.srv_nav, "/switch_to_navigation_mode"):
        node.get_logger().error("Failed to enter navigation mode")
        goto_shutdown(node)

    # 2) Turn left
    node.turn_left_90()

    # 3) Switch to trajectory mode
    if not node.call_trigger(node.srv_traj, "/switch_to_trajectory_mode"):
        node.get_logger().error("Failed to enter trajectory mode")
        goto_shutdown(node)

    # 4) Arm pose
    node.send_arm_pose(
        joint_names=["joint_lift", "wrist_extension", "joint_wrist_yaw"],
        positions = [0.906026669779699, -8.377152024940286e-06, 0.0051132692929521375],
    )

    node.get_logger().info("Done.")
    node.destroy_node()
    rclpy.shutdown()


def goto_shutdown(node):
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(1)


if __name__ == "__main__":
    main()