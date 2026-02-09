#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_srvs.srv import Trigger

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math


def yaw_from_odom(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class DeskDemoRunner(Node):
    def __init__(self):
        super().__init__("desk_demo_runner")

        self.odom = None
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.cmd_pub = self.create_publisher(Twist, "/stretch/cmd_vel", 10)

        self.traj = ActionClient(self, FollowJointTrajectory, "/stretch_controller/follow_joint_trajectory")
        self.clean_srv = self.create_client(Trigger, "/clean_surface/trigger_clean_surface")

    def _on_odom(self, msg):
        self.odom = msg

    def rotate_left_90(self):
        while self.odom is None:
            time.sleep(0.05)

        start_yaw = yaw_from_odom(self.odom)
        target = start_yaw + math.pi / 2.0

        def wrap(a):
            while a > math.pi:
                a -= 2 * math.pi
            while a < -math.pi:
                a += 2 * math.pi
            return a

        target = wrap(target)
        t = Twist()
        t.angular.z = 0.4

        t0 = time.time()
        while time.time() - t0 < 20.0:
            yaw = yaw_from_odom(self.odom)
            err = wrap(target - yaw)
            if abs(err) < 0.03:
                break
            self.cmd_pub.publish(t)
            time.sleep(0.02)

        self.cmd_pub.publish(Twist())

    def send_preprocess_pose(self):
        self.traj.wait_for_server()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["joint_lift", "wrist_extension", "joint_wrist_yaw"]

        pt = JointTrajectoryPoint()
        pt.positions = [0.9180733020918228, 0.34708226646402623, 0.006391586616190172]
        pt.time_from_start = Duration(sec=2)
        goal.trajectory.points = [pt]

        # stamp "now" (safe)
        goal.trajectory.header.stamp = self.get_clock().now().to_msg()

        send_fut = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=30.0)
        gh = send_fut.result()
        if not gh.accepted:
            raise RuntimeError("Preprocess pose rejected")

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=30.0)
        res = res_fut.result().result
        if int(res.error_code) != 0:
            raise RuntimeError(f"Preprocess pose failed error_code={res.error_code}")

    def trigger_clean_surface(self):
        if not self.clean_srv.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("clean_surface service not available")

        fut = self.clean_srv.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=120.0)
        resp = fut.result()
        if not resp.success:
            raise RuntimeError(f"clean_surface failed: {resp.message}")


def main():
    rclpy.init()
    node = DeskDemoRunner()
    try:
        node.get_logger().info("Rotate left 90...")
        node.rotate_left_90()

        node.get_logger().info("Preprocess arm pose...")
        node.send_preprocess_pose()

        node.get_logger().info("Trigger clean surface...")
        node.trigger_clean_surface()

        node.get_logger().info("DONE ✅")
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()