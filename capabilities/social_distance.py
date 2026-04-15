#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger


DISTANCE_MAP = {
    "close": 0.35,    # meters
    "medium": 0.75,
    "far": 1.25,
}


class SocialDistance(Node):

    def __init__(self):
        super().__init__("social_distance_back")

        self.cmd_vel_topic = "/stretch/cmd_vel"

        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.switch_nav = self.create_client(
            Trigger,
            "/switch_to_navigation_mode"
        )

    def switch_to_nav(self):

        if not self.switch_nav.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Navigation service not available")
            return False

        req = Trigger.Request()

        future = self.switch_nav.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()

        if not result.success:
            self.get_logger().error("Failed to switch to navigation mode")
            return False

        self.get_logger().info("Navigation mode enabled")
        return True

    def drive_back(self, distance, speed=0.10):

        duration = abs(distance) / speed

        msg = Twist()
        msg.linear.x = -speed   # negative = backwards

        start = time.time()

        while time.time() - start < duration:
            self.pub.publish(msg)
            time.sleep(0.05)

        self.stop()

    def stop(self):

        z = Twist()

        for _ in range(5):
            self.pub.publish(z)
            time.sleep(0.05)

    def run(self, distance_label):

        if distance_label not in DISTANCE_MAP:
            self.get_logger().error(f"Invalid distance: {distance_label}")
            return

        distance = DISTANCE_MAP[distance_label]

        self.get_logger().info(f"Backing up: {distance_label} ({distance:.2f} m)")

        if not self.switch_to_nav():
            return

        self.drive_back(distance)

        self.get_logger().info("Done.")


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--distance",
        choices=["close", "medium", "far"],
        default="medium",
        help="Social distance level"
    )

    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)

    node = SocialDistance()

    node.run(args.distance)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()