#!/usr/bin/env python3
"""
Straight line cmd_vel test for Stretch.

Publishes a constant forward velocity for N seconds, then stops.
No odom, no nav2, no BaseMotion, no scaling.
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class StraightLineTest(Node):
    def __init__(self):
        super().__init__("straight_line_test")
        self.pub = self.create_publisher(Twist, "/stretch/cmd_vel", 10)

    def run(self, speed_mps: float, duration_s: float, hz: float = 20.0):
        self.get_logger().info(
            f"Driving straight: {speed_mps:.3f} m/s for {duration_s:.2f}s"
        )

        msg = Twist()
        msg.linear.x = speed_mps

        period = 1.0 / hz
        end_time = time.time() + duration_s

        while rclpy.ok() and time.time() < end_time:
            self.pub.publish(msg)
            time.sleep(period)

        # STOP
        self.pub.publish(Twist())
        self.get_logger().info("Stopped.")


def main():
    rclpy.init()
    node = StraightLineTest()

    try:
        # CHANGE THESE NUMBERS ONLY
        speed = 0.10      # meters per second
        duration = 2.0    # seconds

        node.run(speed, duration)

    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()