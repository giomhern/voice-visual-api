#!/usr/bin/env python3
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class AmbientMovement(Node):
    """
    Subtle idle movement for Stretch: rotate base a few degrees left/right to feel "lively".

    Publishes Twist to /stretch/cmd_vel (angular.z only).
    Non-blocking state machine (no sleeps inside timer callback).
    """

    def __init__(
        self,
        cmd_vel_topic: str = "/stretch/cmd_vel",
        max_angle_deg: float = 8.0,     # peak yaw offset in degrees
        angular_speed: float = 0.12,    # rad/s (slow + safe)
        pause_s: float = 1.5,           # pause at each extreme
        update_hz: float = 20.0,        # timer frequency
    ):
        super().__init__("ambient_base_swivel")

        self.pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.dt = 1.0 / float(update_hz)
        self.max_angle = math.radians(float(max_angle_deg))
        self.w = float(angular_speed)
        self.pause_s = float(pause_s)

        # state
        self.offset = 0.0          # integrated yaw offset (approx)
        self.direction = 1.0       # +1 left, -1 right
        self.mode = "TURN"         # "TURN" or "PAUSE"
        self.pause_left = 0.0

        self.timer = self.create_timer(self.dt, self._tick)

        self.get_logger().info(
            f"Ambient swivel running: topic={cmd_vel_topic}, "
            f"max_angle_deg={max_angle_deg}, angular_speed={angular_speed}, pause_s={pause_s}"
        )

    def _publish_stop(self):
        self.pub.publish(Twist())

    def _tick(self):
        # State machine: TURN -> PAUSE -> TURN ...
        if self.mode == "PAUSE":
            self.pause_left -= self.dt
            self._publish_stop()
            if self.pause_left <= 0.0:
                self.mode = "TURN"
            return

        # TURN mode
        # If at (or beyond) bounds, switch direction and pause.
        if abs(self.offset) >= self.max_angle:
            self.direction *= -1.0
            self.mode = "PAUSE"
            self.pause_left = self.pause_s
            self._publish_stop()
            return

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.direction * self.w
        self.pub.publish(twist)

        # integrate offset approximately
        self.offset += twist.angular.z * self.dt

    def stop(self):
        self._publish_stop()
        self.get_logger().info("Ambient swivel stopped.")


def main():
    rclpy.init()
    node = AmbientMovement()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()