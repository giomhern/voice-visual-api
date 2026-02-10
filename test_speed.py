#!/usr/bin/env python3
"""
Standalone /stretch/cmd_vel speed test.

Publishes Twist for a fixed duration, then publishes zero to stop.
This tests real base speed without any odom logic or BaseMotion code.

Usage examples:
  ros2 run <your_pkg> cmd_vel_speed_test -- --profile slow
  ros2 run <your_pkg> cmd_vel_speed_test -- --linear 0.20 --seconds 2.0
"""

import argparse
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


PROFILES = {
    "slow":   (0.10, 0.40),  # (linear m/s, angular rad/s)
    "medium": (0.15, 0.60),
    "fast":   (0.22, 0.80),
}


class CmdVelSpeedTest(Node):
    def __init__(self, args):
        super().__init__("cmd_vel_speed_test")
        self.pub = self.create_publisher(Twist, args.topic, 10)
        self.args = args

    def publish_for(self, linear_x: float, angular_z: float, seconds: float, hz: float = 20.0):
        self.get_logger().info(
            f"Publishing cmd_vel: linear.x={linear_x:.3f} m/s angular.z={angular_z:.3f} rad/s "
            f"for {seconds:.2f}s on {self.args.topic}"
        )

        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)

        period = 1.0 / hz
        end_t = time.time() + seconds

        while rclpy.ok() and time.time() < end_t:
            self.pub.publish(msg)
            time.sleep(period)

        # STOP
        self.stop()

    def stop(self):
        self.pub.publish(Twist())
        self.get_logger().info("STOP published (zero Twist).")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/stretch/cmd_vel")
    parser.add_argument("--profile", choices=["slow", "medium", "fast"], default=None)
    parser.add_argument("--linear", type=float, default=None, help="linear.x m/s (overrides profile)")
    parser.add_argument("--angular", type=float, default=0.0, help="angular.z rad/s")
    parser.add_argument("--seconds", type=float, default=2.0)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--sequence", action="store_true",
                        help="Run slow -> medium -> fast automatically (each for --seconds)")
    args = parser.parse_args()

    rclpy.init()
    node = CmdVelSpeedTest(args)

    try:
        # Option A: run sequence slow->medium->fast
        if args.sequence:
            for name in ["slow", "medium", "fast"]:
                lin, ang = PROFILES[name]
                node.get_logger().info(f"=== PROFILE: {name.upper()} ===")
                node.publish_for(lin, args.angular if args.angular != 0.0 else ang, args.seconds, args.hz)
                time.sleep(1.0)  # pause between runs
            node.get_logger().info("Sequence complete.")
            return

        # Option B: single run using profile or explicit linear
        if args.linear is not None:
            linear_x = args.linear
        elif args.profile is not None:
            linear_x = PROFILES[args.profile][0]
        else:
            linear_x = 0.10  # default if nothing specified

        angular_z = args.angular
        if args.profile is not None and args.angular == 0.0:
            # if angular not given, use profile default
            angular_z = PROFILES[args.profile][1] if args.angular == 0.0 else args.angular
            # but default angular is 0.0, so we'll keep 0 unless user wants turns
            angular_z = 0.0

        node.publish_for(linear_x, angular_z, args.seconds, args.hz)

    except KeyboardInterrupt:
        pass
    finally:
        # Always stop for safety
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()