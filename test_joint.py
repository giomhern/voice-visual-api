#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class CleanSurfaceFlowTest(Node):
    def __init__(self):
        super().__init__("clean_surface_flow_test")
        self.client = self.create_client(
            Trigger,
            "/clean_surface/trigger_clean_surface",
        )

    def run(self):
        self.get_logger().info("Waiting for /clean_surface/trigger_clean_surface...")
        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Service not available")
            return 1

        self.get_logger().info("Calling clean_surface trigger...")
        future = self.client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=120.0)

        if not future.done():
            self.get_logger().error("Service call timed out")
            return 1

        resp = future.result()
        if resp.success:
            self.get_logger().info(f"SUCCESS ✅  message='{resp.message}'")
            return 0
        else:
            self.get_logger().error(f"FAILED ❌  message='{resp.message}'")
            return 1


def main():
    rclpy.init()
    node = CleanSurfaceFlowTest()
    try:
        rc = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    raise SystemExit(rc)


if __name__ == "__main__":
    main()