import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelScaler(Node):
    """Scale incoming cmd_vel and republish. Useful for applying movement_speed to Nav2."""

    def __init__(self):
        super().__init__("cmd_vel_scaler")

        self.declare_parameter("in_topic", "/cmd_vel")
        self.declare_parameter("out_topic", "/stretch/cmd_vel")
        self.declare_parameter("linear_scale", 1.0)
        self.declare_parameter("angular_scale", 1.0)

        in_topic = str(self.get_parameter("in_topic").value)
        out_topic = str(self.get_parameter("out_topic").value)

        self.sub = self.create_subscription(Twist, in_topic, self._on_cmd, 10)
        self.pub = self.create_publisher(Twist, out_topic, 10)

        self.get_logger().info(f"CmdVelScaler scaling {in_topic} -> {out_topic}")

    def _on_cmd(self, msg: Twist):
        lin = float(self.get_parameter("linear_scale").value)
        ang = float(self.get_parameter("angular_scale").value)

        out = Twist()
        out.linear.x = msg.linear.x * lin
        out.linear.y = msg.linear.y * lin
        out.linear.z = msg.linear.z * lin
        out.angular.x = msg.angular.x * ang
        out.angular.y = msg.angular.y * ang
        out.angular.z = msg.angular.z * ang

        self.pub.publish(out)


def main():
    rclpy.init()
    node = CmdVelScaler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
