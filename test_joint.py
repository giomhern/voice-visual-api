#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class StretchJointMover(Node):
    def __init__(self):
        super().__init__('stretch_joint_mover')
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/stretch_controller/follow_joint_trajectory'
        )

    def send_goal(self, joint_names, positions, duration_sec=2):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=duration_sec)

        goal_msg.trajectory.points = [point]

        self.get_logger().info(f'Sending goal: {joint_names} -> {positions}')
        future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback: {feedback_msg.feedback}')

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: error_code={result.error_code}')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = StretchJointMover()

    node.send_goal(
        joint_names=['joint_lift', 'wrist_extension', 'joint_wrist_yaw'],
        positions=[0.5, 0.1, 0.0],
        duration_sec=2
    )

    rclpy.spin(node)


if __name__ == '__main__':
    main()