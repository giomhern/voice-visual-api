#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import Image, JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from cv_bridge import CvBridge
import cv2

import mediapipe as mp


# -----------------------
# USER CONFIG (edit these)
# -----------------------

IMAGE_TOPIC = "/camera/color/image_raw"   # <-- replace with yours
JOINT_STATES_TOPIC = "/joint_states"

# Replace with output of: ros2 action list | grep FollowJointTrajectory
TRAJ_ACTION_NAME = "/stretch_controller/follow_joint_trajectory"

# How to identify head joints from /joint_states names:
# These are "contains" matches; adjust if needed.
PAN_NAME_HINT = "head_pan"
TILT_NAME_HINT = "head_tilt"

# Joint limits (radians). Conservative defaults.
PAN_MIN, PAN_MAX = -2.7, 2.7
TILT_MIN, TILT_MAX = -1.2, 1.2

# Control tuning
KP_PAN = 1.4     # responsiveness left/right
KP_TILT = 1.2    # responsiveness up/down
MAX_STEP = 0.08  # max radians per update (smoothness)
SEND_HZ = 10.0   # command rate

# If your camera axes feel flipped, change these signs.
PAN_SIGN = +1.0
TILT_SIGN = +1.0


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class FaceHeadFollower(Node):
    def __init__(self):
        super().__init__("face_head_follower")

        self.bridge = CvBridge()

        self.face_detector = mp.solutions.face_detection.FaceDetection(
            model_selection=0,
            min_detection_confidence=0.6,
        )

        self.joint_names = []
        self.joint_positions = {}
        self.pan_joint = None
        self.tilt_joint = None

        self.last_send_time = 0.0

        self.create_subscription(JointState, JOINT_STATES_TOPIC, self.on_joint_state, 10)
        self.create_subscription(Image, IMAGE_TOPIC, self.on_image, 10)

        self.traj_client = ActionClient(self, FollowJointTrajectory, TRAJ_ACTION_NAME)

        self.get_logger().info(f"Listening to image: {IMAGE_TOPIC}")
        self.get_logger().info(f"Using trajectory action: {TRAJ_ACTION_NAME}")
        self.get_logger().info("Waiting for joint_states to learn joint names...")

    def on_joint_state(self, msg: JointState):
        # Update joint positions
        for n, p in zip(msg.name, msg.position):
            self.joint_positions[n] = p
        self.joint_names = list(msg.name)

        # Resolve joint names once
        if self.pan_joint is None or self.tilt_joint is None:
            self.pan_joint = self._find_joint(PAN_NAME_HINT)
            self.tilt_joint = self._find_joint(TILT_NAME_HINT)
            if self.pan_joint and self.tilt_joint:
                self.get_logger().info(f"Resolved head joints: pan={self.pan_joint}, tilt={self.tilt_joint}")
                self.get_logger().info("Waiting for trajectory action server...")
            # Don’t spam logs if not found; user will see nothing until resolved.

    def _find_joint(self, hint: str):
        hint = hint.lower()
        for name in self.joint_names:
            if hint in name.lower():
                return name
        return None

    def on_image(self, msg: Image):
        # Must have joints resolved
        if not (self.pan_joint and self.tilt_joint):
            return

        # Throttle command sending
        now = time.time()
        if now - self.last_send_time < 1.0 / SEND_HZ:
            return

        # Ensure action server ready
        if not self.traj_client.server_is_ready():
            # Try to connect once in a while
            self.traj_client.wait_for_server(timeout_sec=0.1)
            return

        # Convert ROS Image -> OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}")
            return

        h, w = frame.shape[:2]

        # Face detect (MediaPipe expects RGB)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.face_detector.process(rgb)

        if not results.detections:
            return

        # Take the most confident detection
        best = max(results.detections, key=lambda d: d.score[0])
        bbox = best.location_data.relative_bounding_box

        # Face center in normalized coords [0,1]
        cx = bbox.xmin + bbox.width * 0.5
        cy = bbox.ymin + bbox.height * 0.5

        # Error: +ex means face is to the right of center
        ex = (cx - 0.5)
        ey = (cy - 0.5)

        # Convert to desired joint deltas.
        # Typically: move pan in opposite direction of ex to center it.
        # Depending on your camera orientation, you might flip signs.
        d_pan = PAN_SIGN * (-KP_PAN * ex)
        d_tilt = TILT_SIGN * (-KP_TILT * ey)

        # Limit per-step change for smoothness
        d_pan = clamp(d_pan, -MAX_STEP, MAX_STEP)
        d_tilt = clamp(d_tilt, -MAX_STEP, MAX_STEP)

        # Current positions
        pan_cur = self.joint_positions.get(self.pan_joint, 0.0)
        tilt_cur = self.joint_positions.get(self.tilt_joint, 0.0)

        pan_cmd = clamp(pan_cur + d_pan, PAN_MIN, PAN_MAX)
        tilt_cmd = clamp(tilt_cur + d_tilt, TILT_MIN, TILT_MAX)

        self.send_head_goal(pan_cmd, tilt_cmd)
        self.last_send_time = now

    def send_head_goal(self, pan: float, tilt: float):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [self.pan_joint, self.tilt_joint]

        pt = JointTrajectoryPoint()
        pt.positions = [float(pan), float(tilt)]
        pt.time_from_start = Duration(sec=0, nanosec=int(0.35 * 1e9))  # smooth, short motion

        goal.trajectory.points = [pt]

        # Fire-and-forget goal
        self.traj_client.send_goal_async(goal)


def main():
    rclpy.init()
    node = FaceHeadFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()