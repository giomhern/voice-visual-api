#!/usr/bin/env python3
from __future__ import annotations

import math
import os
import time
from typing import Dict, Optional, Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist

from tf2_ros import Buffer, TransformListener

import stretch_funmap.manipulation_planning as mp


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    # yaw (z-axis rotation) from quaternion
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class CleanSurfacePatched(Node):
    """
    Patched clean_surface node:
      - Computes wiping plan via stretch_funmap ManipulationView (expects funmap stack already running if needed).
      - Executes:
          * head positioning (via joint trajectory)
          * base translate/rotate (via cmd_vel + odom)
          * lift/wrist motions (via FollowJointTrajectory)
      - CRITICAL FIX:
          * Trajectory header stamp is set to "now" (not future), preventing:
            "trajectory mode does not currently allow execution of goal with start time in the future"
    """

    def __init__(self):
        super().__init__("clean_surface_patched")
        self.cb = ReentrantCallbackGroup()

        # ---- Parameters
        self.declare_parameter("debug_directory", "")
        self.debug_directory: str = str(self.get_parameter("debug_directory").value or "")
        if self.debug_directory and not self.debug_directory.endswith("/"):
            self.debug_directory += "/"

        # ---- State
        self.joint_states: Optional[JointState] = None
        self.odom: Optional[Odometry] = None
        self.point_cloud: Optional[PointCloud2] = None

        # ---- TF
        self.tf2_buffer = Buffer()
        self.tf2_listener = TransformListener(self.tf2_buffer, self)

        # ---- ROS I/O
        self.joint_sub = self.create_subscription(
            JointState, "/stretch/joint_states", self._on_joint_states, 10, callback_group=self.cb
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self._on_odom, 10, callback_group=self.cb
        )
        self.pc_sub = self.create_subscription(
            PointCloud2, "/camera/depth/color/points", self._on_point_cloud, 10, callback_group=self.cb
        )

        self.cmd_vel_pub = self.create_publisher(Twist, "/stretch/cmd_vel", 10)

        self.traj = ActionClient(
            self, FollowJointTrajectory, "/stretch_controller/follow_joint_trajectory", callback_group=self.cb
        )

        self.srv = self.create_service(
            Trigger, "/clean_surface/trigger_clean_surface", self._on_trigger, callback_group=self.cb
        )

        self.get_logger().info(
            "[clean_surface_patched] Ready. Service: /clean_surface/trigger_clean_surface\n"
            "  - Subscribing: /stretch/joint_states, /odom, /camera/depth/color/points\n"
            "  - Publishing : /stretch/cmd_vel\n"
            "  - Action     : /stretch_controller/follow_joint_trajectory\n"
        )

    # -------------------------
    # Callbacks / state getters
    # -------------------------
    def _on_joint_states(self, msg: JointState) -> None:
        self.joint_states = msg

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg

    def _on_point_cloud(self, msg: PointCloud2) -> None:
        self.point_cloud = msg

    def _get_joint_pos(self, name: str) -> Optional[float]:
        js = self.joint_states
        if js is None:
            return None
        try:
            idx = js.name.index(name)
        except ValueError:
            return None
        if idx >= len(js.position):
            return None
        return float(js.position[idx])

    def _get_base_yaw(self) -> Optional[float]:
        if self.odom is None:
            return None
        q = self.odom.pose.pose.orientation
        return _yaw_from_quat(q.x, q.y, q.z, q.w)

    # -------------------------
    # Motion primitives
    # -------------------------
    def _stop_base(self) -> None:
        self.cmd_vel_pub.publish(Twist())

    def _rotate_base(self, delta_yaw_rad: float, ang_speed: float = 0.4, timeout_s: float = 20.0) -> bool:
        """
        Rotate in place by delta_yaw_rad using odom yaw integration.
        """
        start = time.time()
        yaw0 = self._get_base_yaw()
        if yaw0 is None:
            self.get_logger().error("[base] No odom yet; cannot rotate.")
            return False

        target = yaw0 + delta_yaw_rad

        def wrap(a: float) -> float:
            while a > math.pi:
                a -= 2 * math.pi
            while a < -math.pi:
                a += 2 * math.pi
            return a

        target = wrap(target)
        direction = 1.0 if delta_yaw_rad >= 0 else -1.0

        twist = Twist()
        twist.angular.z = direction * abs(ang_speed)

        while time.time() - start < timeout_s:
            yaw = self._get_base_yaw()
            if yaw is None:
                continue
            err = wrap(target - yaw)
            if abs(err) < 0.03:  # ~1.7 degrees
                break
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.02)

        self._stop_base()
        return True

    def _translate_base(self, delta_m: float, lin_speed: float = 0.10, timeout_s: float = 30.0) -> bool:
        """
        Translate forward/back by delta_m using odom position projection on current heading.
        """
        if self.odom is None:
            self.get_logger().error("[base] No odom yet; cannot translate.")
            return False

        start = time.time()
        x0 = float(self.odom.pose.pose.position.x)
        y0 = float(self.odom.pose.pose.position.y)
        yaw0 = self._get_base_yaw()
        if yaw0 is None:
            self.get_logger().error("[base] No yaw yet; cannot translate.")
            return False

        # Unit heading
        hx = math.cos(yaw0)
        hy = math.sin(yaw0)

        direction = 1.0 if delta_m >= 0 else -1.0

        twist = Twist()
        twist.linear.x = direction * abs(lin_speed)

        while time.time() - start < timeout_s:
            if self.odom is None:
                continue
            x = float(self.odom.pose.pose.position.x)
            y = float(self.odom.pose.pose.position.y)
            dx = x - x0
            dy = y - y0
            moved = dx * hx + dy * hy  # projection onto heading
            if abs(delta_m - moved) < 0.015:
                break
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.02)

        self._stop_base()
        return True

    def _send_joints(
        self,
        joint_names: List[str],
        positions: List[float],
        duration_s: float = 2.0,
        server_wait_s: float = 10.0,
        result_wait_s: float = 30.0,
    ) -> bool:
        if len(joint_names) != len(positions):
            self.get_logger().error("[traj] joint_names/positions mismatch")
            return False

        if not self.traj.wait_for_server(timeout_sec=float(server_wait_s)):
            self.get_logger().error("[traj] Action server not available.")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)

        # IMPORTANT FIX:
        # Set trajectory start stamp to *now* (no future offsets).
        goal.trajectory.header.stamp = self.get_clock().now().to_msg()

        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        sec = int(duration_s)
        nsec = int((duration_s - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nsec)
        goal.trajectory.points = [pt]

        self.get_logger().info(f"[traj] {joint_names} -> {positions}  (t={duration_s:.2f}s)")
        send_fut = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=float(result_wait_s))
        if not send_fut.done():
            self.get_logger().error("[traj] send_goal timed out")
            return False

        gh = send_fut.result()
        if not gh.accepted:
            self.get_logger().error("[traj] goal rejected")
            return False

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=float(result_wait_s))
        if not res_fut.done():
            self.get_logger().error("[traj] result timed out")
            return False

        res = res_fut.result().result
        ok = int(res.error_code) == 0
        self.get_logger().info(f"[traj] result error_code={res.error_code} ok={ok}")
        return ok

    def move_to_pose(self, pose: Dict[str, float], duration_s: float = 2.0) -> bool:
        """
        Minimal pose interface (similar spirit to HelloNode):
          - joint_lift, wrist_extension, joint_wrist_yaw, joint_head_pan, joint_head_tilt, etc. -> trajectory action
          - translate_mobile_base (meters) -> cmd_vel + odom
          - rotate_mobile_base (radians) -> cmd_vel + odom
        """
        if not pose:
            return True

        # Base commands (do these first if present)
        if "rotate_mobile_base" in pose:
            if not self._rotate_base(float(pose["rotate_mobile_base"])):
                return False

        if "translate_mobile_base" in pose:
            if not self._translate_base(float(pose["translate_mobile_base"])):
                return False

        # Joint commands
        joint_names = []
        positions = []
        for k, v in pose.items():
            if k in ("rotate_mobile_base", "translate_mobile_base"):
                continue
            joint_names.append(k)
            positions.append(float(v))

        if joint_names:
            return self._send_joints(joint_names, positions, duration_s=duration_s)

        return True

    # -------------------------
    # Perception / plan
    # -------------------------
    def _look_at_surface(self) -> None:
        if self.point_cloud is None:
            self.get_logger().warn("[look] No point cloud received yet.")
            return

        manip = mp.ManipulationView(self.tf2_buffer, self.debug_directory if self.debug_directory else None)

        # ManipulationView expects a callable like move_to_pose
        manip.move_head(self.move_to_pose)

        # It also expects a PointCloud2 + TF buffer
        manip.update(self.point_cloud, self.tf2_buffer)

        if self.debug_directory:
            dirname = os.path.join(self.debug_directory, "clean_surface")
            os.makedirs(dirname, exist_ok=True)
            filename = f"look_at_surface_{int(time.time())}"
            manip.save_scan(os.path.join(dirname, filename))

        self.manipulation_view = manip

    # -------------------------
    # Service handler
    # -------------------------
    def _on_trigger(self, request, response):
        self.get_logger().info("[clean] Triggered clean surface (patched)")

        # Wait briefly for core streams if needed
        t0 = time.time()
        while (self.joint_states is None or self.odom is None or self.point_cloud is None) and (time.time() - t0 < 5.0):
            time.sleep(0.05)

        if self.joint_states is None or self.odom is None or self.point_cloud is None:
            response.success = False
            response.message = "Missing required streams (joint_states/odom/point_cloud)."
            return response

        # Parameters copied from your original
        tool_width_m = 0.08
        tool_length_m = 0.08
        step_size_m = 0.04
        min_extension_m = 0.01
        max_extension_m = 0.5

        # Build plan
        self._look_at_surface()
        if not hasattr(self, "manipulation_view") or self.manipulation_view is None:
            response.success = False
            response.message = "ManipulationView not ready."
            return response

        strokes, simple_plan, lift_to_surface_m = self.manipulation_view.get_surface_wiping_plan(
            self.tf2_buffer, tool_width_m, tool_length_m, step_size_m
        )
        self.get_logger().info(f"[clean] lift_to_surface_m={lift_to_surface_m}")
        self.get_logger().info(f"[clean] plan_len={0 if simple_plan is None else len(simple_plan)}")

        if not simple_plan:
            response.success = False
            response.message = "No wiping plan generated."
            return response

        # Current positions
        lift_pos = self._get_joint_pos("joint_lift")
        wrist_pos = self._get_joint_pos("wrist_extension")
        if lift_pos is None or wrist_pos is None:
            response.success = False
            response.message = "Missing joint positions for joint_lift/wrist_extension."
            return response

        above_surface_m = 0.10
        lift_above_surface_m = lift_pos + float(lift_to_surface_m) + above_surface_m

        # Raise tool above surface
        if not self.move_to_pose({"joint_lift": lift_above_surface_m}, duration_s=2.0):
            response.success = False
            response.message = "Failed raising lift."
            return response

        initial_wrist = wrist_pos

        # Execute plan rows
        for i, m in enumerate(simple_plan):
            forward_m = float(m["mobile_base_forward_m"])
            if not self.move_to_pose({"translate_mobile_base": forward_m}, duration_s=0.0):
                response.success = False
                response.message = f"Failed base translate at step {i}."
                return response

            # Extend to start
            start_extension_m = max(initial_wrist + float(m["start_wrist_extension_m"]), min_extension_m)
            if not self.move_to_pose({"wrist_extension": start_extension_m}, duration_s=1.5):
                response.success = False
                response.message = f"Failed start extension at step {i}."
                return response

            # Extend to end
            end_extension_m = min(initial_wrist + float(m["end_wrist_extension_m"]), max_extension_m)
            if not self.move_to_pose({"wrist_extension": end_extension_m}, duration_s=1.5):
                response.success = False
                response.message = f"Failed end extension at step {i}."
                return response

            # Retract back to start
            if not self.move_to_pose({"wrist_extension": start_extension_m}, duration_s=1.5):
                response.success = False
                response.message = f"Failed retract at step {i}."
                return response

        # Wrap up: raise + retract
        self.move_to_pose({"joint_lift": lift_above_surface_m}, duration_s=2.0)
        self.move_to_pose({"wrist_extension": initial_wrist}, duration_s=2.0)

        response.success = True
        response.message = "Completed surface cleaning (patched)!"
        return response


def main():
    rclpy.init()
    node = CleanSurfacePatched()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_base()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()