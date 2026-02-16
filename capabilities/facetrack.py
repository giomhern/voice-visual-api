#!/usr/bin/env python3
import math
import time
from typing import Optional, Tuple, Any, Dict, List

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import MarkerArray, Marker
from builtin_interfaces.msg import Duration

from cv_bridge import CvBridge

import head_estimator as he
import deep_learning_model_options as do


def _get_bbox(head: Any) -> Optional[Tuple[float, float, float, float]]:
    if head is None:
        return None

    if isinstance(head, dict):
        for k in ["bbox", "box", "face_bbox", "rect"]:
            if k in head and head[k] is not None:
                b = head[k]
                if isinstance(b, dict):
                    xs = [b.get("x_min"), b.get("xmin"), b.get("left")]
                    ys = [b.get("y_min"), b.get("ymin"), b.get("top")]
                    xe = [b.get("x_max"), b.get("xmax"), b.get("right")]
                    ye = [b.get("y_max"), b.get("ymax"), b.get("bottom")]
                    if all(v is not None for v in [xs[0] or xs[1] or xs[2], ys[0] or ys[1] or ys[2], xe[0] or xe[1] or xe[2], ye[0] or ye[1] or ye[2]]):
                        x1 = float(xs[0] or xs[1] or xs[2])
                        y1 = float(ys[0] or ys[1] or ys[2])
                        x2 = float(xe[0] or xe[1] or xe[2])
                        y2 = float(ye[0] or ye[1] or ye[2])
                        return (x1, y1, x2, y2)
                elif isinstance(b, (list, tuple)) and len(b) >= 4:
                    x1, y1, x2, y2 = b[:4]
                    return (float(x1), float(y1), float(x2), float(y2))
        for k in ["x1y1x2y2", "xyxy"]:
            if k in head and head[k] is not None:
                b = head[k]
                if isinstance(b, (list, tuple)) and len(b) >= 4:
                    x1, y1, x2, y2 = b[:4]
                    return (float(x1), float(y1), float(x2), float(y2))

    for attr in ["bbox", "box", "rect"]:
        if hasattr(head, attr):
            b = getattr(head, attr)
            if isinstance(b, dict):
                keys = list(b.keys())
                if set(keys) >= {"x_min", "y_min", "x_max", "y_max"}:
                    return (float(b["x_min"]), float(b["y_min"]), float(b["x_max"]), float(b["y_max"]))
            if isinstance(b, (list, tuple)) and len(b) >= 4:
                x1, y1, x2, y2 = b[:4]
                return (float(x1), float(y1), float(x2), float(y2))

    for attrset in [
        ("x_min", "y_min", "x_max", "y_max"),
        ("xmin", "ymin", "xmax", "ymax"),
        ("left", "top", "right", "bottom"),
    ]:
        if all(hasattr(head, a) for a in attrset):
            x1 = float(getattr(head, attrset[0]))
            y1 = float(getattr(head, attrset[1]))
            x2 = float(getattr(head, attrset[2]))
            y2 = float(getattr(head, attrset[3]))
            return (x1, y1, x2, y2)

    return None


class DeepPerceptionSubscriber(Node):
    def __init__(self):
        super().__init__("deep_perception_subscriber")

        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")

        self.declare_parameter("marker_topic", "/faces/marker_array")
        self.declare_parameter("debug_image_topic", "/faces/color/image_with_bb")

        self.declare_parameter("frame_id", "camera_color_optical_frame")
        self.declare_parameter("publish_debug_image", True)

        self.declare_parameter("min_depth_m", 0.25)
        self.declare_parameter("max_depth_m", 3.5)
        self.declare_parameter("depth_window", 7)

        self.declare_parameter("lifetime_s", 0.25)
        self.declare_parameter("cube_thickness_m", 0.01)

        self.bridge = CvBridge()

        models_directory = do.get_directory()
        use_neural_compute_stick = do.use_neural_compute_stick()
        self.estimator = he.HeadPoseEstimator(
            models_directory, use_neural_compute_stick=use_neural_compute_stick
        )

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.last_depth = None
        self.last_depth_encoding = None

        self.sub_info = self.create_subscription(
            CameraInfo, self.get_parameter("camera_info_topic").value, self.on_info, 10
        )
        self.sub_depth = self.create_subscription(
            Image, self.get_parameter("depth_topic").value, self.on_depth, 10
        )
        self.sub_color = self.create_subscription(
            Image, self.get_parameter("color_topic").value, self.on_color, 10
        )

        self.pub_markers = self.create_publisher(
            MarkerArray, self.get_parameter("marker_topic").value, 10
        )
        self.pub_debug = self.create_publisher(
            Image, self.get_parameter("debug_image_topic").value, 10
        )

        self.next_id = 1

    def on_info(self, msg: CameraInfo):
        k = msg.k
        self.fx = float(k[0])
        self.fy = float(k[4])
        self.cx = float(k[2])
        self.cy = float(k[5])

    def on_depth(self, msg: Image):
        try:
            d = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception:
            return
        self.last_depth = d
        self.last_depth_encoding = msg.encoding

    def depth_at(self, u: int, v: int) -> float:
        if self.last_depth is None:
            return float("nan")

        dimg = self.last_depth
        if len(dimg.shape) != 2:
            return float("nan")

        h, w = dimg.shape[:2]
        if u < 0 or v < 0 or u >= w or v >= h:
            return float("nan")

        win = int(self.get_parameter("depth_window").value)
        r = max(1, win // 2)
        u0 = max(0, u - r)
        u1 = min(w - 1, u + r)
        v0 = max(0, v - r)
        v1 = min(h - 1, v + r)

        patch = dimg[v0 : v1 + 1, u0 : u1 + 1].astype(np.float32)

        enc = (self.last_depth_encoding or "").upper()
        if "16U" in enc:
            patch = patch / 1000.0

        vals = patch.reshape(-1)
        vals = vals[np.isfinite(vals)]
        vals = vals[vals > 0.0]
        if vals.size == 0:
            return float("nan")

        return float(np.median(vals))

    def xyz_from_uvz(self, u: float, v: float, z: float) -> Tuple[float, float, float]:
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        return (float(x), float(y), float(z))

    def publish_empty(self, stamp):
        ma = MarkerArray()
        self.pub_markers.publish(ma)

    def on_color(self, msg: Image):
        if self.fx is None or self.last_depth is None:
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return

        heads, out_img = self.estimator.apply_to_image(bgr, draw_output=True)

        frame_id = self.get_parameter("frame_id").value
        min_z = float(self.get_parameter("min_depth_m").value)
        max_z = float(self.get_parameter("max_depth_m").value)

        ma = MarkerArray()

        if heads is None:
            self.pub_markers.publish(ma)
            if bool(self.get_parameter("publish_debug_image").value):
                self.pub_debug.publish(self.bridge.cv2_to_imgmsg(out_img, encoding="bgr8"))
            return

        if not isinstance(heads, (list, tuple)):
            heads = [heads]

        best = None
        best_z = None
        best_bbox = None

        for h in heads:
            bbox = _get_bbox(h)
            if bbox is None:
                continue
            x1, y1, x2, y2 = bbox
            u = int(round((x1 + x2) / 2.0))
            v = int(round((y1 + y2) / 2.0))
            z = self.depth_at(u, v)
            if not math.isfinite(z):
                continue
            if z < min_z or z > max_z:
                continue
            if best is None or z < best_z:
                best = (u, v, z)
                best_z = z
                best_bbox = bbox

        if best is None:
            self.pub_markers.publish(ma)
            if bool(self.get_parameter("publish_debug_image").value):
                self.pub_debug.publish(self.bridge.cv2_to_imgmsg(out_img, encoding="bgr8"))
            return

        u, v, z = best
        x, y, z = self.xyz_from_uvz(float(u), float(v), float(z))

        m = Marker()
        m.header.stamp = msg.header.stamp
        m.header.frame_id = frame_id
        m.ns = ""
        m.id = int(self.next_id)
        self.next_id = (self.next_id + 1) % 1000000
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.pose.orientation.w = 1.0
        if best_bbox is not None:
            x1, y1, x2, y2 = best_bbox
            wpx = max(2.0, float(x2 - x1))
            hpx = max(2.0, float(y2 - y1))
            sx = wpx * z / self.fx
            sy = hpx * z / self.fy
            m.scale.x = float(sx)
            m.scale.y = float(sy)
        else:
            m.scale.x = 0.18
            m.scale.y = 0.18
        m.scale.z = float(self.get_parameter("cube_thickness_m").value)
        m.color.r = 0.73
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 0.5
        lt = float(self.get_parameter("lifetime_s").value)
        sec = int(lt)
        nsec = int((lt - sec) * 1e9)
        m.lifetime = Duration(sec=sec, nanosec=nsec)
        m.text = "face"

        ma.markers.append(m)
        self.pub_markers.publish(ma)

        if bool(self.get_parameter("publish_debug_image").value):
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(out_img, encoding="bgr8"))


def main():
    rclpy.init()
    rclpy.spin(DeepPerceptionSubscriber())
    rclpy.shutdown()


if __name__ == "__main__":
    main()