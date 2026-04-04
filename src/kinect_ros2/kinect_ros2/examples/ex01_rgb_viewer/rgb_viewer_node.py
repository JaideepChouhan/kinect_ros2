#!/usr/bin/env python3
"""
Example 01: Live RGB Frame Viewer (Subscriber version)
========================================================
Subscribes to /camera/rgb/image_raw and publishes annotated version
with FPS and timestamp overlay.

Run with:
    ros2 run kinect_ros2 ex01_rgb_viewer
"""

import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge


class RGBViewerNode(Node):
    def __init__(self):
        super().__init__('rgb_viewer_node')

        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('frame_id', 'kinect_rgb_frame')
        self.declare_parameter('font_scale', 0.7)

        rate = self.get_parameter('publish_rate').value
        self._fid = self.get_parameter('frame_id').value
        self._fscale = self.get_parameter('font_scale').value

        self._bridge = CvBridge()
        self._t_last = time.monotonic()
        self._fps = 0.0
        self._frame_no = 0
        self._alpha = 0.1

        # Publishers
        self._pub_raw = self.create_publisher(Image, '/kinect/rgb/image_raw', 10)
        self._pub_ann = self.create_publisher(Image, '/kinect/rgb/image_annotated', 10)
        self._pub_fps = self.create_publisher(Float32, '/kinect/rgb/fps', 10)

        # QoS matching the camera driver (BEST_EFFORT)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.create_subscription(Image, '/camera/rgb/image_raw', self._rgb_cb, qos)

        self.get_logger().info(f'RGBViewerNode ready (subscriber mode) | rate={rate:.0f} Hz')

    def _rgb_cb(self, msg: Image):
        rgb = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

        now = time.monotonic()
        dt = now - self._t_last
        self._t_last = now
        self._frame_no += 1

        inst_fps = 1.0 / dt if dt > 0 else 0.0
        self._fps = (1 - self._alpha) * self._fps + self._alpha * inst_fps

        stamp = msg.header.stamp

        # Publish raw (passthrough)
        raw_msg = self._bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
        raw_msg.header = msg.header
        raw_msg.header.frame_id = self._fid
        self._pub_raw.publish(raw_msg)

        # Annotate
        annotated = self._annotate(rgb.copy(), stamp)
        ann_msg = self._bridge.cv2_to_imgmsg(annotated, encoding='rgb8')
        ann_msg.header = msg.header
        ann_msg.header.frame_id = self._fid
        self._pub_ann.publish(ann_msg)

        fps_msg = Float32()
        fps_msg.data = float(self._fps)
        self._pub_fps.publish(fps_msg)

    def _annotate(self, img: np.ndarray, stamp) -> np.ndarray:
        h, w = img.shape[:2]
        bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        overlay = bgr.copy()
        cv2.rectangle(overlay, (0, 0), (w, 36), (20, 30, 50), -1)
        cv2.addWeighted(overlay, 0.55, bgr, 0.45, 0, bgr)

        font = cv2.FONT_HERSHEY_SIMPLEX
        fs = self._fscale
        color = (0, 220, 200)

        cv2.putText(bgr, f'FPS: {self._fps:.1f}', (8, 24), font, fs, color, 1, cv2.LINE_AA)
        cv2.putText(bgr, f'Frame: {self._frame_no}  |  {stamp.sec}.{stamp.nanosec // 1_000_000:03d}',
                    (160, 24), font, fs * 0.75, (180, 180, 180), 1, cv2.LINE_AA)
        cv2.putText(bgr, 'kinect_ros2 v2', (w - 170, h - 8), font, 0.45, (100, 100, 100), 1, cv2.LINE_AA)

        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def main(args=None):
    rclpy.init(args=args)
    node = RGBViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()