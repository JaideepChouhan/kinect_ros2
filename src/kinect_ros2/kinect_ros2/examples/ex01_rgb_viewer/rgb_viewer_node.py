#!/usr/bin/env python3
"""
Example 01: Live RGB Frame Viewer
==================================
Capture RGB frames from the Kinect and publish them on two topics:
  - /kinect/rgb/image_raw       (clean frame)
  - /kinect/rgb/image_annotated (with FPS + timestamp text overlay)

Run with:
    ros2 run kinect_ros2 ex01_rgb_viewer
"""

import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge

from kinect_ros2.kinect_device import KinectDevice, KinectDeviceError


class RGBViewerNode(Node):
    """
    Publishes raw and annotated RGB frames from the Kinect.
    """

    def __init__(self):
        super().__init__('rgb_viewer_node')

        # ── Parameters ──────────────────────────────────────
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('device_index',  0)
        self.declare_parameter('frame_id',  'kinect_rgb_frame')
        self.declare_parameter('font_scale',    0.7)

        rate     = self.get_parameter('publish_rate').value
        self._idx = self.get_parameter('device_index').value
        self._fid = self.get_parameter('frame_id').value
        self._fscale = self.get_parameter('font_scale').value

        # ── State ────────────────────────────────────────────
        self._bridge    = CvBridge()
        self._device    = None
        self._t_last    = time.monotonic()
        self._fps       = 0.0
        self._frame_no  = 0
        self._alpha     = 0.1   # EMA weight for FPS smoothing

        # ── Publishers ───────────────────────────────────────
        self._pub_raw  = self.create_publisher(Image,   '/kinect/rgb/image_raw',       10)
        self._pub_ann  = self.create_publisher(Image,   '/kinect/rgb/image_annotated', 10)
        self._pub_fps  = self.create_publisher(Float32, '/kinect/rgb/fps',              10)

        # ── Open device ──────────────────────────────────────
        try:
            self._device = KinectDevice(self._idx, led_color='green')
            self._device.__enter__()
        except KinectDeviceError as e:
            self.get_logger().fatal(str(e))
            raise SystemExit(1)

        # ── Timer ────────────────────────────────────────────
        self.create_timer(1.0 / rate, self._callback)
        self.get_logger().info(
            f'RGBViewerNode ready | rate={rate:.0f} Hz | '
            f'device={self._idx}')

    # ────────────────────────────────────────────────────────
    def _callback(self):
        try:
            rgb, _ = self._device.get_video()
        except Exception as e:
            self.get_logger().error(f'get_video() failed: {e}')
            return

        now   = time.monotonic()
        dt    = now - self._t_last
        self._t_last = now
        self._frame_no += 1

        # Exponential moving average FPS
        inst_fps  = 1.0 / dt if dt > 0 else 0.0
        self._fps = (1 - self._alpha) * self._fps + self._alpha * inst_fps

        stamp = self.get_clock().now().to_msg()

        # ── Publish raw frame ────────────────────────────────
        msg_raw              = self._bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
        msg_raw.header.stamp    = stamp
        msg_raw.header.frame_id = self._fid
        self._pub_raw.publish(msg_raw)

        # ── Annotate frame ───────────────────────────────────
        annotated = self._annotate(rgb.copy(), stamp)
        msg_ann              = self._bridge.cv2_to_imgmsg(annotated, encoding='rgb8')
        msg_ann.header.stamp    = stamp
        msg_ann.header.frame_id = self._fid
        self._pub_ann.publish(msg_ann)

        # ── Publish FPS ──────────────────────────────────────
        fps_msg      = Float32()
        fps_msg.data = float(self._fps)
        self._pub_fps.publish(fps_msg)

    # ────────────────────────────────────────────────────────
    def _annotate(self, img: np.ndarray, stamp) -> np.ndarray:
        """Draw a semi-transparent HUD on the image."""
        h, w = img.shape[:2]

        # Convert RGB -> BGR for OpenCV drawing, then back
        bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        # Translucent dark bar at top
        overlay = bgr.copy()
        cv2.rectangle(overlay, (0, 0), (w, 36), (20, 30, 50), -1)
        cv2.addWeighted(overlay, 0.55, bgr, 0.45, 0, bgr)

        font  = cv2.FONT_HERSHEY_SIMPLEX
        fs    = self._fscale
        color = (0, 220, 200)   # teal in BGR

        cv2.putText(bgr,
            f'FPS: {self._fps:.1f}',
            (8, 24), font, fs, color, 1, cv2.LINE_AA)

        cv2.putText(bgr,
            f'Frame: {self._frame_no}  |  '
            f'{stamp.sec}.{stamp.nanosec // 1_000_000:03d}',
            (160, 24), font, fs * 0.75, (180, 180, 180), 1, cv2.LINE_AA)

        # Corner watermark
        cv2.putText(bgr, 'kinect_ros2 v2',
            (w - 170, h - 8), font, 0.45, (100, 100, 100), 1, cv2.LINE_AA)

        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    # ────────────────────────────────────────────────────────
    def destroy_node(self):
        if self._device:
            self._device.__exit__(None, None, None)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = RGBViewerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
