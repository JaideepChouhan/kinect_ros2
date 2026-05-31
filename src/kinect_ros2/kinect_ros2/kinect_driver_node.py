#!/usr/bin/env python3
# Copyright 2024 ROS2 Community -- Apache-2.0
"""
kinect_driver_node.py  –  Kinect v1 driver with built-in LaserScan publisher.

WHAT CHANGED FROM THE ORIGINAL
────────────────────────────────
  • /scan (sensor_msgs/LaserScan) is now published directly inside this node.
    There is NO need for a separate depth_to_laserscan node.

  • Root cause of LIBUSB_ERROR_BUSY fixed:
    kinect_full.launch.py launches tilt_control inside the same process group,
    which opens its own freenect context on the same USB device, racing with
    the driver.  Solution: remove tilt_control from the full launch (or use
    the standalone kinect.launch.py which never had this problem), and keep
    only ONE process that ever touches the Kinect USB device.

  • rclpy.shutdown() double-call traceback fixed:
    The finally block now guards with `if rclpy.ok()` (already present in
    your code – the tracebacks were caused by the namespace-launch device
    conflict above, not this node itself).

NEW PARAMETERS (all optional – sane defaults already set)
──────────────────────────────────────────────────────────
  publish_scan    bool   true    Set false to disable /scan entirely
  scan_height     int    20      Central depth rows to sample (more = slower)
  scan_range_min  float  0.40   Minimum valid range [m]  (Kinect HW floor)
  scan_range_max  float  3.50   Maximum useful range [m]
  scan_use_min    bool   true    true = nearest obstacle per column (safest)
                                 false = mean depth per column
  scan_topic      str    /scan  Output topic name

PUBLISHED TOPICS
─────────────────
  /camera/rgb/image_raw          sensor_msgs/Image
  /camera/rgb/camera_info        sensor_msgs/CameraInfo
  /camera/depth/image_raw        sensor_msgs/Image   (16UC1 raw counts)
  /camera/depth/image_meters     sensor_msgs/Image   (32FC1, metres)
  /camera/depth/camera_info      sensor_msgs/CameraInfo
  /scan                          sensor_msgs/LaserScan   ← NEW, built-in

QUICKSTART
───────────
  ros2 run kinect_ros2 kinect_driver
  ros2 topic hz /scan          # should show ~30 Hz
  ros2 topic echo /scan --once # check range values
"""

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy, QoSHistoryPolicy,
    QoSProfile, QoSReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Header
from cv_bridge import CvBridge

try:
    import freenect
    _FREENECT_OK = True
except ImportError:
    _FREENECT_OK = False

from kinect_ros2.depth_utils import raw_to_meters, make_camera_info_dict

# ── QoS ──────────────────────────────────────────────────────────────────────
# BEST_EFFORT matches the Kinect's USB stream nature; Nav2 / slam_toolbox
# accept BEST_EFFORT on /scan without issues.
_SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class KinectDriverNode(Node):
    """
    Single-process Kinect v1 driver.

    Grabs RGB + Depth from freenect, publishes all image topics,
    and converts the depth image to a LaserScan in-place.
    One node, one USB context, zero device contention.
    """

    def __init__(self):
        super().__init__('kinect_driver_node')

        if not _FREENECT_OK:
            self.get_logger().fatal(
                'freenect Python module not found.  '
                'Install: pip3 install freenect --break-system-packages'
            )
            raise RuntimeError('freenect unavailable')

        # ── Declare all parameters ────────────────────────────────────────
        # Camera / driver
        self.declare_parameter('device_index',    0)
        self.declare_parameter('target_fps',      30.0)
        self.declare_parameter('rgb_frame_id',    'camera_rgb_optical_frame')
        self.declare_parameter('depth_frame_id',  'camera_depth_optical_frame')
        self.declare_parameter('publish_rgb',     True)
        self.declare_parameter('publish_depth',   True)
        self.declare_parameter('depth_calib_c1',  -0.0030711016)
        self.declare_parameter('depth_calib_c2',   3.3309495161)
        self.declare_parameter('rgb_fx',  525.0)
        self.declare_parameter('rgb_fy',  525.0)
        self.declare_parameter('rgb_cx',  319.5)
        self.declare_parameter('rgb_cy',  239.5)
        self.declare_parameter('depth_fx', 580.0)
        self.declare_parameter('depth_fy', 580.0)
        self.declare_parameter('depth_cx', 319.5)
        self.declare_parameter('depth_cy', 239.5)

        # LaserScan  (new)
        self.declare_parameter('publish_scan',   True)
        self.declare_parameter('scan_height',    20)
        self.declare_parameter('scan_range_min', 0.40)
        self.declare_parameter('scan_range_max', 3.50)
        self.declare_parameter('scan_use_min',   True)
        self.declare_parameter('scan_topic',     '/scan')

        # ── Read parameters ───────────────────────────────────────────────
        p = self.get_parameter
        self._dev        = p('device_index').value
        fps              = p('target_fps').value
        self._rgb_fid    = p('rgb_frame_id').value
        self._dep_fid    = p('depth_frame_id').value
        self._pub_rgb    = p('publish_rgb').value
        self._pub_dep    = p('publish_depth').value
        self._c1         = p('depth_calib_c1').value
        self._c2         = p('depth_calib_c2').value

        self._pub_scan   = p('publish_scan').value
        self._scan_h     = int(p('scan_height').value)
        self._range_min  = float(p('scan_range_min').value)
        self._range_max  = float(p('scan_range_max').value)
        self._use_min    = bool(p('scan_use_min').value)
        scan_topic       = p('scan_topic').value

        # Depth intrinsics used for scan angle calculation
        self._depth_fx   = float(p('depth_fx').value)
        self._depth_cx   = float(p('depth_cx').value)

        # ── Internal state ────────────────────────────────────────────────
        self._bridge      = CvBridge()
        self._frames      = 0
        self._t0          = time.time()

        # Pre-compute angle LUT (filled on first depth frame)
        self._angles: np.ndarray | None = None
        self._last_width: int = 0

        # ── Publishers: images ────────────────────────────────────────────
        self._pub_rgb_img = self.create_publisher(
            Image,      '/camera/rgb/image_raw',     _SENSOR_QOS)
        self._pub_rgb_ci  = self.create_publisher(
            CameraInfo, '/camera/rgb/camera_info',   _SENSOR_QOS)
        self._pub_dep_raw = self.create_publisher(
            Image,      '/camera/depth/image_raw',   _SENSOR_QOS)
        self._pub_dep_m   = self.create_publisher(
            Image,      '/camera/depth/image_meters', _SENSOR_QOS)
        self._pub_dep_ci  = self.create_publisher(
            CameraInfo, '/camera/depth/camera_info', _SENSOR_QOS)

        # ── Publisher: scan ───────────────────────────────────────────────
        if self._pub_scan:
            self._pub_scan_msg = self.create_publisher(
                LaserScan, scan_topic, 10)

        # ── Timer ─────────────────────────────────────────────────────────
        self._timer = self.create_timer(1.0 / fps, self._capture)

        self.get_logger().info(
            f'KinectDriverNode started '
            f'[device={self._dev}, target={fps:.0f} Hz, '
            f'scan={self._pub_scan}, '
            f'scan_height={self._scan_h}, '
            f'range=[{self._range_min}, {self._range_max}] m]'
        )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _param(self, name):
        return self.get_parameter(name).value

    def _build_ci(self, info: dict) -> CameraInfo:
        msg = CameraInfo()
        msg.header.stamp       = info['stamp']
        msg.header.frame_id    = info['frame_id']
        msg.width              = info['width']
        msg.height             = info['height']
        msg.distortion_model   = info['distortion_model']
        msg.d = info['d']
        msg.k = info['k']
        msg.r = info['r']
        msg.p = info['p']
        return msg

    def _precompute_angles(self, width: int) -> np.ndarray:
        """Horizontal angle [rad] for each pixel column, using depth intrinsics."""
        cols = np.arange(width, dtype=np.float64)
        return np.arctan2(cols - self._depth_cx, self._depth_fx)

    # ── LaserScan conversion (runs inside the capture callback) ───────────────

    def _depth_to_laserscan(self, depth_m: np.ndarray, stamp) -> LaserScan:
        """
        Convert a 32FC1 depth image (metres) to a sensor_msgs/LaserScan.

        Algorithm
        ─────────
        1. Extract the central `scan_height` rows (horizontal band at eye level).
        2. Mask pixels outside [range_min, range_max] and non-finite values.
        3. Reduce each column to a single range value:
               use_min=True  → nanmin  (closest obstacle – default, safest)
               use_min=False → nanmean (average depth)
        4. Columns with no valid pixels → inf (LaserScan convention).
        5. Angles computed from depth camera intrinsics (fx, cx).
        """
        h, w = depth_m.shape

        # Recompute angle LUT only when image width changes (once at startup)
        if w != self._last_width:
            self._angles     = self._precompute_angles(w)
            self._last_width = w
            self.get_logger().info(
                f'LaserScan FOV: '
                f'{math.degrees(float(self._angles[0])):.1f}° … '
                f'{math.degrees(float(self._angles[-1])):.1f}°  '
                f'({w} columns)'
            )

        # Central ROI extraction
        cy   = h // 2
        half = self._scan_h // 2
        r0   = max(0, cy - half)
        r1   = min(h, cy + half)
        roi  = depth_m[r0:r1, :].copy()   # shape: (scan_height, width)

        # Mask invalid pixels → NaN so nanmin/nanmean ignores them
        bad = (
            ~np.isfinite(roi)          |
            (roi <= self._range_min)   |
            (roi >  self._range_max)
        )
        roi[bad] = np.nan

        # Column reduction
        # np.errstate suppresses the "All-NaN slice" RuntimeWarning that numpy
        # emits when an entire column is masked (e.g. a reflective surface or
        # a depth hole spans the full scan_height band).  The result for that
        # column is NaN, which we correctly convert to inf below – so the
        # warning is expected and harmless; we just don't want it in the log.
        with np.errstate(all='ignore'):
            if self._use_min:
                ranges = np.nanmin(roi, axis=0)
            else:
                ranges = np.nanmean(roi, axis=0)

        # NaN → inf  (no obstacle detected in this direction)
        ranges = np.where(np.isnan(ranges), np.inf, ranges).astype(np.float32)

        # Build message
        scan                  = LaserScan()
        scan.header.stamp     = stamp
        scan.header.frame_id  = self._dep_fid   # same frame as depth camera
        scan.angle_min        = float(self._angles[0])
        scan.angle_max        = float(self._angles[-1])
        scan.angle_increment  = float(
            (self._angles[-1] - self._angles[0]) / max(w - 1, 1)
        )
        scan.time_increment   = 0.0
        scan.scan_time        = 1.0 / 30.0      # Kinect depth is ~30 Hz
        scan.range_min        = float(self._range_min)
        scan.range_max        = float(self._range_max)
        scan.ranges           = ranges.tolist()
        scan.intensities      = []

        return scan

    # ── Main capture callback ─────────────────────────────────────────────────

    def _capture(self):
        stamp = self.get_clock().now().to_msg()

        # ── RGB ───────────────────────────────────────────────────────────
        if self._pub_rgb:
            try:
                rgb, _ = freenect.sync_get_video(self._dev)

                msg            = self._bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
                msg.header     = Header(stamp=stamp, frame_id=self._rgb_fid)
                self._pub_rgb_img.publish(msg)

                ci = make_camera_info_dict(
                    stamp, self._rgb_fid, 640, 480,
                    self._param('rgb_fx'), self._param('rgb_fy'),
                    self._param('rgb_cx'), self._param('rgb_cy'),
                    dist=[0.2624, -0.9531, -0.0054, 0.0026, 1.1633],
                )
                self._pub_rgb_ci.publish(self._build_ci(ci))

            except Exception as exc:
                self.get_logger().warn(
                    f'RGB capture failed: {exc}',
                    throttle_duration_sec=2.0,
                )

        # ── Depth + LaserScan ─────────────────────────────────────────────
        if self._pub_dep:
            try:
                raw, _ = freenect.sync_get_depth(self._dev)

                # Raw 16UC1
                raw_msg        = self._bridge.cv2_to_imgmsg(raw, encoding='16UC1')
                raw_msg.header = Header(stamp=stamp, frame_id=self._dep_fid)
                self._pub_dep_raw.publish(raw_msg)

                # Metric float32
                depth_m        = raw_to_meters(raw, self._c1, self._c2)
                m_msg          = self._bridge.cv2_to_imgmsg(depth_m, encoding='32FC1')
                m_msg.header   = Header(stamp=stamp, frame_id=self._dep_fid)
                self._pub_dep_m.publish(m_msg)

                # CameraInfo
                ci = make_camera_info_dict(
                    stamp, self._dep_fid, 640, 480,
                    self._param('depth_fx'), self._param('depth_fy'),
                    self._param('depth_cx'), self._param('depth_cy'),
                )
                self._pub_dep_ci.publish(self._build_ci(ci))

                # ── LaserScan (built-in, no extra node needed) ────────────
                if self._pub_scan:
                    scan = self._depth_to_laserscan(depth_m, stamp)
                    self._pub_scan_msg.publish(scan)

            except Exception as exc:
                self.get_logger().warn(
                    f'Depth capture failed: {exc}',
                    throttle_duration_sec=2.0,
                )

        # ── Periodic diagnostics ──────────────────────────────────────────
        self._frames += 1
        if self._frames % 300 == 0:
            elapsed = time.time() - self._t0
            fps     = self._frames / elapsed
            self.get_logger().info(
                f'Running at {fps:.1f} Hz ({self._frames} frames)'
            )


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = KinectDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Use print here, not get_logger().  After Ctrl+C the ROS context
        # begins shutting down before this line runs, so any logger call
        # produces "Failed to publish log message to rosout" on stderr.
        print('[kinect_driver_node] Shutting down (KeyboardInterrupt).')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()