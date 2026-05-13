#!/usr/bin/env python3
# Copyright 2024 ROS2 Community -- Apache-2.0
"""
depth_to_laserscan_node.py  –  Convert Kinect depth image to LaserScan for Nav2.

The Kinect v1 depth camera has a horizontal FOV of ~57.8° and outputs
a 640×480 float32 depth image in metres. This node extracts the central
`scan_height` rows, takes the minimum valid depth per column, and publishes
a sensor_msgs/LaserScan – exactly what Nav2 and slam_toolbox expect.

Horizontal angle per column (depth camera intrinsics from kinect_driver_node):
    angle_j = arctan( (j − cx) / fx )
    fx = 580.0 px,  cx = 319.5 px,  width = 640 px
    → angle_min ≈ −0.505 rad (−28.9°)   column 0   = left
    → angle_max ≈ +0.505 rad (+28.9°)   column 639 = right

Why use minimum depth across rows?
    Taking the minimum finds the closest obstacle in the vertical slice,
    which is the safest choice for navigation: the robot avoids the nearest
    object regardless of its exact height in the image.

Published topics
─────────────────
  /scan    (sensor_msgs/LaserScan)   – ready for slam_toolbox and Nav2

Subscribed topics
──────────────────
  /camera/depth/image_meters  (sensor_msgs/Image, 32FC1, BEST_EFFORT QoS)

Parameters
───────────
  scan_height      int    20      Number of central depth rows to sample
  range_min        float  0.40    Minimum valid range (m) – Kinect hardware limit
  range_max        float  3.50    Maximum useful range (m)
  scan_frame_id    str    kinect_depth_optical_frame
  use_min          bool   true    true = min depth per col; false = mean
  depth_fx         float  580.0   Depth camera focal length (px)
  depth_cx         float  319.5   Depth camera principal point x (px)
  output_topic     str    /scan

TF requirement
──────────────
  The scan is published in `scan_frame_id`. Nav2 needs a static TF from
  base_link → kinect_depth_optical_frame.  Add to your launch file:

  Node(package='tf2_ros', executable='static_transform_publisher',
       arguments=['0.0', '0.0', '0.3',    # x y z  (Kinect height on robot)
                  '0', '0', '0', '1',      # qx qy qz qw
                  'base_link', 'kinect_depth_optical_frame'])
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
)
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge

# Match the kinect_driver_node QoS (BEST_EFFORT)
_SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class DepthToLaserScanNode(Node):
    """Converts Kinect depth image to LaserScan for Nav2 / slam_toolbox."""

    def __init__(self) -> None:
        super().__init__('depth_to_laserscan_node')

        # ── Parameters ─────────────────────────────────────────────────────
        self.declare_parameter('scan_height',   20)
        self.declare_parameter('range_min',      0.40)
        self.declare_parameter('range_max',      3.50)
        self.declare_parameter('scan_frame_id', 'kinect_depth_optical_frame')
        self.declare_parameter('use_min',        True)
        self.declare_parameter('depth_fx',      580.0)
        self.declare_parameter('depth_cx',      319.5)
        self.declare_parameter('output_topic',  '/scan')

        p = self.get_parameter
        self._scan_height = p('scan_height').value
        self._range_min   = p('range_min').value
        self._range_max   = p('range_max').value
        self._frame_id    = p('scan_frame_id').value
        self._use_min     = p('use_min').value
        self._fx          = p('depth_fx').value
        self._cx          = p('depth_cx').value
        out_topic         = p('output_topic').value

        self._bridge = CvBridge()

        # Pre-computed angle arrays (filled on first depth message)
        self._angles: np.ndarray | None = None
        self._last_width: int = 0

        # ── Publishers / Subscribers ────────────────────────────────────────
        self._pub = self.create_publisher(LaserScan, out_topic, 10)

        self.create_subscription(
            Image,
            '/camera/depth/image_meters',
            self._depth_cb,
            _SENSOR_QOS,
        )

        # ── Diagnostics ────────────────────────────────────────────────────
        self._msg_count = 0

        self.get_logger().info(
            f'DepthToLaserScanNode ready\n'
            f'  scan_height : {self._scan_height} central rows\n'
            f'  range       : [{self._range_min}, {self._range_max}] m\n'
            f'  reduction   : {"MIN" if self._use_min else "MEAN"} per column\n'
            f'  frame_id    : {self._frame_id}\n'
            f'  publishing  : {out_topic}'
        )

    # ── Pre-computation ───────────────────────────────────────────────────────

    def _precompute_angles(self, width: int) -> np.ndarray:
        """Return horizontal angle (rad) for each pixel column."""
        cols = np.arange(width, dtype=np.float64)
        return np.arctan2(cols - self._cx, self._fx)

    # ── Depth callback ────────────────────────────────────────────────────────

    def _depth_cb(self, msg: Image) -> None:
        # Convert ROS image to float32 numpy array
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge error: {exc}', throttle_duration_sec=5.0)
            return

        h, w = depth.shape

        # Re-compute angles only when image width changes (should be once)
        if w != self._last_width:
            self._angles     = self._precompute_angles(w)
            self._last_width = w
            angle_min_deg    = math.degrees(float(self._angles[0]))
            angle_max_deg    = math.degrees(float(self._angles[-1]))
            self.get_logger().info(
                f'Image width={w}  FOV: {angle_min_deg:.1f}° to {angle_max_deg:.1f}°'
            )

        # ── Extract central ROI ────────────────────────────────────────────
        cy        = h // 2
        half      = self._scan_height // 2
        r0        = max(0,  cy - half)
        r1        = min(h,  cy + half)
        roi       = depth[r0:r1, :].copy()      # shape (scan_height, width)

        # ── Mask invalid depths ────────────────────────────────────────────
        # 0.0 = no reading; nan/inf = error; out-of-range = beyond sensor
        invalid = (
            (roi <= self._range_min) |
            (roi >  self._range_max) |
            ~np.isfinite(roi)
        )
        roi[invalid] = np.nan

        # ── Reduce rows → one range per column ────────────────────────────
        if self._use_min:
            ranges = np.nanmin(roi, axis=0)     # closest obstacle per column
        else:
            ranges = np.nanmean(roi, axis=0)    # average depth per column

        # LaserScan convention: inf means no valid reading in that direction
        ranges = np.where(np.isnan(ranges), np.inf, ranges).astype(np.float32)

        # ── Build LaserScan message ────────────────────────────────────────
        scan                 = LaserScan()
        scan.header.stamp    = msg.header.stamp
        scan.header.frame_id = self._frame_id
        scan.angle_min       = float(self._angles[0])
        scan.angle_max       = float(self._angles[-1])
        scan.angle_increment = float(
            (self._angles[-1] - self._angles[0]) / max(w - 1, 1)
        )
        scan.time_increment  = 0.0
        scan.scan_time       = 1.0 / 30.0       # Kinect depth rate
        scan.range_min       = float(self._range_min)
        scan.range_max       = float(self._range_max)
        scan.ranges          = ranges.tolist()

        self._pub.publish(scan)

        # Periodic diagnostics
        self._msg_count += 1
        if self._msg_count % 300 == 0:
            valid_pct = float(np.isfinite(ranges).sum()) / w * 100
            self.get_logger().info(
                f'LaserScan #{self._msg_count}  '
                f'valid={valid_pct:.0f}%  '
                f'min={float(np.nanmin(ranges[np.isfinite(ranges)])):.2f}m'
                if np.any(np.isfinite(ranges)) else
                f'LaserScan #{self._msg_count}  all inf (no valid readings)'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthToLaserScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
