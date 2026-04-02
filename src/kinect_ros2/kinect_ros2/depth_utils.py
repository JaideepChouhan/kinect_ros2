#!/usr/bin/env python3
# Copyright 2024 ROS2 Community -- Apache-2.0
"""
Depth conversion utilities for the Kinect v1 sensor.

The Kinect reports raw 11-bit disparity values (0-2047).
These are converted to metric depth (metres) using the
standard OpenKinect calibration formula:

    depth_m = 1 / (raw * C1 + C2)

Typical accuracy: +/-5 mm at 1 m, +/-50 mm at 3 m.
"""

import numpy as np
from typing import Tuple

# Default OpenKinect calibration constants
DEFAULT_C1: float = -0.0030711016
DEFAULT_C2: float =  3.3309495161

# Valid disparity range for Kinect v1
DISPARITY_MIN: int = 1
DISPARITY_MAX: int = 2046   # 2047 = no return (invalid)

# Physical depth limits (metres)
DEPTH_MIN_M: float = 0.40
DEPTH_MAX_M: float = 5.00


def raw_to_meters(
    raw: np.ndarray,
    c1: float = DEFAULT_C1,
    c2: float = DEFAULT_C2,
) -> np.ndarray:
    """Convert Kinect raw 11-bit disparity to metric depth (metres)."""
    depth = np.zeros(raw.shape, dtype=np.float32)
    valid = (raw > DISPARITY_MIN) & (raw < DISPARITY_MAX)
    raw_f = raw.astype(np.float32)
    depth[valid] = 1.0 / (raw_f[valid] * c1 + c2)
    depth = np.where(
        valid,
        np.clip(depth, DEPTH_MIN_M, DEPTH_MAX_M),
        0.0
    )
    return depth


def make_camera_info_dict(
    stamp, frame_id: str,
    width: int, height: int,
    fx: float, fy: float,
    cx: float, cy: float,
    dist: list = None,
) -> dict:
    """Build a dict that maps directly onto sensor_msgs/CameraInfo fields."""
    if dist is None:
        dist = [0.0, 0.0, 0.0, 0.0, 0.0]
    return dict(
        stamp=stamp, frame_id=frame_id,
        width=width, height=height,
        distortion_model='plumb_bob',
        d=dist,
        k=[fx,  0.0, cx,
           0.0, fy,  cy,
           0.0, 0.0, 1.0],
        r=[1.0, 0.0, 0.0,
           0.0, 1.0, 0.0,
           0.0, 0.0, 1.0],
        p=[fx,  0.0, cx, 0.0,
           0.0, fy,  cy, 0.0,
           0.0, 0.0, 1.0, 0.0],
    )


def nearest_in_roi(
    depth_m: np.ndarray,
    roi_fraction: float = 0.5,
    min_valid: float = 0.10,
) -> Tuple[float, int, int]:
    """Return (distance, row, col) of the nearest valid pixel in a central ROI."""
    h, w = depth_m.shape
    if 0.0 < roi_fraction < 1.0:
        my = int(h * (1.0 - roi_fraction) / 2)
        mx = int(w * (1.0 - roi_fraction) / 2)
        roi = depth_m[my:h - my, mx:w - mx]
        r_off, c_off = my, mx
    else:
        roi = depth_m
        r_off, c_off = 0, 0

    valid = roi > min_valid
    if not np.any(valid):
        return float('inf'), -1, -1

    flat = np.argmin(np.where(valid, roi, np.inf))
    row, col = divmod(int(flat), roi.shape[1])
    return float(roi[row, col]), row + r_off, col + c_off
