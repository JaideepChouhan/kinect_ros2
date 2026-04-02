#!/usr/bin/env python3
"""
kinect_device.py
================
Context-manager wrapper around freenect.sync_* calls.

Usage
-----
    from kinect_ros2.kinect_device import KinectDevice

    with KinectDevice(device_index=0) as dev:
        rgb,   ts = dev.get_video()        # (480,640,3) uint8  RGB
        depth, ts = dev.get_depth()        # (480,640)   uint16 disparity
        dev.set_tilt(10.0)                 # degrees [-31, 31]
        acc = dev.get_accel()              # (ax, ay, az) in g

This class is intentionally minimal — it does NOT run a thread or
cache frames. Each call blocks until a fresh frame is available
(typically < 33 ms at 30 Hz).
"""

import time
from typing import Optional, Tuple
import numpy as np

try:
    import freenect
    _FREENECT_OK = True
except ImportError:
    _FREENECT_OK = False

from kinect_ros2.depth_utils import raw_to_meters as raw_depth_to_meters, DEPTH_MIN_M as DEPTH_MIN_METERS, DEPTH_MAX_M as DEPTH_MAX_METERS


class KinectDeviceError(RuntimeError):
    """Raised when the Kinect device cannot be opened or accessed."""


class KinectDevice:
    """
    Context manager providing synchronous access to one Kinect sensor.

    Parameters
    ----------
    device_index : int
        Zero-based index of the Kinect to open (default 0).
    led_color : str
        LED colour on open: 'green', 'red', 'yellow', 'blink_green',
        'blink_red_yellow', or 'off'.  Default 'green'.
    """

    _LED_MAP = {
        'off':              freenect.LED_OFF              if _FREENECT_OK else 0,
        'green':            freenect.LED_GREEN            if _FREENECT_OK else 1,
        'red':              freenect.LED_RED              if _FREENECT_OK else 2,
        'yellow':           freenect.LED_YELLOW           if _FREENECT_OK else 3,
        'blink_green':      freenect.LED_BLINK_GREEN      if _FREENECT_OK else 4,
        'blink_red_yellow': freenect.LED_BLINK_RED_YELLOW if _FREENECT_OK else 6,
    }

    def __init__(self,
                 device_index: int = 0,
                 led_color: str = 'green'):
        if not _FREENECT_OK:
            raise KinectDeviceError(
                'freenect is not installed. '
                'Run: pip3 install freenect --break-system-packages')
        self._index = device_index
        self._led   = led_color
        self._open  = False

    # ── Context-manager protocol ─────────────────────────────

    def __enter__(self) -> 'KinectDevice':
        self._open = True
        led_code = self._LED_MAP.get(self._led, freenect.LED_GREEN)
        try:
            freenect.sync_set_led(led_code, self._index)
        except Exception:
            pass   # LED control is non-critical
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> bool:
        try:
            # Turn off LED on close
            freenect.sync_set_led(freenect.LED_OFF, self._index)
            freenect.sync_stop()
        except Exception:
            pass
        self._open = False
        return False   # do not suppress exceptions

    # ── Data access ──────────────────────────────────────────

    def get_video(self) -> Tuple[np.ndarray, int]:
        """
        Return one RGB video frame.

        Returns
        -------
        (frame, timestamp)
            frame     : ndarray (480, 640, 3)  uint8  RGB order
            timestamp : int  microseconds since Kinect boot
        """
        self._check_open()
        return freenect.sync_get_video(self._index)

    def get_depth(self) -> Tuple[np.ndarray, int]:
        """
        Return one raw disparity frame.

        Returns
        -------
        (frame, timestamp)
            frame     : ndarray (480, 640)  uint16  11-bit disparity
            timestamp : int  microseconds since Kinect boot
        """
        self._check_open()
        return freenect.sync_get_depth(self._index)

    def get_depth_meters(self) -> Tuple[np.ndarray, int]:
        """
        Return depth converted to metres (float32).

        Returns
        -------
        (depth_m, timestamp)
            depth_m   : ndarray (480, 640)  float32  metres; 0 = invalid
            timestamp : int
        """
        raw, ts = self.get_depth()
        return raw_depth_to_meters(raw), ts

    def get_accel(self) -> Optional[Tuple[float, float, float]]:
        """
        Return the accelerometer reading in g units, or None on error.

        Returns
        -------
        (ax, ay, az) : Tuple[float, float, float] | None
        """
        self._check_open()
        try:
            ax, ay, az = freenect.sync_get_accel(self._index)
            return float(ax), float(ay), float(az)
        except Exception:
            return None

    # ── Motor control ────────────────────────────────────────

    def set_tilt(self, degrees: float) -> None:
        """
        Set the Kinect tilt motor angle.

        Parameters
        ----------
        degrees : float
            Target angle in degrees. Clamped to [-31, 31].
        """
        self._check_open()
        angle = max(-31.0, min(31.0, float(degrees)))
        freenect.sync_set_tilt_degs(angle, self._index)

    # ── Internal ─────────────────────────────────────────────

    def _check_open(self):
        if not self._open:
            raise KinectDeviceError(
                'KinectDevice must be used inside a "with" block.')
