#!/usr/bin/env python3
# Copyright 2024 ROS2 Community -- Apache-2.0
"""
tilt_control_node.py - Robust Kinect tilt motor & accelerometer interface.

Handles multiple libfreenect Python binding API versions.
Gracefully degrades if tilt functions are unavailable.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3

try:
    import freenect
    _FREENECT_AVAILABLE = True
except ImportError:
    _FREENECT_AVAILABLE = False

_TILT_MIN = -27.0
_TILT_MAX = 27.0
_ACCEL_SCALE = 9.81 / 819.0   # raw int16 -> m/s²


class TiltControlNode(Node):
    def __init__(self):
        super().__init__('tilt_control_node')

        if not _FREENECT_AVAILABLE:
            self.get_logger().fatal('freenect module not installed. Run: pip3 install freenect')
            raise RuntimeError('freenect missing')

        # Declare parameters
        self.declare_parameter('device_index', 0)
        self.declare_parameter('publish_rate_hz', 5.0)

        self._dev_idx = self.get_parameter('device_index').value
        rate = self.get_parameter('publish_rate_hz').value

        # Initialize libfreenect and open device
        self._ctx = None
        self._dev = None
        try:
            self._ctx = freenect.init()
            self._dev = freenect.open_device(self._ctx, self._dev_idx)
            if not self._dev:
                raise RuntimeError(f'Cannot open Kinect device {self._dev_idx}')
            self.get_logger().info(f'Opened device {self._dev_idx}')
        except Exception as e:
            self.get_logger().error(f'Device init failed: {e}')
            raise

        # Publishers & subscribers
        self._pub_tilt = self.create_publisher(Float32, '/kinect/tilt_angle', 10)
        self._pub_accel = self.create_publisher(Vector3, '/kinect/accel', 10)
        self._sub_cmd = self.create_subscription(
            Float32, '/kinect/tilt_cmd', self._cmd_callback, 10
        )

        # Periodic read timer
        self._timer = self.create_timer(1.0 / rate, self._read_and_publish)

        # Detect which API variant is available
        self._tilt_api_style = self._detect_tilt_api()
        self.get_logger().info(f'Tilt API style: {self._tilt_api_style}')

        self.get_logger().info('TiltControlNode ready.')

    def _detect_tilt_api(self):
        """Determine correct calling convention for tilt functions."""
        # Try to get tilt angle using device argument
        try:
            angle = freenect.get_tilt_degs(self._dev)
            self.get_logger().debug(f'get_tilt_degs(dev) works: {angle}')
            return 'device'
        except TypeError:
            pass

        # Try using state object
        try:
            state = freenect.get_tilt_state(self._dev)
            angle = freenect.get_tilt_degs(state)
            self.get_logger().debug(f'get_tilt_degs(state) works: {angle}')
            return 'state'
        except (TypeError, AttributeError):
            pass

        # Fallback: no working API
        self.get_logger().warning(
            'Cannot determine tilt API. Tilt angle will be 0.0 and accel may not work.'
        )
        return 'none'

    def _get_tilt_angle(self):
        """Return current tilt angle in degrees using detected API."""
        if self._tilt_api_style == 'device':
            return float(freenect.get_tilt_degs(self._dev))
        elif self._tilt_api_style == 'state':
            state = freenect.get_tilt_state(self._dev)
            return float(freenect.get_tilt_degs(state))
        else:
            return 0.0

    def _get_accel(self):
        """Return accelerometer (x, y, z) as raw ints from device."""
        try:
            if self._tilt_api_style == 'device':
                # Some bindings allow get_accel(dev)
                return freenect.get_accel(self._dev)
            elif self._tilt_api_style == 'state':
                state = freenect.get_tilt_state(self._dev)
                return freenect.get_accel(state)
            else:
                return (0, 0, 0)
        except Exception:
            return (0, 0, 0)

    def _cmd_callback(self, msg: Float32):
        """Command to set tilt angle."""
        angle = max(_TILT_MIN, min(_TILT_MAX, msg.data))
        try:
            # set_tilt_degs usually expects (dev, angle)
            freenect.set_tilt_degs(self._dev, angle)
            self.get_logger().info(f'Tilt set to {angle:.1f}°')
        except Exception as e:
            self.get_logger().error(f'Failed to set tilt: {e}')

    def _read_and_publish(self):
        """Periodic read of tilt angle and accelerometer."""
        try:
            # Tilt angle
            angle = self._get_tilt_angle()
            angle_msg = Float32()
            angle_msg.data = angle
            self._pub_tilt.publish(angle_msg)

            # Accelerometer
            ax, ay, az = self._get_accel()
            accel_msg = Vector3()
            accel_msg.x = ax * _ACCEL_SCALE
            accel_msg.y = ay * _ACCEL_SCALE
            accel_msg.z = az * _ACCEL_SCALE
            self._pub_accel.publish(accel_msg)

        except Exception as e:
            self.get_logger().warn(f'Read/publish error: {e}')

    def __del__(self):
        """Clean up libfreenect resources."""
        try:
            if self._dev:
                freenect.close_device(self._dev)
            if self._ctx:
                freenect.shutdown(self._ctx)
        except Exception:
            pass


def main(args=None):
    # Safely initialize rclpy only once
    if not rclpy.ok():
        rclpy.init(args=args)

    node = TiltControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down TiltControlNode (keyboard)')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()