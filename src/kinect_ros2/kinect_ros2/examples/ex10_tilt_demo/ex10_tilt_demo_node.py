#!/usr/bin/env python3
# Copyright 2024 ROS2 Community -- Apache-2.0
"""
ex10_tilt_demo_node.py  –  Tilt motor control example for Kinect v1 (1473).

Demonstrates three modes of tilt control by subscribing to the feedback
topics published by kinect_tilt_accel (C++ node) and publishing commands
back to /kinect/tilt_cmd.

Modes
─────
  sweep   (default)
      Smoothly sweeps the Kinect between tilt_min and tilt_max degrees,
      reversing direction at each limit.  Step size and rate are configurable.
      Good for scanning a room or testing the motor range.

  hold
      Moves to a fixed target angle and holds it.
      Useful for verifying a specific camera angle.

  follow
      Tilts toward detected objects using /kinect/proximity_alert.
        DANGER  → tilt down  (object close, look down at floor)
        WARNING → tilt up    (object at mid range, track it)
        CLEAR   → return to 0°
      Demonstrates sensor-driven tilt behaviour.

Subscribed topics
──────────────────
  /kinect/tilt_angle     (std_msgs/Float32)      – current tilt from C++ node
  /kinect/accel          (geometry_msgs/Vector3)  – m/s² from C++ node
  /kinect/proximity_alert (std_msgs/String)        – CLEAR/WARNING/DANGER
                                                    (only used in follow mode)
Published topics
─────────────────
  /kinect/tilt_cmd       (std_msgs/Float32)       – desired tilt angle

Parameters
──────────
  mode            str    sweep     sweep | hold | follow
  tilt_min        float  -20.0     degrees, lower sweep limit
  tilt_max        float   20.0     degrees, upper sweep limit
  step_deg        float    1.0     degrees moved per timer tick (sweep mode)
  tick_rate_hz    float    5.0     timer frequency
  hold_angle      float    0.0     target angle for hold mode
  follow_up_deg   float   15.0     angle for WARNING in follow mode
  follow_down_deg float  -15.0     angle for DANGER in follow mode

Run
────
  # Terminal 1: camera driver (firmware upload)
  ros2 run kinect_ros2 kinect_driver

  # Terminal 2: tilt + accel C++ node
  ros2 run kinect_audio_cpp kinect_tilt_accel

  # Terminal 3: this example
  ros2 run kinect_ros2 ex10_tilt_demo

  # Change mode at runtime:
  ros2 param set /tilt_demo_node mode hold
  ros2 param set /tilt_demo_node hold_angle 15.0
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Vector3


class TiltDemoNode(Node):
    """Tilt control example demonstrating sweep, hold, and follow modes."""

    def __init__(self) -> None:
        super().__init__('tilt_demo_node')

        # ── Parameters ─────────────────────────────────────────────────────
        self.declare_parameter('mode',            'sweep')
        self.declare_parameter('tilt_min',       -20.0)
        self.declare_parameter('tilt_max',        20.0)
        self.declare_parameter('step_deg',         1.0)
        self.declare_parameter('tick_rate_hz',     5.0)
        self.declare_parameter('hold_angle',       0.0)
        self.declare_parameter('follow_up_deg',   15.0)
        self.declare_parameter('follow_down_deg', -15.0)

        self._read_params()

        # ── State ───────────────────────────────────────────────────────────
        self._target      = 0.0       # current commanded angle
        self._direction   = 1.0       # +1 = moving up, -1 = moving down
        self._actual      = 0.0       # last reported tilt angle
        self._accel       = (0.0, 0.0, 9.81)
        self._alert       = 'CLEAR'
        self._cmd_count   = 0

        # ── Publishers ──────────────────────────────────────────────────────
        self._pub = self.create_publisher(Float32, '/kinect/tilt_cmd', 10)

        # ── Subscribers ─────────────────────────────────────────────────────
        self.create_subscription(
            Float32, '/kinect/tilt_angle', self._tilt_cb, 10
        )
        self.create_subscription(
            Vector3, '/kinect/accel', self._accel_cb, 10
        )
        self.create_subscription(
            String, '/kinect/proximity_alert', self._alert_cb, 10
        )

        # ── Timer ────────────────────────────────────────────────────────────
        period = 1.0 / self.get_parameter('tick_rate_hz').value
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f'\n{"─"*50}\n'
            f' ex10_tilt_demo_node\n'
            f'  mode      : {self._mode}\n'
            f'  range     : [{self._tilt_min:.0f}°, {self._tilt_max:.0f}°]\n'
            f'  step      : {self._step_deg:.1f}° / tick\n'
            f'  rate      : {self.get_parameter("tick_rate_hz").value:.0f} Hz\n'
            f'\n'
            f'  /kinect/tilt_cmd  ← publishing commands\n'
            f'  /kinect/tilt_angle← reading feedback\n'
            f'  /kinect/accel     ← reading accelerometer\n'
            f'{"─"*50}'
        )

    # ── Parameter helper ─────────────────────────────────────────────────────

    def _read_params(self) -> None:
        p = self.get_parameter
        self._mode           = p('mode').value
        self._tilt_min       = p('tilt_min').value
        self._tilt_max       = p('tilt_max').value
        self._step_deg       = p('step_deg').value
        self._hold_angle     = p('hold_angle').value
        self._follow_up      = p('follow_up_deg').value
        self._follow_down    = p('follow_down_deg').value

    # ── Subscribers ──────────────────────────────────────────────────────────

    def _tilt_cb(self, msg: Float32) -> None:
        self._actual = msg.data

    def _accel_cb(self, msg: Vector3) -> None:
        self._accel = (msg.x, msg.y, msg.z)

    def _alert_cb(self, msg: String) -> None:
        self._alert = msg.data.strip().upper()

    # ── Main tick ────────────────────────────────────────────────────────────

    def _tick(self) -> None:
        # Re-read params in case they were changed via ros2 param set
        self._read_params()

        mode = self._mode.lower()
        if mode == 'sweep':
            self._tick_sweep()
        elif mode == 'hold':
            self._tick_hold()
        elif mode == 'follow':
            self._tick_follow()
        else:
            self.get_logger().warn(
                f'Unknown mode "{mode}". Use: sweep | hold | follow'
            )
            return

        self._send_cmd()

    # ── Mode: sweep ──────────────────────────────────────────────────────────

    def _tick_sweep(self) -> None:
        """Move target by one step; reverse at limits."""
        self._target += self._direction * self._step_deg

        if self._target >= self._tilt_max:
            self._target   = self._tilt_max
            self._direction = -1.0
            self.get_logger().info(
                f'[SWEEP] Reached max {self._tilt_max:.0f}° → reversing ↓'
            )

        elif self._target <= self._tilt_min:
            self._target   = self._tilt_min
            self._direction = 1.0
            self.get_logger().info(
                f'[SWEEP] Reached min {self._tilt_min:.0f}° → reversing ↑'
            )

    # ── Mode: hold ───────────────────────────────────────────────────────────

    def _tick_hold(self) -> None:
        """Move toward the hold_angle target."""
        new_target = max(self._tilt_min,
                         min(self._tilt_max, self._hold_angle))

        if abs(new_target - self._target) > 0.1:
            self._target = new_target
            self.get_logger().info(
                f'[HOLD]  Target → {self._target:.1f}°'
            )

    # ── Mode: follow ─────────────────────────────────────────────────────────

    def _tick_follow(self) -> None:
        """Tilt in response to proximity alert level."""
        prev = self._target

        if self._alert == 'DANGER':
            self._target = self._follow_down     # look down toward close object
        elif self._alert == 'WARNING':
            self._target = self._follow_up       # tilt up to track distant object
        else:                                    # CLEAR
            self._target = 0.0                   # return to neutral

        if abs(self._target - prev) > 0.1:
            self.get_logger().info(
                f'[FOLLOW] Alert={self._alert}  → {self._target:.0f}°'
            )

    # ── Command publisher ────────────────────────────────────────────────────

    def _send_cmd(self) -> None:
        """Clamp and publish the target angle."""
        clamped = max(self._tilt_min, min(self._tilt_max, self._target))

        msg = Float32()
        msg.data = float(clamped)
        self._pub.publish(msg)

        self._cmd_count += 1

        # Log every 10 commands so the terminal isn't flooded
        if self._cmd_count % 10 == 0:
            ax, ay, az = self._accel
            # Gravity magnitude – should stay near 9.81 m/s²
            g_mag = math.sqrt(ax**2 + ay**2 + az**2)

            self.get_logger().info(
                f'[{self._mode.upper():6s}]  '
                f'cmd={clamped:+6.1f}°  '
                f'actual={self._actual:+6.1f}°  '
                f'accel=[{ax:+.2f},{ay:+.2f},{az:+.2f}] m/s²  '
                f'|g|={g_mag:.2f}'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TiltDemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down – returning tilt to 0°')
        # Return to centre on exit
        pub = node.create_publisher(Float32, '/kinect/tilt_cmd', 10)
        pub.publish(Float32(data=0.0))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
