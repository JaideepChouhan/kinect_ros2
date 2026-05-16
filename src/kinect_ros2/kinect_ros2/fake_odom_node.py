#!/usr/bin/env python3
# Copyright 2024 ROS2 Community -- Apache-2.0
"""
fake_odom_node.py  –  Publish a static odom→base_footprint TF + /odom topic.

PURPOSE
────────
Nav2 requires /odom (nav_msgs/Odometry) and the TF chain
  odom → base_footprint
to exist before it will activate.  When your robot has no wheel encoders
yet (or the Arduino/micro-ROS firmware is not ready), this node provides
a "standing still at origin" odometry so the rest of the stack can run.

REMOVE this node (set fake_odom:=false in the launch file) once your
real odometry source is publishing /odom.

Published
──────────
  /odom          nav_msgs/Odometry    10 Hz  (robot standing still at origin)
  /tf            geometry_msgs/TransformStamped  (odom → base_footprint)

Parameters
───────────
  publish_rate   float   10.0   Hz
  odom_frame     str     odom
  base_frame     str     base_footprint
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class FakeOdomNode(Node):
    def __init__(self) -> None:
        super().__init__('fake_odom_node')

        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('odom_frame',   'odom')
        self.declare_parameter('base_frame',   'base_footprint')

        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        rate             = self.get_parameter('publish_rate').value

        self._pub = self.create_publisher(Odometry, '/odom', 10)
        self._tf  = TransformBroadcaster(self)

        self._timer = self.create_timer(1.0 / rate, self._publish)
        self.get_logger().warn(
            '*** FakeOdomNode running – robot appears stationary at origin. '
            'Replace with real wheel-encoder odometry before mapping/navigating! ***'
        )

    def _publish(self) -> None:
        now = self.get_clock().now().to_msg()

        # ── /odom message ─────────────────────────────────────────────────
        odom                         = Odometry()
        odom.header.stamp            = now
        odom.header.frame_id         = self._odom_frame
        odom.child_frame_id          = self._base_frame
        odom.pose.pose.orientation.w = 1.0   # identity quaternion
        self._pub.publish(odom)

        # ── TF: odom → base_footprint ─────────────────────────────────────
        tf                           = TransformStamped()
        tf.header.stamp              = now
        tf.header.frame_id           = self._odom_frame
        tf.child_frame_id            = self._base_frame
        tf.transform.rotation.w      = 1.0
        self._tf.sendTransform(tf)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeOdomNode()
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
