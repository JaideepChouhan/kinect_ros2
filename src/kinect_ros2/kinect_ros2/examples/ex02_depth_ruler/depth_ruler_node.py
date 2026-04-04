#!/usr/bin/env python3
"""
Example 02: Real-Time Depth Ruler
===================================
Subscribes to /kinect/depth/image_meters and publishes:
  - /kinect/ruler/distance_m     Float32  nearest distance in central ROI
  - /kinect/ruler/depth_colormap Image    false-colour depth image

Run with:
    # Terminal 1: start the camera driver
    ros2 run kinect_ros2 kinect_driver
    # Terminal 2: start this node
    ros2 run kinect_ros2 ex02_depth_ruler
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge


class DepthRulerNode(Node):
    def __init__(self):
        super().__init__('depth_ruler_node')

        # Parameters
        self.declare_parameter('roi_cx',     0.50)
        self.declare_parameter('roi_cy',     0.50)
        self.declare_parameter('roi_width',  0.20)
        self.declare_parameter('roi_height', 0.20)
        self.declare_parameter('percentile', 10.0)
        self.declare_parameter('frame_id', 'kinect_depth_frame')

        self._rcx  = self.get_parameter('roi_cx').value
        self._rcy  = self.get_parameter('roi_cy').value
        self._rw   = self.get_parameter('roi_width').value
        self._rh   = self.get_parameter('roi_height').value
        self._pct  = self.get_parameter('percentile').value
        self._fid  = self.get_parameter('frame_id').value

        self._bridge = CvBridge()

        # Publishers
        self._pub_dist = self.create_publisher(Float32, '/kinect/ruler/distance_m', 10)
        self._pub_cmap = self.create_publisher(Image, '/kinect/ruler/depth_colormap', 10)

        # Subscriber
        self.create_subscription(Image, '/kinect/depth/image_meters', self._depth_cb, 10)

        self.get_logger().info(
            f'DepthRulerNode ready | ROI centre=({self._rcx:.0%},{self._rcy:.0%}) '
            f'size=({self._rw:.0%}x{self._rh:.0%}) | percentile={self._pct:.0f}')

    def _depth_cb(self, msg: Image):
        depth_m = self._bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        h, w = depth_m.shape

        # ROI extraction
        x0 = int((self._rcx - self._rw/2) * w)
        x1 = int((self._rcx + self._rw/2) * w)
        y0 = int((self._rcy - self._rh/2) * h)
        y1 = int((self._rcy + self._rh/2) * h)
        roi = depth_m[y0:y1, x0:x1]

        valid = roi[(roi > 0.3) & (roi < 8.0)]
        dist = float(np.percentile(valid, self._pct)) if len(valid) else float('nan')

        # Publish distance
        d_msg = Float32()
        d_msg.data = dist if not np.isnan(dist) else -1.0
        self._pub_dist.publish(d_msg)

        # Publish false-colour image
        self._publish_colormap(depth_m, x0, y0, x1, y1, dist, msg.header.stamp)

    def _publish_colormap(self, depth_m, x0, y0, x1, y1, dist, stamp):
        clipped = np.clip(depth_m, 0.3, 4.0)
        normed = ((clipped - 0.3) / (4.0 - 0.3) * 255).astype(np.uint8)
        normed = 255 - normed  # invert so near=hot
        cmap = cv2.applyColorMap(normed, cv2.COLORMAP_JET)
        cmap[depth_m == 0] = [0, 0, 0]

        cv2.rectangle(cmap, (x0, y0), (x1, y1), (255,255,255), 2)
        if not np.isnan(dist) and dist > 0:
            cv2.putText(cmap, f'{dist:.2f} m', (x0+4, y0-6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,255,255), 2)

        rgb_cmap = cv2.cvtColor(cmap, cv2.COLOR_BGR2RGB)
        out = self._bridge.cv2_to_imgmsg(rgb_cmap, encoding='rgb8')
        out.header.stamp = stamp
        out.header.frame_id = self._fid
        self._pub_cmap.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DepthRulerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
