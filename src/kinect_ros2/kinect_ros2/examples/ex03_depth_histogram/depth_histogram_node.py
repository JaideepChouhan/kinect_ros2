#!/usr/bin/env python3
"""
Example 03: Depth Histogram Visualiser (Corrected QoS)
========================================================
Subscribes to /camera/depth/image_meters (BEST_EFFORT QoS) and publishes
a bar-chart image and mean depth.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge

CHART_W = 640
CHART_H = 320
MARGIN_L = 50
MARGIN_B = 40
MARGIN_T = 20
MARGIN_R = 20
HIST_MIN = 0.3   # metres
HIST_MAX = 5.0
N_BINS   = 47


class DepthHistogramNode(Node):
    def __init__(self):
        super().__init__('depth_histogram_node')

        self.declare_parameter('n_bins', N_BINS)
        self.declare_parameter('hist_min', HIST_MIN)
        self.declare_parameter('hist_max', HIST_MAX)

        self._n_bins = self.get_parameter('n_bins').value
        self._hmin   = self.get_parameter('hist_min').value
        self._hmax   = self.get_parameter('hist_max').value

        self._bridge = CvBridge()
        self._bins   = np.linspace(self._hmin, self._hmax, self._n_bins + 1)

        self._pub_img  = self.create_publisher(Image,   '/kinect/histogram/image', 10)
        self._pub_mean = self.create_publisher(Float32, '/kinect/histogram/mean_m', 10)

        # QoS matching the camera driver (BEST_EFFORT)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.create_subscription(Image, '/camera/depth/image_meters', self._depth_cb, qos)

        self.get_logger().info(f'DepthHistogramNode ready | bins={self._n_bins} | range=[{self._hmin}, {self._hmax}] m')

    def _depth_cb(self, msg: Image):
        depth_m = self._bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        valid = depth_m[(depth_m >= self._hmin) & (depth_m <= self._hmax)]
        if len(valid) == 0:
            return  # no valid depth points

        counts, _ = np.histogram(valid, bins=self._bins)
        mean_d = float(np.mean(valid))

        # Publish mean
        m_msg = Float32()
        m_msg.data = mean_d
        self._pub_mean.publish(m_msg)

        # Draw and publish chart
        chart = self._draw_chart(counts, mean_d)
        out = self._bridge.cv2_to_imgmsg(chart, encoding='rgb8')
        out.header.stamp = msg.header.stamp
        self._pub_img.publish(out)

    def _draw_chart(self, counts: np.ndarray, mean_d: float) -> np.ndarray:
        bg_color   = (18, 28, 44)
        bar_color  = (0, 180, 215)
        mean_color = (247, 127, 0)
        text_color = (200, 210, 220)
        grid_color = (40, 55, 75)

        img = np.full((CHART_H, CHART_W, 3), bg_color, dtype=np.uint8)
        img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        plot_w = CHART_W - MARGIN_L - MARGIN_R
        plot_h = CHART_H - MARGIN_T - MARGIN_B
        max_c = max(counts.max(), 1)
        bar_w = plot_w // self._n_bins

        # Grid lines
        for i in range(1, 5):
            y = MARGIN_T + int(plot_h * (1 - i/4))
            cv2.line(img_bgr, (MARGIN_L, y), (CHART_W - MARGIN_R, y), grid_color, 1)
            cv2.putText(img_bgr, f'{int(max_c * i / 4)}', (4, y+4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, text_color, 1)

        # Bars
        for i, c in enumerate(counts):
            bh = int(plot_h * c / max_c)
            x0 = MARGIN_L + i * bar_w + 1
            x1 = x0 + bar_w - 2
            y0 = MARGIN_T + plot_h - bh
            y1 = MARGIN_T + plot_h
            ratio = i / self._n_bins
            b_col = (int(bar_color[0]*(1-ratio) + 20*ratio),
                     int(bar_color[1]*(1-ratio) + 120*ratio),
                     int(bar_color[2]*ratio + 50*(1-ratio)))
            cv2.rectangle(img_bgr, (x0, y0), (x1, y1), b_col, -1)

        # Mean line
        mean_x = MARGIN_L + int((mean_d - self._hmin) / (self._hmax - self._hmin) * plot_w)
        cv2.line(img_bgr, (mean_x, MARGIN_T), (mean_x, MARGIN_T+plot_h), mean_color, 2)
        cv2.putText(img_bgr, f'mean={mean_d:.2f}m', (mean_x+4, MARGIN_T+16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, mean_color, 1)

        # X-axis labels
        for i in range(0, self._n_bins+1, self._n_bins//5):
            v = self._bins[i]
            x = MARGIN_L + int((v - self._hmin) / (self._hmax - self._hmin) * plot_w)
            cv2.putText(img_bgr, f'{v:.1f}m', (x-12, CHART_H-6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, text_color, 1)

        cv2.putText(img_bgr, 'Depth Distribution [kinect_ros2 v2]',
                    (MARGIN_L, MARGIN_T-4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 1)

        return cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)


def main(args=None):
    rclpy.init(args=args)
    node = DepthHistogramNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
