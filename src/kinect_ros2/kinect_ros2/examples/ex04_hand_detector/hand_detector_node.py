#!/usr/bin/env python3
"""
Example 04: Depth-Based Hand Proximity Detector
=================================================
Detects the nearest blob in the depth image (assumed to be a hand or
close object) and publishes its centroid and a visualised overlay.

Published Topics
----------------
/kinect/hand/detected       std_msgs/Bool         True if hand found
/kinect/hand/centroid_x     std_msgs/Float32      x pixel (0..639)
/kinect/hand/centroid_y     std_msgs/Float32      y pixel (0..479)
/kinect/hand/distance_m     std_msgs/Float32      distance in metres
/kinect/hand/image_overlay  sensor_msgs/Image     Annotated depth-colour image

Run with:
    ros2 run kinect_ros2 kinect_driver          # Terminal 1
    ros2 run kinect_ros2 ex04_hand_detector     # Terminal 2
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from cv_bridge import CvBridge


class HandDetectorNode(Node):
    def __init__(self):
        super().__init__('hand_detector_node')

        self.declare_parameter('near_threshold_m', 0.80)
        self.declare_parameter('min_blob_area_px', 500)
        self.declare_parameter('frame_id', 'kinect_depth_frame')

        self._thresh   = self.get_parameter('near_threshold_m').value
        self._min_area = self.get_parameter('min_blob_area_px').value
        self._fid      = self.get_parameter('frame_id').value

        self._bridge = CvBridge()

        # Publishers
        self._pub_det  = self.create_publisher(Bool,    '/kinect/hand/detected',      10)
        self._pub_cx   = self.create_publisher(Float32, '/kinect/hand/centroid_x',    10)
        self._pub_cy   = self.create_publisher(Float32, '/kinect/hand/centroid_y',    10)
        self._pub_dist = self.create_publisher(Float32, '/kinect/hand/distance_m',    10)
        self._pub_img  = self.create_publisher(Image,   '/kinect/hand/image_overlay', 10)

        # QoS matching the camera driver
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.create_subscription(Image, '/camera/depth/image_meters', self._depth_cb, qos)

        self.get_logger().info(
            f'HandDetectorNode ready | near_threshold={self._thresh:.2f} m | min_blob={self._min_area} px²')

    def _depth_cb(self, msg: Image):
        depth_m = self._bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        h, w    = depth_m.shape
        stamp   = msg.header.stamp

        # Binary mask: pixels in (0.1 m, threshold]
        mask = ((depth_m > 0.10) & (depth_m <= self._thresh)).astype(np.uint8)

        # Find connected components
        n_labels, labels, stats, centroids = \
            cv2.connectedComponentsWithStats(mask, connectivity=8)

        best_label = -1
        best_area  = self._min_area - 1
        for lbl in range(1, n_labels):
            area = stats[lbl, cv2.CC_STAT_AREA]
            if area > best_area:
                best_area = area
                best_label = lbl

        detected = best_label >= 0

        # Publish detection flag
        b_msg = Bool()
        b_msg.data = detected
        self._pub_det.publish(b_msg)

        if detected:
            cx = float(centroids[best_label, 0])
            cy = float(centroids[best_label, 1])

            # Mean depth of the blob
            blob_mask = (labels == best_label)
            blob_depth = depth_m[blob_mask]
            valid_d = blob_depth[(blob_depth > 0.1) & (blob_depth < 8.0)]
            dist_m = float(np.mean(valid_d)) if len(valid_d) else -1.0

            # Publish scalars
            cx_msg = Float32(); cx_msg.data = cx
            cy_msg = Float32(); cy_msg.data = cy
            dm_msg = Float32(); dm_msg.data = dist_m
            self._pub_cx.publish(cx_msg)
            self._pub_cy.publish(cy_msg)
            self._pub_dist.publish(dm_msg)

            # Bounding box
            bx = stats[best_label, cv2.CC_STAT_LEFT]
            by = stats[best_label, cv2.CC_STAT_TOP]
            bw = stats[best_label, cv2.CC_STAT_WIDTH]
            bh = stats[best_label, cv2.CC_STAT_HEIGHT]
        else:
            cx = cy = dist_m = -1.0
            bx = by = bw = bh = 0

        # Build and publish overlay image
        self._publish_overlay(depth_m, mask, detected,
                              cx, cy, dist_m, bx, by, bw, bh, stamp)

    def _publish_overlay(self, depth_m, mask, detected,
                         cx, cy, dist_m, bx, by, bw, bh, stamp):
        # Colourmap background
        clipped = np.clip(depth_m, 0.3, 4.0)
        normed = 255 - ((clipped - 0.3) / 3.7 * 255).astype(np.uint8)
        normed[depth_m == 0] = 0
        cmap = cv2.applyColorMap(normed, cv2.COLORMAP_JET)

        # Highlight near-layer blob in bright green
        cmap[mask == 1] = [0, 220, 80]

        if detected:
            # Bounding box
            cv2.rectangle(cmap, (bx, by), (bx + bw, by + bh), (255, 255, 255), 2)
            # Centroid cross
            cx_i, cy_i = int(cx), int(cy)
            cv2.drawMarker(cmap, (cx_i, cy_i), (255, 200, 0), cv2.MARKER_CROSS, 20, 2)
            # Distance label
            cv2.putText(cmap, f'{dist_m:.2f} m', (bx + 4, by - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(cmap, 'HAND DETECTED', (8, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 80), 2, cv2.LINE_AA)
        else:
            cv2.putText(cmap, 'No detection', (8, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 100, 100), 1, cv2.LINE_AA)

        rgb_out = cv2.cvtColor(cmap, cv2.COLOR_BGR2RGB)
        out = self._bridge.cv2_to_imgmsg(rgb_out, encoding='rgb8')
        out.header.stamp = stamp
        out.header.frame_id = self._fid
        self._pub_img.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = HandDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
