#!/usr/bin/env python3
"""
Example 05: Colourised RGB-D Point Cloud
==========================================
Fuses /camera/depth/image_meters and /camera/rgb/image_raw to produce
an XYZRGB PointCloud2 message on /kinect/depth/points_colored.

Run with:
    ros2 run kinect_ros2 kinect_driver                 # Terminal 1
    ros2 run kinect_ros2 ex05_coloured_cloud          # Terminal 2

In RViz2:
    Fixed Frame → kinect_depth_frame
    Add PointCloud2 → /kinect/depth/points_colored
    Colour Transformer → RGB8
"""

import struct
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer

from kinect_ros2.depth_utils import raw_to_meters  # not used directly but kept for reference

# Depth camera intrinsics (Kinect v1)
DEPTH_WIDTH = 640
DEPTH_HEIGHT = 480
DEPTH_FX = 580.0
DEPTH_FY = 580.0
DEPTH_CX = 320.0
DEPTH_CY = 240.0


class ColouredCloudNode(Node):
    def __init__(self):
        super().__init__('coloured_cloud_node')

        self.declare_parameter('frame_id', 'kinect_depth_frame')
        self.declare_parameter('slop', 0.05)   # sync tolerance seconds
        self.declare_parameter('skip', 2)      # process every N-th pair

        self._fid  = self.get_parameter('frame_id').value
        self._slop = self.get_parameter('slop').value
        self._skip = self.get_parameter('skip').value
        self._skip_counter = 0

        self._bridge = CvBridge()

        # Publisher
        self._pub = self.create_publisher(
            PointCloud2, '/kinect/depth/points_colored', 5)

        # QoS for camera topics (BEST_EFFORT)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Synchronised subscribers
        self._sub_depth = Subscriber(self, Image, '/camera/depth/image_meters', qos_profile=qos)
        self._sub_rgb   = Subscriber(self, Image, '/camera/rgb/image_raw', qos_profile=qos)

        self._sync = ApproximateTimeSynchronizer(
            [self._sub_depth, self._sub_rgb],
            queue_size=5,
            slop=self._slop
        )
        self._sync.registerCallback(self._sync_cb)

        # Pre-compute pixel coordinate grids (for speed)
        rows, cols = np.mgrid[0:DEPTH_HEIGHT, 0:DEPTH_WIDTH]
        self._cols = cols.astype(np.float32)
        self._rows = rows.astype(np.float32)

        self.get_logger().info(
            f'ColouredCloudNode ready | sync_slop={self._slop} s | skip={self._skip}')

    def _sync_cb(self, depth_msg: Image, rgb_msg: Image):
        self._skip_counter += 1
        if self._skip_counter < self._skip:
            return
        self._skip_counter = 0

        depth_m = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        rgb     = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='rgb8')

        cloud_msg = self._build_xyzrgb(depth_m, rgb, depth_msg.header.stamp)
        self._pub.publish(cloud_msg)

    def _build_xyzrgb(self, depth_m: np.ndarray,
                      rgb: np.ndarray,
                      stamp) -> PointCloud2:
        """
        Back-project depth image to 3-D and attach per-pixel RGB colour.
        Point step = 16 bytes (3 floats + 1 float that packs RGB).
        """
        valid = depth_m > 0.0

        z = depth_m[valid]
        x = (self._cols[valid] - DEPTH_CX) * z / DEPTH_FX
        y = (self._rows[valid] - DEPTH_CY) * z / DEPTH_FY

        # Extract RGB for valid pixels
        r_ch = rgb[valid, 0].astype(np.uint32)
        g_ch = rgb[valid, 1].astype(np.uint32)
        b_ch = rgb[valid, 2].astype(np.uint32)

        # Pack RGB into a single float32 (PCL convention)
        rgb_packed = (r_ch << 16) | (g_ch << 8) | b_ch
        rgb_float  = rgb_packed.view(np.float32)

        # Stack into (N, 4) array: x, y, z, rgb
        n = z.shape[0]
        data = np.zeros((n, 4), dtype=np.float32)
        data[:, 0] = x
        data[:, 1] = y
        data[:, 2] = z
        data[:, 3] = rgb_float

        # Build PointCloud2 message
        fields = [
            PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        cloud = PointCloud2()
        cloud.header.stamp    = stamp
        cloud.header.frame_id = self._fid
        cloud.height          = 1
        cloud.width           = n
        cloud.fields          = fields
        cloud.is_bigendian    = False
        cloud.point_step      = 16
        cloud.row_step        = 16 * n
        cloud.data            = data.tobytes()
        cloud.is_dense        = True

        return cloud


def main(args=None):
    rclpy.init(args=args)
    node = ColouredCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
