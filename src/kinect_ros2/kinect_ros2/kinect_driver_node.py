#!/usr/bin/env python3
# Copyright 2024 ROS2 Community -- Apache-2.0

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, Imu
from std_msgs.msg import Header
from cv_bridge import CvBridge

try:
    import freenect
    _FREENECT_OK = True
except ImportError:
    _FREENECT_OK = False

from kinect_ros2.depth_utils import raw_to_meters, make_camera_info_dict

_SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class KinectDriverNode(Node):
    def __init__(self):
        super().__init__('kinect_driver_node')

        if not _FREENECT_OK:
            self.get_logger().fatal('freenect Python module not found. Install with: pip3 install freenect')
            raise RuntimeError('freenect unavailable')

        # Declare parameters
        self.declare_parameter('device_index', 0)
        self.declare_parameter('target_fps', 30.0)
        self.declare_parameter('rgb_frame_id', 'camera_rgb_optical_frame')
        self.declare_parameter('depth_frame_id', 'camera_depth_optical_frame')
        self.declare_parameter('publish_rgb', True)
        self.declare_parameter('publish_depth', True)
        self.declare_parameter('publish_imu', True)
        self.declare_parameter('depth_calib_c1', -0.0030711016)
        self.declare_parameter('depth_calib_c2', 3.3309495161)
        self.declare_parameter('rgb_fx', 525.0)
        self.declare_parameter('rgb_fy', 525.0)
        self.declare_parameter('rgb_cx', 319.5)
        self.declare_parameter('rgb_cy', 239.5)
        self.declare_parameter('depth_fx', 580.0)
        self.declare_parameter('depth_fy', 580.0)
        self.declare_parameter('depth_cx', 319.5)
        self.declare_parameter('depth_cy', 239.5)

        # Read parameters
        p = self.get_parameter
        self._dev = p('device_index').value
        fps = p('target_fps').value
        self._rgb_fid = p('rgb_frame_id').value
        self._dep_fid = p('depth_frame_id').value
        self._pub_rgb = p('publish_rgb').value
        self._pub_dep = p('publish_depth').value
        self._pub_imu = p('publish_imu').value
        self._c1 = p('depth_calib_c1').value
        self._c2 = p('depth_calib_c2').value

        # State
        self._bridge = CvBridge()
        self._frames = 0
        self._t0 = time.time()

        # Publishers
        self._pub_rgb_img = self.create_publisher(Image, '/camera/rgb/image_raw', _SENSOR_QOS)
        self._pub_rgb_ci = self.create_publisher(CameraInfo, '/camera/rgb/camera_info', _SENSOR_QOS)
        self._pub_dep_raw = self.create_publisher(Image, '/camera/depth/image_raw', _SENSOR_QOS)
        self._pub_dep_m = self.create_publisher(Image, '/camera/depth/image_meters', _SENSOR_QOS)
        self._pub_dep_ci = self.create_publisher(CameraInfo, '/camera/depth/camera_info', _SENSOR_QOS)
        
        # IMU publisher
        if self._pub_imu:
            self._pub_imu_data = self.create_publisher(Imu, '/kinect/imu/data', 10)

        # Timer for main capture
        self._timer = self.create_timer(1.0 / fps, self._capture)

        self.get_logger().info(f'KinectDriverNode started [device={self._dev}, target={fps:.0f} Hz, IMU={self._pub_imu}]')

    def _build_ci(self, info: dict) -> CameraInfo:
        msg = CameraInfo()
        msg.header.stamp = info['stamp']
        msg.header.frame_id = info['frame_id']
        msg.width = info['width']
        msg.height = info['height']
        msg.distortion_model = info['distortion_model']
        msg.d = info['d']
        msg.k = info['k']
        msg.r = info['r']
        msg.p = info['p']
        return msg

    def _param(self, name):
        return self.get_parameter(name).value

    def _publish_imu(self, stamp):
        if not self._pub_imu:
            return
            
        try:
            if hasattr(freenect, 'sync_get_accel'):
                result = freenect.sync_get_accel(self._dev)
                self.get_logger().debug(f'sync_get_accel returned: {result}')
                if result is not None and len(result) == 3:
                    ax, ay, az = result
                    # ... create and publish IMU message ...
                    self._pub_imu_data.publish(imu_msg)
                    if self._frames % 100 == 0:
                        self.get_logger().info(f'IMU published: x={ax:.2f} y={ay:.2f} z={az:.2f}')
                else:
                    if self._frames % 100 == 0:
                        self.get_logger().warn('sync_get_accel returned None or invalid')
            else:
                if self._frames % 100 == 0:
                    self.get_logger().warn('sync_get_accel not available')
        except Exception as e:
            if self._frames % 100 == 0:
                self.get_logger().error(f'IMU exception: {e}')
                
    def _capture(self):
        stamp = self.get_clock().now().to_msg()
        
        # Publish IMU data
        self._publish_imu(stamp)

        # RGB
        if self._pub_rgb:
            try:
                rgb, _ = freenect.sync_get_video(self._dev)
                msg = self._bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
                msg.header = Header(stamp=stamp, frame_id=self._rgb_fid)
                self._pub_rgb_img.publish(msg)

                ci = make_camera_info_dict(
                    stamp, self._rgb_fid, 640, 480,
                    self._param('rgb_fx'), self._param('rgb_fy'),
                    self._param('rgb_cx'), self._param('rgb_cy'),
                    dist=[0.2624, -0.9531, -0.0054, 0.0026, 1.1633],
                )
                self._pub_rgb_ci.publish(self._build_ci(ci))
            except Exception as exc:
                self.get_logger().warn(f'RGB capture failed: {exc}')

        # Depth
        if self._pub_dep:
            try:
                raw, _ = freenect.sync_get_depth(self._dev)

                raw_msg = self._bridge.cv2_to_imgmsg(raw, encoding='16UC1')
                raw_msg.header = Header(stamp=stamp, frame_id=self._dep_fid)
                self._pub_dep_raw.publish(raw_msg)

                depth_m = raw_to_meters(raw, self._c1, self._c2)
                m_msg = self._bridge.cv2_to_imgmsg(depth_m, encoding='32FC1')
                m_msg.header = Header(stamp=stamp, frame_id=self._dep_fid)
                self._pub_dep_m.publish(m_msg)

                ci = make_camera_info_dict(
                    stamp, self._dep_fid, 640, 480,
                    self._param('depth_fx'), self._param('depth_fy'),
                    self._param('depth_cx'), self._param('depth_cy'),
                )
                self._pub_dep_ci.publish(self._build_ci(ci))
            except Exception as exc:
                self.get_logger().warn(f'Depth capture failed: {exc}')

        self._frames += 1
        if self._frames % 300 == 0:
            fps = self._frames / (time.time() - self._t0)
            self.get_logger().info(f'Running at {fps:.1f} Hz ({self._frames} frames)')


def main(args=None):
    rclpy.init(args=args)
    node = KinectDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down KinectDriverNode.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()