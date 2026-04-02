#!/usr/bin/env python3
# Copyright 2024 ROS2 Community -- Apache-2.0

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from kinect_ros2.depth_utils import nearest_in_roi

_SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class DepthAlertNode(Node):
    def __init__(self):
        super().__init__('depth_alert_node')

        self.declare_parameter('danger_distance_m', 0.80)
        self.declare_parameter('warning_distance_m', 1.50)
        self.declare_parameter('min_valid_distance_m', 0.10)
        self.declare_parameter('alert_topic', '/kinect/proximity_alert')
        self.declare_parameter('roi_fraction', 0.5)

        p = self.get_parameter
        self._danger = p('danger_distance_m').value
        self._warning = p('warning_distance_m').value
        self._min_d = p('min_valid_distance_m').value
        self._roi = p('roi_fraction').value
        topic = p('alert_topic').value

        self._bridge = CvBridge()
        self._pub = self.create_publisher(String, topic, 10)
        self._sub = self.create_subscription(Image, '/camera/depth/image_meters', self._cb, _SENSOR_QOS)
        self._status = self.create_timer(10.0, self._heartbeat)

        self.get_logger().info(f'DepthAlertNode ready [DANGER<{self._danger}m, WARNING<{self._warning}m]')

    def _cb(self, msg: Image):
        try:
            depth_m = self._bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            dist, row, col = nearest_in_roi(depth_m, self._roi, self._min_d)

            if dist <= self._danger:
                text = f'DANGER: obstacle at {dist:.2f} m [px ({col},{row})] -- STOP IMMEDIATELY'
                self.get_logger().warn(text)
            elif dist <= self._warning:
                text = f'WARNING: obstacle at {dist:.2f} m [px ({col},{row})] -- slow down'
                self.get_logger().info(text)
            else:
                d_str = f'{dist:.2f} m' if dist != float('inf') else 'no return'
                text = f'CLEAR: nearest obstacle at {d_str}'

            out = String()
            out.data = text
            self._pub.publish(out)

        except Exception as exc:
            self.get_logger().error(f'Callback error: {exc}')

    def _heartbeat(self):
        self.get_logger().info('DepthAlertNode running.')


def main(args=None):
    rclpy.init(args=args)
    node = DepthAlertNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down DepthAlertNode.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
