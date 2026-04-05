#!/usr/bin/env python3
"""Example 07: Monitor accelerometer data from Kinect"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class AccelMonitorNode(Node):
    def __init__(self):
        super().__init__('accel_monitor_node')
        
        # Subscribe to IMU topic from driver node
        self.subscription = self.create_subscription(
            Imu,
            '/kinect/imu/data',
            self.imu_callback,
            10
        )
        
        self.get_logger().info('AccelMonitorNode started - listening for IMU data')
        self.get_logger().info('Press Ctrl+C to stop')
    
    def imu_callback(self, msg):
        acc = msg.linear_acceleration
        self.get_logger().info(
            f'Accelerometer - X: {acc.x:6.3f} m/s², '
            f'Y: {acc.y:6.3f} m/s², '
            f'Z: {acc.z:6.3f} m/s²'
        )

def main(args=None):
    rclpy.init(args=args)
    node = AccelMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down AccelMonitorNode.')
    finally:
        node.destroy_node()
        if rclpy.ok():  # Only shutdown if still OK
            rclpy.shutdown()

if __name__ == '__main__':
    main()