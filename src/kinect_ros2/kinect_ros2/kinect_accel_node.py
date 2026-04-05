#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import freenect

class KinectAccelNode(Node):
    def __init__(self):
        super().__init__('kinect_accel_node')
        self.imu_pub = self.create_publisher(Imu, '/kinect/imu/data', 10)
        
        # Timer for IMU reading (50 Hz)
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        # Device index
        self.device_index = 0
        
        self.get_logger().info('KinectAccelNode started - standalone IMU reader')
        self.get_logger().warning('Make sure no other node is accessing the Kinect!')
    
    def timer_callback(self):
        try:
            # Try to get accelerometer data
            if hasattr(freenect, 'sync_get_accel'):
                result = freenect.sync_get_accel(self.device_index)
                if result and len(result) == 3:
                    ax, ay, az = result
                    
                    imu_msg = Imu()
                    imu_msg.header.stamp = self.get_clock().now().to_msg()
                    imu_msg.header.frame_id = 'kinect_accel_link'
                    imu_msg.linear_acceleration.x = float(ax)
                    imu_msg.linear_acceleration.y = float(ay)
                    imu_msg.linear_acceleration.z = float(az)
                    
                    # Set covariance
                    imu_msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 
                                                               0.0, 0.01, 0.0, 
                                                               0.0, 0.0, 0.01]
                    
                    self.imu_pub.publish(imu_msg)
                    self.get_logger().debug(f'Published IMU: x={ax:.2f}, y={ay:.2f}, z={az:.2f}')
                    
        except Exception as e:
            self.get_logger().debug(f'IMU read error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = KinectAccelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down KinectAccelNode')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()