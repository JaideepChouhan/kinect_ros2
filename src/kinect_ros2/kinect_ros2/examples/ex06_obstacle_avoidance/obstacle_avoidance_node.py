#!/usr/bin/env python3
"""
Example 06: Reactive Obstacle Avoidance Publisher
===================================================
Divides the depth image into three sectors (left, centre, right).
Based on the nearest obstacle in each sector, it publishes a
geometry_msgs/Twist on /cmd_vel to steer the robot away.

State Machine
-------------
    FORWARD       : All sectors clear  → drive forward
    TURN_LEFT     : Centre/right blocked, left clear → turn left
    TURN_RIGHT    : Centre/left blocked, right clear → turn right
    STOP          : All sectors blocked              → stop

Published Topics
----------------
/cmd_vel            geometry_msgs/Twist   Velocity command
/kinect/avoidance/state   std_msgs/String Readable state label
/kinect/avoidance/sector_image  sensor_msgs/Image  Annotated debug image

Run with:
    ros2 run kinect_ros2 kinect_driver                 # Terminal 1
    ros2 run kinect_ros2 ex06_obstacle_avoidance       # Terminal 2
    ros2 topic echo /kinect/avoidance/state            # Terminal 3
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


class ObstacleAvoidanceNode(Node):
    _STATE_FORWARD    = 'FORWARD'
    _STATE_TURN_LEFT  = 'TURN_LEFT'
    _STATE_TURN_RIGHT = 'TURN_RIGHT'
    _STATE_STOP       = 'STOP'

    _COL = {
        'clear':   (0, 200, 80),
        'blocked': (0, 60,  200),
        'info':    (220, 220, 220),
        'state':   (247, 127, 0),
    }

    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        self.declare_parameter('danger_distance_m', 0.80)
        self.declare_parameter('linear_speed',      0.20)
        self.declare_parameter('angular_speed',     0.40)
        self.declare_parameter('sector_overlap',    0.10)
        self.declare_parameter('frame_id',  'kinect_depth_frame')

        self._danger  = self.get_parameter('danger_distance_m').value
        self._lin     = self.get_parameter('linear_speed').value
        self._ang     = self.get_parameter('angular_speed').value
        self._overlap = self.get_parameter('sector_overlap').value
        self._fid     = self.get_parameter('frame_id').value

        self._bridge = CvBridge()
        self._state  = self._STATE_STOP

        # Publishers
        self._pub_cmd   = self.create_publisher(Twist,  '/cmd_vel', 10)
        self._pub_state = self.create_publisher(String, '/kinect/avoidance/state', 10)
        self._pub_img   = self.create_publisher(Image,  '/kinect/avoidance/sector_image', 10)

        # QoS matching the camera driver
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.create_subscription(Image, '/camera/depth/image_meters', self._depth_cb, qos)

        self.get_logger().info(
            f'ObstacleAvoidanceNode ready | danger={self._danger} m | '
            f'v={self._lin} m/s | ω={self._ang} rad/s')

    def _depth_cb(self, msg: Image):
        depth_m = self._bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        h, w    = depth_m.shape
        stamp   = msg.header.stamp

        # Only use the middle vertical band (ignore floor and ceiling)
        row_start = int(h * 0.30)
        row_end   = int(h * 0.70)
        band      = depth_m[row_start:row_end, :]

        # Define three sectors with optional overlap
        ov   = int(w * self._overlap)
        segs = {
            'left':   band[:, :w // 3 + ov],
            'centre': band[:, w // 3 - ov: 2 * w // 3 + ov],
            'right':  band[:, 2 * w // 3 - ov:],
        }

        nearest = {}
        blocked = {}
        for name, seg in segs.items():
            valid = seg[(seg > 0.3) & (seg < 8.0)]
            nearest[name] = float(np.min(valid)) if len(valid) else float('inf')
            blocked[name] = nearest[name] < self._danger

        # State machine
        c_blk = blocked['centre']
        l_blk = blocked['left']
        r_blk = blocked['right']

        if not c_blk and not l_blk and not r_blk:
            self._state = self._STATE_FORWARD
        elif c_blk and not l_blk and r_blk:
            self._state = self._STATE_TURN_LEFT
        elif c_blk and l_blk and not r_blk:
            self._state = self._STATE_TURN_RIGHT
        elif not c_blk and l_blk and not r_blk:
            self._state = self._STATE_TURN_LEFT
        elif not c_blk and not l_blk and r_blk:
            self._state = self._STATE_TURN_RIGHT
        else:
            self._state = self._STATE_STOP

        # Publish Twist
        twist = Twist()
        if self._state == self._STATE_FORWARD:
            twist.linear.x  =  self._lin
            twist.angular.z =  0.0
        elif self._state == self._STATE_TURN_LEFT:
            twist.linear.x  =  0.0
            twist.angular.z =  self._ang
        elif self._state == self._STATE_TURN_RIGHT:
            twist.linear.x  =  0.0
            twist.angular.z = -self._ang
        else:  # STOP
            twist.linear.x  =  0.0
            twist.angular.z =  0.0
        self._pub_cmd.publish(twist)

        # Publish state string
        st_msg = String()
        st_msg.data = self._state
        self._pub_state.publish(st_msg)

        self.get_logger().info(
            f'[{self._state:12s}] '
            f'L={nearest["left"]:.2f}m  '
            f'C={nearest["centre"]:.2f}m  '
            f'R={nearest["right"]:.2f}m')

        # Publish annotated sector image
        self._publish_debug_img(depth_m, h, w, row_start, row_end,
                                nearest, blocked, stamp)

    def _publish_debug_img(self, depth_m, h, w, r0, r1,
                           nearest, blocked, stamp):
        clipped = np.clip(depth_m, 0.3, 4.0)
        normed  = 255 - ((clipped - 0.3) / 3.7 * 255).astype(np.uint8)
        normed[depth_m == 0] = 0
        cmap = cv2.applyColorMap(normed, cv2.COLORMAP_TURBO)

        # Draw sector lines
        ov   = int(w * self._overlap)
        lx1  = w // 3
        lx2  = 2 * w // 3
        cv2.line(cmap, (lx1, r0), (lx1, r1), (255, 255, 255), 2)
        cv2.line(cmap, (lx2, r0), (lx2, r1), (255, 255, 255), 2)

        # Sector labels
        positions = [('left',   w // 6), ('centre', w // 2), ('right', 5 * w // 6)]
        for name, xpos in positions:
            col   = self._COL['blocked'] if blocked[name] else self._COL['clear']
            label = f'{"BLK" if blocked[name] else "CLR"} {nearest[name]:.1f}m'
            cv2.putText(cmap, label, (xpos - 55, r0 - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2, cv2.LINE_AA)

        # State banner
        cv2.rectangle(cmap, (0, 0), (w, 34), (10, 20, 40), -1)
        state_col = {
            self._STATE_FORWARD:    (0, 200, 80),
            self._STATE_TURN_LEFT:  (0, 180, 247),
            self._STATE_TURN_RIGHT: (0, 127, 247),
            self._STATE_STOP:       (0, 60, 200),
        }.get(self._state, (200, 200, 200))
        cv2.putText(cmap, f'STATE: {self._state}', (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, state_col, 2, cv2.LINE_AA)

        rgb_out = cv2.cvtColor(cmap, cv2.COLOR_BGR2RGB)
        out = self._bridge.cv2_to_imgmsg(rgb_out, encoding='rgb8')
        out.header.stamp = stamp
        out.header.frame_id = self._fid
        self._pub_img.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
