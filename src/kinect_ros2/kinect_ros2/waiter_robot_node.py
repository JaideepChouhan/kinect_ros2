#!/usr/bin/env python3
# Copyright 2024 ROS2 Community -- Apache-2.0
"""
waiter_robot_node.py  –  Autonomous waiter-robot mission controller.

OVERVIEW
─────────
This node turns your Kinect-nav2 robot into a restaurant waiter.
It maintains a named list of table waypoints, accepts delivery orders,
navigates to each table in sequence, waits for confirmation, then
returns to the home/kitchen position.

ARCHITECTURE
─────────────
  ┌─────────────────────────────────────────────────┐
  │              waiter_robot_node                  │
  │                                                 │
  │  /waiter/order  (WaiterOrder)  ←── operator UI │
  │  /waiter/status (WaiterStatus) ──→ operator UI │
  │                                                 │
  │  navigate_to_pose action client ──→ Nav2        │
  │  waypoint_follower action client ──→ Nav2       │
  └─────────────────────────────────────────────────┘

TOPICS
───────
  Subscribed:
    /waiter/order    std_msgs/String
        JSON payload:  {"tables": [1, 3, 2]}
        Tables are visited in the given order.
        Special name "home" returns to kitchen.

  Published:
    /waiter/status   std_msgs/String
        JSON payload:  {"state": "NAVIGATING", "target": "table_2",
                        "queue": [3]}

PARAMETERS
───────────
  tables_json   str   JSON dict of table poses, e.g.:
                      '{"home":  {"x":0.0,"y":0.0,"yaw":0.0},
                        "table_1":{"x":2.5,"y":0.5,"yaw":1.57},
                        "table_2":{"x":2.5,"y":-1.0,"yaw":1.57},
                        "table_3":{"x":4.0,"y":0.0,"yaw":0.0}}'
  wait_at_table_sec   float   5.0   Seconds to pause at each table
  return_home         bool    true  Return to home after all deliveries

HOW TO ADD TABLES
──────────────────
1. Build your map with slam_toolbox.
2. In RViz2, use "2D Nav Goal" to drive the robot to each table.
   Read the /amcl_pose topic to get the (x, y, yaw) for each table.
3. Edit the tables_json parameter in your launch file or params YAML.

HOW TO SEND AN ORDER
─────────────────────
  ros2 topic pub --once /waiter/order std_msgs/String \
      'data: "{\"tables\": [1, 2]}"'

  Or use the companion waiter_ui.py script.
"""

import json
import math
import time
from collections import deque
from typing import Any

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert a yaw angle (radians) to a geometry_msgs/Quaternion."""
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


# ── Default table layout (metres, in map frame) ──────────────────────────────
# These are example positions.  Measure yours from your actual map.
_DEFAULT_TABLES = {
    "home":    {"x": 0.0,  "y":  0.0,  "yaw": 0.0},
    "table_1": {"x": 2.5,  "y":  0.5,  "yaw": 1.57},
    "table_2": {"x": 2.5,  "y": -1.0,  "yaw": 1.57},
    "table_3": {"x": 4.0,  "y":  0.0,  "yaw": 0.0},
    "table_4": {"x": 4.0,  "y": -1.5,  "yaw": 0.0},
}

_STATES = ('IDLE', 'NAVIGATING', 'WAITING_AT_TABLE', 'RETURNING_HOME', 'ERROR')


class WaiterRobotNode(Node):
    """Waiter robot mission controller."""

    def __init__(self) -> None:
        super().__init__('waiter_robot_node')

        # ── Parameters ─────────────────────────────────────────────────────
        self.declare_parameter('tables_json',       json.dumps(_DEFAULT_TABLES))
        self.declare_parameter('wait_at_table_sec', 5.0)
        self.declare_parameter('return_home',       True)
        self.declare_parameter('map_frame',         'map')

        tables_str          = self.get_parameter('tables_json').value
        self._wait_sec      = self.get_parameter('wait_at_table_sec').value
        self._return_home   = self.get_parameter('return_home').value
        self._map_frame     = self.get_parameter('map_frame').value

        try:
            self._tables: dict[str, dict] = json.loads(tables_str)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid tables_json parameter: {exc}')
            self._tables = _DEFAULT_TABLES

        # ── State ───────────────────────────────────────────────────────────
        self._state        = 'IDLE'
        self._queue: deque = deque()
        self._current      = ''
        self._nav_future   = None

        # ── Action client (Nav2) ────────────────────────────────────────────
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ── Topics ──────────────────────────────────────────────────────────
        latch_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_pub = self.create_publisher(String, '/waiter/status', latch_qos)
        self.create_subscription(String, '/waiter/order', self._order_cb, 10)

        # ── Periodic update loop ────────────────────────────────────────────
        self._timer = self.create_timer(0.5, self._loop)

        self.get_logger().info(
            f'WaiterRobotNode ready.  Known tables: {list(self._tables.keys())}'
        )
        self._publish_status()

    # ── Order callback ────────────────────────────────────────────────────────

    def _order_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f'Bad order JSON: {msg.data}')
            return

        if self._state != 'IDLE':
            self.get_logger().warn(
                f'Order received but robot is busy ({self._state}). Ignored.'
            )
            return

        tables = data.get('tables', [])
        names  = [f'table_{t}' if isinstance(t, int) else t for t in tables]
        valid  = [n for n in names if n in self._tables]
        invalid = set(names) - set(valid)
        if invalid:
            self.get_logger().warn(f'Unknown table names ignored: {invalid}')

        if not valid:
            self.get_logger().error('Order has no valid table targets.')
            return

        self._queue = deque(valid)
        self.get_logger().info(f'New order received. Queue: {list(self._queue)}')
        self._publish_status()

    # ── Main control loop ─────────────────────────────────────────────────────

    def _loop(self) -> None:
        if self._state == 'IDLE':
            if self._queue:
                self._go_next()
            return

        if self._state == 'NAVIGATING':
            # Navigation is handled asynchronously via action callbacks
            return

        if self._state == 'WAITING_AT_TABLE':
            return  # handled by timer callback set in _arrived_at_table

        if self._state == 'RETURNING_HOME':
            return  # handled asynchronously

        if self._state == 'ERROR':
            self.get_logger().warn('In ERROR state. Waiting for new order.', throttle_duration_sec=10.0)

    # ── Navigation helpers ────────────────────────────────────────────────────

    def _go_next(self) -> None:
        target = self._queue.popleft()
        self._navigate_to(target)

    def _navigate_to(self, name: str) -> None:
        pose_data = self._tables.get(name)
        if pose_data is None:
            self.get_logger().error(f'Table not found: {name}')
            self._state = 'ERROR'
            self._publish_status()
            return

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_to_pose action server not available!')
            self._state = 'ERROR'
            self._publish_status()
            return

        goal_pose = PoseStamped()
        goal_pose.header.frame_id    = self._map_frame
        goal_pose.header.stamp       = self.get_clock().now().to_msg()
        goal_pose.pose.position.x    = float(pose_data['x'])
        goal_pose.pose.position.y    = float(pose_data['y'])
        goal_pose.pose.orientation   = _yaw_to_quaternion(float(pose_data.get('yaw', 0.0)))

        goal_msg          = NavigateToPose.Goal()
        goal_msg.pose     = goal_pose

        self._current = name
        self._state   = 'NAVIGATING'
        self.get_logger().info(
            f'Navigating to {name}  ({pose_data["x"]:.2f}, {pose_data["y"]:.2f})'
        )
        self._publish_status()

        self._nav_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._nav_feedback_cb,
        )
        self._nav_future.add_done_callback(self._nav_goal_accepted_cb)

    def _nav_goal_accepted_cb(self, future: Any) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Goal to {self._current} was REJECTED by Nav2.')
            self._state = 'ERROR'
            self._publish_status()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_feedback_cb(self, feedback_msg: Any) -> None:
        # Optionally log distance remaining
        fb = feedback_msg.feedback
        dist = getattr(fb, 'distance_remaining', None)
        if dist is not None:
            self.get_logger().debug(f'  → {self._current}: {dist:.2f} m remaining')

    def _nav_result_cb(self, future: Any) -> None:
        result = future.result()
        status = result.status

        # action_msgs/GoalStatus: 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
        if status == 4:
            self.get_logger().info(f'Arrived at {self._current}!')
            self._arrived_at_table()
        else:
            self.get_logger().error(
                f'Navigation to {self._current} failed (status={status}). '
                'Clearing queue and going home.'
            )
            self._queue.clear()
            self._state = 'ERROR'
            self._publish_status()
            if self._return_home:
                self._go_home()

    def _arrived_at_table(self) -> None:
        self._state = 'WAITING_AT_TABLE'
        self._publish_status()
        self.get_logger().info(
            f'Waiting {self._wait_sec:.0f}s at {self._current}…'
        )
        # Use a one-shot timer to continue after the pause
        self._wait_timer = self.create_timer(self._wait_sec, self._done_waiting)

    def _done_waiting(self) -> None:
        self._wait_timer.cancel()
        self._wait_timer.destroy()
        if self._queue:
            self.get_logger().info(
                f'Leaving {self._current}. Next: {self._queue[0]}'
            )
            self._state = 'IDLE'
            self._go_next()
        else:
            self.get_logger().info('All tables served!')
            if self._return_home:
                self._go_home()
            else:
                self._state = 'IDLE'
                self._publish_status()

    def _go_home(self) -> None:
        self._state = 'RETURNING_HOME'
        self._publish_status()
        self._queue = deque()
        if 'home' in self._tables:
            self._navigate_to('home')
        else:
            self.get_logger().warn('No "home" position defined. Staying put.')
            self._state = 'IDLE'
            self._publish_status()

    # ── Status publisher ──────────────────────────────────────────────────────

    def _publish_status(self) -> None:
        payload = {
            'state':   self._state,
            'target':  self._current,
            'queue':   list(self._queue),
            'tables':  list(self._tables.keys()),
        }
        msg      = String()
        msg.data = json.dumps(payload)
        self._status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaiterRobotNode()
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
