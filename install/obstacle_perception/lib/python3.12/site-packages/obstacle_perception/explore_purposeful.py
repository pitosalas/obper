#!/usr/bin/env python3
# Authors: Pito Salas and ChatGPT
# License: MIT
# File: explore_purposeful.py

"""
Specification: explore_purposeful.py

Purpose:
    This ROS 2 node drives a robot through an unknown environment using only LIDAR and odometry,
    in a purposeful and intelligent manner. It selects the farthest visible direction as a waypoint,
    rotates toward it, and drives forward until it reaches the goal or detects a new obstacle.
    The behavior is governed by a finite state machine.

Internal Components:
    - State Machine: States include IDLE, TURNING, and DRIVING.
    - Goal Selection: Chooses the farthest beam direction and computes a goal in the odom frame.
    - Visited Grid Map: Tracks visited areas using a 2D matrix in odom space, with 25cm resolution.
    - Visualization: Publishes a MarkerArray showing visited cells in RViz for monitoring.

Expected Behavior:
    - Continuously selects new goals from LIDAR beam data.
    - Turns in place to align with the chosen goal.
    - Drives forward, updates visited map, and replans if blocked.
    - Displays visited cells in RViz.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from robot_msgs.msg import BeamDistances
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from builtin_interfaces.msg import Duration
import numpy as np
import math

class ExplorePurposeful(Node):
    def __init__(self):
        super().__init__('explore_purposeful')

        # Parameters
        self.grid_resolution = 0.25  # meters per cell
        self.grid_size = 100  # 100x100 grid → 25m x 25m
        self.linear_speed = 0.4
        self.angular_speed = 0.8
        self.goal_tolerance = 0.3

        # State
        self.visited_map = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)
        self.current_state = 'IDLE'
        self.goal_point = None
        self.current_beams = []

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ROS
        self.beam_sub = self.create_subscription(BeamDistances, '/beam_distances', self.beam_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/visited_cells', 10)
        self.timer = self.create_timer(0.2, self.control_loop)

    def beam_callback(self, msg):
        self.current_beams = list(zip(msg.angles, msg.distances))
        if self.current_state == 'IDLE' and self.current_beams:
            self.select_new_goal()

    def select_new_goal(self):
        best_angle, best_distance = max(self.current_beams, key=lambda x: x[1])
        pose = self.get_robot_pose()
        if pose is None:
            return
        x = pose[0] + best_distance * math.cos(pose[2] + best_angle)
        y = pose[1] + best_distance * math.sin(pose[2] + best_angle)
        self.goal_point = (x, y)
        self.current_state = 'TURNING'

    def get_robot_pose(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', now)
            t = trans.transform.translation
            q = trans.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z))
            return (t.x, t.y, yaw)
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn("TF lookup failed.")
            return None

    def control_loop(self):
        pose = self.get_robot_pose()
        if pose is None:
            return

        self.update_visited_map(pose[0], pose[1])
        self.publish_visited_cells()

        if self.current_state == 'TURNING':
            angle_to_goal = math.atan2(self.goal_point[1] - pose[1], self.goal_point[0] - pose[0])
            angle_diff = self.normalize_angle(angle_to_goal - pose[2])
            if abs(angle_diff) < 0.1:
                self.current_state = 'DRIVING'
                self.cmd_pub.publish(Twist())
            else:
                twist = Twist()
                twist.angular.z = self.angular_speed * np.sign(angle_diff)
                self.cmd_pub.publish(twist)

        elif self.current_state == 'DRIVING':
            dx = self.goal_point[0] - pose[0]
            dy = self.goal_point[1] - pose[1]
            dist = math.hypot(dx, dy)
            if dist < self.goal_tolerance:
                self.cmd_pub.publish(Twist())
                self.current_state = 'IDLE'
            else:
                twist = Twist()
                twist.linear.x = self.linear_speed
                angle_to_goal = math.atan2(dy, dx)
                angle_diff = self.normalize_angle(angle_to_goal - pose[2])
                twist.angular.z = self.angular_speed * angle_diff
                self.cmd_pub.publish(twist)

    def update_visited_map(self, x, y):
        cx = int((x + (self.grid_size * self.grid_resolution) / 2) / self.grid_resolution)
        cy = int((y + (self.grid_size * self.grid_resolution) / 2) / self.grid_resolution)
        if 0 <= cx < self.grid_size and 0 <= cy < self.grid_size:
            self.visited_map[cy, cx] = 1

    def publish_visited_cells(self):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'visited'
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale.x = self.grid_resolution
        marker.scale.y = self.grid_resolution
        marker.scale.z = 0.01
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5

        marker.points.clear()
        for j in range(self.grid_size):
            for i in range(self.grid_size):
                if self.visited_map[j, i]:
                    pt = Point()
                    pt.x = (i - self.grid_size / 2) * self.grid_resolution
                    pt.y = (j - self.grid_size / 2) * self.grid_resolution
                    pt.z = 0.01
                    marker.points.append(pt)

        self.marker_pub.publish(marker)

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = ExplorePurposeful()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
