#!/usr/bin/env python3
# Authors: Pito Salas and ChatGPT
# License: MIT
# File: explore_purposeful.py
# Version: 1.2
# Last revised: 2025-04-24

"""
Purpose:
    This ROS 2 node drives a robot through an unknown environment using only LIDAR and odometry,
    in a purposeful and intelligent manner. It selects the farthest visible direction as a waypoint,
    rotates toward it, and drives forward until it reaches the goal or detects a new obstacle.
    The behavior is governed by a finite state machine.

Key Features:
    - State Machine: IDLE, TURN, DRIV
    - Goal Selection: Pick farthest visible beam
    - Visited Map: 2D grid in odom frame
    - Markers: Visited cells + goal point
    - CSV-style logging for each control loop
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from robot_msgs.msg import BeamDistances
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import numpy as np
import math
import random


class Explore2(Node):
    def __init__(self):
        super().__init__("explore_purposeful")

        # Parameters
        self.grid_resolution = 0.25
        self.grid_size = 100
        self.linear_speed = 0.3
        self.angular_speed = 1.5
        self.goal_tolerance = 0.3

        # State
        self.visited_map = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)
        self.set_state("IDLE")
        self.goal_point = None
        self.current_beams = []
        self.loop_count = 0

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ROS interfaces
        self.beam_sub = self.create_subscription(
            BeamDistances, "/beam_distances", self.beam_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/visited_cells", 10)
        self.timer = self.create_timer(0.2, self.control_loop)  # 5 Hz
        self.cmd_timer = self.create_timer(0.05, self.publish_current_twist)  # 20 Hz

        self.current_twist = Twist()

    def beam_callback(self, msg):
        self.get_logger().debug(f"State: {self.current_state}")
        self.current_beams = list(zip(msg.angles, msg.distances))
        if self.current_state == "IDLE" and self.current_beams:
            self.select_new_goal()

    def select_new_goal(self):
        beams = sorted(self.current_beams, key=lambda x: x[1], reverse=True)
        candidates = beams[:5]
        best_angle, best_distance = random.choice(candidates)

        pose = self.get_robot_pose()
        if pose is None:
            return

        x = pose[0] + best_distance * math.cos(pose[2] + best_angle)
        y = pose[1] + best_distance * math.sin(pose[2] + best_angle)
        self.goal_point = (x, y)
        self.set_state("TURN")
        self.get_logger().debug(
            f"New goal: ({x:.2f}, {y:.2f}) from angle={math.degrees(best_angle):.1f}°, distance={best_distance:.2f}"
        )

    def set_state(self, state):
        if state not in ["IDLE", "TURN", "DRIV"]:
            self.get_logger().debug(f"Invalid state: {state}")
            return
        self.current_state = state
        self.get_logger().debug(f"State changed to: {self.current_state}")

    def get_robot_pose(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform("odom", "base_link", now)
            t = trans.transform.translation
            q = trans.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            return (t.x, t.y, yaw)
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().debug(f"TF lookup failed: {type(e).__name__}")
            return None

    def publish_twist(self, linear_x, angular_z):
        self.current_twist.linear.x = linear_x
        self.current_twist.angular.z = angular_z
        self.get_logger().debug(
            f"Set cmd_vel: linear.x={linear_x:.2f}, angular.z={angular_z:.2f}"
        )

    def publish_current_twist(self):
        self.cmd_pub.publish(self.current_twist)

    def control_loop(self):
        pose = self.get_robot_pose()
        if pose is None:
            return

        self.loop_count += 1
        self.update_visited_map(pose[0], pose[1])
        self.publish_visited_cells()

        if self.current_state == "TURN":
            angle_to_goal = math.atan2(
                self.goal_point[1] - pose[1], self.goal_point[0] - pose[0]
            )
            angle_diff = self.normalize_angle(angle_to_goal - pose[2])
            if abs(angle_diff) < 0.1:
                self.set_state("DRIV")
                self.publish_twist(0.0, 0.0)
            else:
                self.publish_twist(0.0, self.angular_speed * np.sign(angle_diff))

        elif self.current_state == "DRIV":
            dx = self.goal_point[0] - pose[0]
            dy = self.goal_point[1] - pose[1]
            dist = math.hypot(dx, dy)
            if dist < self.goal_tolerance:
                self.publish_twist(0.0, 0.0)
                self.set_state("IDLE")
            else:
                angle_to_goal = math.atan2(dy, dx)
                angle_diff = self.normalize_angle(angle_to_goal - pose[2])
                ang_z = np.clip(self.angular_speed * angle_diff, -1.0, 1.0)
                self.publish_twist(self.linear_speed, ang_z)
        self.log_loop_data(pose)

    def log_loop_data(self, pose):
        minf = float("-inf")
        lx = self.current_twist.linear.x
        az = self.current_twist.angular.z
        px, py = pose[0], pose[1]
        gx, gy = self.goal_point if self.goal_point else (minf, minf)
        current_beam_as_str = ",".join(
            f"{a:.1f}:{d:.1f}" for a, d in self.current_beams
        )
        dist = math.hypot(gx - px, gy - py) if self.goal_point else minf
        self.get_logger().info(
            f"{self.loop_count:3d},{self.current_state},{lx:.1f},{az:.1f}, {px:.2f},{py:.2f},{gx:.1f},{gy:.1f},{dist:.1f},[{current_beam_as_str}]"
        )

    def create_marker(
        self, ns, marker_id, marker_type, r, g, b, a, scale_x, scale_y, scale_z
    ):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.scale.x = scale_x
        marker.scale.y = scale_y
        marker.scale.z = scale_z
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a
        return marker

    def update_visited_map(self, x, y):
        cx = int(
            (x + (self.grid_size * self.grid_resolution) / 2) / self.grid_resolution
        )
        cy = int(
            (y + (self.grid_size * self.grid_resolution) / 2) / self.grid_resolution
        )
        if 0 <= cx < self.grid_size and 0 <= cy < self.grid_size:
            self.visited_map[cy, cx] = 1

    def publish_visited_cells(self):
        marker_array = MarkerArray()

        # Visited cell cubes
        marker = self.create_marker(
            "visited",
            0,
            Marker.CUBE_LIST,
            0.0,
            1.0,
            0.0,
            1.0,
            self.grid_resolution,
            self.grid_resolution,
            0.1,
        )
        marker.points.clear()

        # for j in range(self.grid_size):
        #     for i in range(self.grid_size):
        #         if self.visited_map[j, i]:
        #             pt = Point()
        #             pt.x = (i - self.grid_size / 2) * self.grid_resolution
        #             pt.y = (j - self.grid_size / 2) * self.grid_resolution
        #             pt.z = 0.01
        #             marker.points.append(pt)

        # marker_array.markers.append(marker)

        # Goal marker
        if self.goal_point:
            goal_marker = self.create_marker(
                "goal", 1, Marker.SPHERE, 1.0, 0.0, 0.0, 1.0, 0.2, 0.2, 0.05
            )
            goal_marker.pose.position.x = self.goal_point[0]
            goal_marker.pose.position.y = self.goal_point[1]
            goal_marker.pose.position.z = 0.05
            marker_array.markers.append(goal_marker)

        self.marker_pub.publish(marker_array)

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = Explore2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
