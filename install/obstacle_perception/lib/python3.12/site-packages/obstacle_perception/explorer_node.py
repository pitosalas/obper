#!/usr/bin/env python3
# Authors: Pito Salas and ChatGPT
# License: MIT
# File: explorer_node.py

import rclpy
from rclpy.node import Node
from robot_msgs.msg import BeamDistances
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration
from collections import deque
import math

class ExplorerNode(Node):
    def __init__(self):
        super().__init__("explorer_node")

        # Parameters
        self.clearance_threshold = 0.6
        self.stop_threshold = 0.4
        self.linear_speed = 0.6
        self.angular_gain = 1.5
        self.max_range = 2.0

        # State
        self.current_beams = []
        self.recent_positions = deque(maxlen=30)
        self.current_direction = None

        # Publishers and subscribers
        self.beam_sub = self.create_subscription(BeamDistances, "/beam_distances", self.beam_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.marker_pub = self.create_publisher(Marker, "/explorer_direction_marker", 10)

        # Timer
        self.timer = self.create_timer(0.2, self.control_loop)

    def beam_callback(self, msg):
        self.current_beams = list(zip(msg.angles, msg.distances))

    def control_loop(self):
        if not self.current_beams:
            self.get_logger().info("Waiting for beam data...")
            return

        best_score = float("-inf")
        best_direction = None

        for angle, dist in self.current_beams:
            score = self.score_beam(angle, dist)
            if score > best_score:
                best_score = score
                best_direction = angle

        if best_direction is not None:
            self.current_direction = best_direction
            self.publish_cmd(best_direction)
            self.publish_direction_marker(best_direction)
            self.remember_position(best_direction)
        else:
            self.get_logger().warn("No viable direction — stopping.")
            self.stop()

    def score_beam(self, angle, distance):
        if distance < self.clearance_threshold:
            return -float('inf')  # Blocked

        # Bias for forward motion
        forward_bonus = max(0, 1.0 - abs(angle))

        # Penalty if it's revisiting a recent location
        predicted_x = self.max_range * math.cos(angle)
        predicted_y = self.max_range * math.sin(angle)
        revisit_penalty = 0.5 if self.is_near_recent(predicted_x, predicted_y) else 0.0

        return distance + 0.5 * forward_bonus - revisit_penalty

    def is_near_recent(self, x, y, threshold=0.6):
        for rx, ry in self.recent_positions:
            if math.hypot(x - rx, y - ry) < threshold:
                return True
        return False

    def remember_position(self, angle):
        x = self.max_range * math.cos(angle)
        y = self.max_range * math.sin(angle)
        self.recent_positions.append((x, y))

    def publish_cmd(self, angle):
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_gain * angle
        self.cmd_pub.publish(twist)
        self.get_logger().info(
            f"Driving direction: {angle:.2f} rad, linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f}"
        )

    def stop(self):
        self.cmd_pub.publish(Twist())

    def publish_direction_marker(self, angle):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "explorer"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.005
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.lifetime = Duration(sec=1)

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        start = Point(x=0.0, y=0.0, z=0.1)
        end = Point(x=self.max_range * math.cos(angle), y=self.max_range * math.sin(angle), z=0.1)
        marker.points = [start, end]

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = ExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()