#!/usr/bin/env python3
# Authors: Pito Salas and ChatGPT

import rclpy
from rclpy.node import Node
from robot_msgs.msg import BeamDistances
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
import random
import math
from builtin_interfaces.msg import Duration


class ExplorerNode(Node):
    def __init__(self):
        super().__init__("explorer_node")

        # Parameters
        self.clearance_threshold = 0.8  # meters
        self.stop_threshold = 0.5  # meters
        self.linear_speed = 1.0  # m/s ← updated here
        self.angular_scale = 1.5  # rad/s per radian
        self.max_range = 2.0  # for visualization

        # State
        self.current_beams = None
        self.current_direction = None

        # ROS interfaces
        self.beam_sub = self.create_subscription(
            BeamDistances, "/beam_distances", self.beam_callback, 10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.marker_pub = self.create_publisher(
            Marker, "/explorer_direction_marker", 10
        )
        self.timer = self.create_timer(0.2, self.control_loop)

    def beam_callback(self, msg):
        if len(msg.angles) != len(msg.distances):
            self.get_logger().warn("Mismatched beam data lengths.")
            return
        self.current_beams = list(zip(msg.angles, msg.distances))

    def pick_random_direction(self):
        if not self.current_beams:
            return None
        candidates = [
            angle
            for angle, dist in self.current_beams
            if dist > self.clearance_threshold
        ]
        if not candidates:
            return None
        return random.choice(candidates)

    def control_loop(self):
        if self.current_beams is None:
            self.get_logger().info("Waiting for beam data...")
            return

        if self.current_direction is not None:
            for angle, dist in self.current_beams:
                if (
                    abs(angle - self.current_direction) < 0.2
                    and dist < self.stop_threshold
                ):
                    self.get_logger().info("Obstacle ahead — stopping.")
                    self.stop()
                    self.current_direction = None
                    return

            self.publish_cmd(self.current_direction)
            self.publish_direction_marker(self.current_direction)
        else:
            direction = self.pick_random_direction()
            if direction is None:
                self.get_logger().warn("No clear directions available!")
                self.stop()
                return

            self.current_direction = direction
            self.get_logger().info(f"New direction: {direction:.2f} rad")
            self.publish_cmd(direction)
            self.publish_direction_marker(direction)

    def publish_cmd(self, angle):
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_scale * angle
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(
            f"Publishing cmd_vel: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}"
        )

    def stop(self):
        self.cmd_vel_pub.publish(Twist())

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
        marker.lifetime = Duration(sec=1, nanosec=0)

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        start = Point(x=0.0, y=0.0, z=0.1)
        end = Point(
            x=self.max_range * math.cos(angle),
            y=self.max_range * math.sin(angle),
            z=0.1,
        )
        marker.points = [start, end]

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = ExplorerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
