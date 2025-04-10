#!/usr/bin/env python3
# Authors: Pito Salas and ChatGPT

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
import math
import numpy as np


class LocalCostmapSubscriber(Node):
    def __init__(self):
        super().__init__('local_costmap_listener')

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',
            self.costmap_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.marker_pub = self.create_publisher(MarkerArray, '/beam_markers', 10)

        self.costmap = None
        self.resolution = None
        self.origin_x = None
        self.origin_y = None
        self.width = None
        self.height = None
        self.get_logger().info("Waiting for costmap...")

    def costmap_callback(self, msg: OccupancyGrid):
        self.costmap = msg.data
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.width = msg.info.width
        self.height = msg.info.height
        self.get_logger().debug(f"Received costmap: {self.width}x{self.height}, res={self.resolution}")

    def world_to_map(self, x, y):
        if self.resolution is None:
            return None

        i = int(math.floor((x - self.origin_x) / self.resolution))
        j = int(math.floor((y - self.origin_y) / self.resolution))

        if 0 <= i < self.width and 0 <= j < self.height:
            return (i, j)
        else:
            self.get_logger().debug(f"World({x:.2f}, {y:.2f}) -> Map({i}, {j}) OOB!.")
            return None

    def map_to_world(self, i, j):
        if self.resolution is None:
            return None

        if 0 <= i < self.width and 0 <= j < self.height:
            x = self.origin_x + (i + 0.5) * self.resolution
            y = self.origin_y + (j + 0.5) * self.resolution
            return (x, y)
        else:
            self.get_logger().debug(f"Grid index ({i}, {j}) out of bounds.")
            return None

    def print_costmap(self):
        """
        Debug tool: Print the costmap as text using cost-based symbols.
        Uses lookup tables for cost cutoffs and corresponding symbols.
        """
        if self.costmap is None:
            self.get_logger().warn("No costmap data to print.")
            return

        # Define cost cutoffs and their matching symbols
        cutoffs = [20, 40, 60, 80, 100]
        symbols = [' ', '.', '+', '*', 'X']

        output = []

        for j in reversed(range(self.height)):
            row = ""
            for i in range(self.width):
                idx = j * self.width + i
                cost = self.costmap[idx]

                if cost < 0 or cost > 100:
                    row += '?'  # Unexpected or unknown
                else:
                    for cutoff, symbol in zip(cutoffs, symbols):
                        if cost <= cutoff:
                            row += symbol
                            break
            output.append(row)

        print("\n".join(output))

    def check_beams(self, angles, widths, max_range=2.5, step_size=None):
        if self.costmap is None or self.resolution is None:
            self.get_logger().warn("Costmap not ready for beam checking.")
            return [None] * len(angles)

        if step_size is None:
            step_size = self.resolution / 2.0

        robot_x = self.origin_x + self.width * self.resolution / 2.0
        robot_y = self.origin_y + self.height * self.resolution / 2.0

        distances = []

        for angle, spread in zip(angles, widths):
            distance = max_range
            steps = int(max_range / step_size)

            for step in range(steps):
                d = step * step_size
                x = robot_x + d * math.cos(angle)
                y = robot_y + d * math.sin(angle)

                result = self.world_to_map(x, y)
                if result is None:
                    cost = 0
                else:
                    i, j = result
                    idx = j * self.width + i
                    cost = self.costmap[idx]

                if cost > 50:
                    distance = d
                    break

            distances.append(distance)

        return distances

    def publish_beam_markers(self, angles, distances, frame_id="odom"):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        robot_x = self.origin_x + self.width * self.resolution / 2.0
        robot_y = self.origin_y + self.height * self.resolution / 2.0

        for idx, (angle, distance) in enumerate(zip(angles, distances)):
            x = robot_x + distance * math.cos(angle)
            y = robot_y + distance * math.sin(angle)

            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = now
            marker.ns = "beam_rays"
            marker.id = idx
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.scale.x = 0.02  # shaft diameter
            marker.scale.y = 0.02   # head diameter
            marker.scale.z = 0.02   # head length

            # Green for clear, red for blocked
            if distance < 2.5:
                marker.color.r = 1.0
                marker.color.g = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            start = Point(x=robot_x, y=robot_y, z=0.1)
            end = Point(x=x, y=y, z=0.1)
            marker.points = [start, end]

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = LocalCostmapSubscriber()

    try:
        def timer_callback():
            if node.costmap:
                angles = np.linspace(-math.pi/2, math.pi/2, 9)
                widths = [0.1] * len(angles)
                distances = node.check_beams(angles, widths)
                node.publish_beam_markers(angles, distances)

        node.create_timer(0.5, timer_callback)
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()