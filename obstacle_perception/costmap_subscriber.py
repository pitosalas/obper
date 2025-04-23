#!/usr/bin/env python3
# Authors: Pito Salas and ChatGPT

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from robot_msgs.msg import BeamDistances
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import tf_transformations
import math
import numpy as np


class LocalCostmapSubscriber(Node):
    def __init__(self):
        super().__init__('local_costmap_listener')

        # Subscriptions and publishers
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',
            self.costmap_callback,
            10)

        self.marker_pub = self.create_publisher(MarkerArray, '/beam_markers', 10)
        self.beam_pub = self.create_publisher(BeamDistances, '/beam_distances', 10)

        # TF for base_link → odom transform
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_frame = "odom"
        self.source_frame = "base_link"

        # Map data
        self.costmap = None
        self.resolution = None
        self.origin_x = None
        self.origin_y = None
        self.width = None
        self.height = None

        self.get_logger().info("Waiting for costmap...")
        self.create_timer(0.5, self.timer_callback)

    def costmap_callback(self, msg: OccupancyGrid):
        self.costmap = msg.data
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.width = msg.info.width
        self.height = msg.info.height

    def get_robot_yaw(self):
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                now,
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
            q = transform.transform.rotation
            _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            return yaw
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn("TF lookup failed.")
            return None

    def world_to_map(self, x, y):
        if self.resolution is None:
            return None

        i = int((x - self.origin_x) / self.resolution)
        j = int((y - self.origin_y) / self.resolution)

        if 0 <= i < self.width and 0 <= j < self.height:
            return (i, j)
        else:
            return None

    def map_to_world(self, i, j):
        if self.resolution is None:
            return None

        if 0 <= i < self.width and 0 <= j < self.height:
            x = self.origin_x + (i + 0.5) * self.resolution
            y = self.origin_y + (j + 0.5) * self.resolution
            return (x, y)
        else:
            return None

    def check_beams(self, angles, widths, max_range=2.5, step_size=None):
        if self.costmap is None or self.resolution is None:
            self.get_logger().warn("Costmap not ready for beam checking.")
            return [None] * len(angles)

        yaw = self.get_robot_yaw()
        if yaw is None:
            return [None] * len(angles)

        if step_size is None:
            step_size = self.resolution / 2.0

        robot_x = self.origin_x + self.width * self.resolution / 2.0
        robot_y = self.origin_y + self.height * self.resolution / 2.0

        distances = []

        for angle, spread in zip(angles, widths):
            distance = max_range
            global_angle = angle + yaw
            steps = int(max_range / step_size)

            for step in range(steps):
                d = step * step_size
                x = robot_x + d * math.cos(global_angle)
                y = robot_y + d * math.sin(global_angle)

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

    def publish_beam_markers(self, angles, distances, frame_id="base_link"):
        marker_array = MarkerArray()

        for idx, (angle, distance) in enumerate(zip(angles, distances)):
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)

            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = rclpy.time.Time().to_msg()  # time=0 means "latest available TF"
            marker.ns = "beam_rays"
            marker.id = idx
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.scale.x = 0.02
            marker.scale.y = 0.02
            marker.scale.z = 0.02

            marker.color.r = 1.0 if distance < 2.5 else 0.0
            marker.color.g = 0.0 if distance < 2.5 else 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            start = Point(x=0.0, y=0.0, z=0.1)
            end = Point(x=x, y=y, z=0.1)
            marker.points = [start, end]

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

    def timer_callback(self):
        if self.costmap is None:
            return

        angles = np.linspace(-math.pi/2, math.pi/2, 9)
        widths = [0.1] * len(angles)
        distances = self.check_beams(angles, widths)

        # Publish distances
        msg = BeamDistances()
        msg.angles = list(angles)
        msg.distances = distances
        self.beam_pub.publish(msg)

        # Publish RViz markers
        self.publish_beam_markers(angles, distances)

    def print_costmap(self):
        if self.costmap is None:
            self.get_logger().warn("No costmap data to print.")
            return

        cutoffs = [20, 40, 60, 80, 100]
        symbols = [' ', '.', '+', '*', 'X']

        output = []
        for j in reversed(range(self.height)):
            row = ""
            for i in range(self.width):
                idx = j * self.width + i
                cost = self.costmap[idx]

                if cost < 0 or cost > 100:
                    row += '?'
                else:
                    for cutoff, symbol in zip(cutoffs, symbols):
                        if cost <= cutoff:
                            row += symbol
                            break
            output.append(row)

        print("\n".join(output))


def main(args=None):
    rclpy.init(args=args)
    node = LocalCostmapSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()