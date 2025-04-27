
#!/usr/bin/env python3
# Authors: Pito Salas and ChatGPT
# License: MIT
# File: costmap_subscriber.py
# Version: 1.1
# Last Revised: 2025-04-27

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray
from robot_msgs.msg import BeamDistances
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import tf_transformations
import math
import numpy as np
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Marker

class LocalCostmapSubscriber(Node):
    def __init__(self,
                 target_frame="odom",
                 source_frame="base_link",
                 cost_threshold=50,
                 default_max_range=2.5,
                 default_step_size=None):
        super().__init__('local_costmap_listener')

        # Configurable parameters
        self.target_frame = target_frame
        self.source_frame = source_frame
        self.cost_threshold = cost_threshold
        self.default_max_range = default_max_range
        self.default_step_size = default_step_size

        # Subscriptions and publishers
        self.subscription = self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self.costmap_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/beam_markers', 10)
        self.beam_pub = self.create_publisher(BeamDistances, '/beam_distances', 10)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Internal costmap
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
                timeout=rclpy.duration.Duration(seconds=0.2))
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

    def check_beams(self, angles, widths, max_range=None, step_size=None):
        if self.costmap is None or self.resolution is None:
            self.get_logger().warn("Costmap not ready for beam checking.")
            return [None] * len(angles)

        yaw = self.get_robot_yaw()
        if yaw is None:
            return [None] * len(angles)

        max_range = max_range if max_range is not None else self.default_max_range
        step_size = step_size if step_size is not None else (self.default_step_size or self.resolution / 2.0)

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

                if cost > self.cost_threshold:
                    distance = d
                    break

            distances.append(distance)

        return distances

    def timer_callback(self):
        pass  # Empty for unit test purposes

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

if __name__ == "__main__":
    main()