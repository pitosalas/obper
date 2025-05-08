#!/usr/bin/env python3
# File: obper/costmap_subscriber.py
# Authors: Pito Salas and ChatGPT
# License: MIT
# Version: 2.1
# Last Revised: 2025-04-27

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from robot_msgs.msg import BeamDistances
from geometry_msgs.msg import Point
import tf2_ros
import tf_transformations
import numpy as np
import math
from rclpy.time import Time
from rclpy.duration import Duration


# Description:
# This script implements a ROS 2 node (`LocalCostmapSubscriber`) that subscribes to a local costmap
# and performs beam analysis to detect obstacles. The costmap is stored as a 1D array in the
# `BeamChecker` class, which represents the occupancy grid. The map's dimensions (width, height) and resolution are used
# to interpret the 1D array as a 2D grid. The node also publishes beam distances and visual markers for
# visualization in RViz.

# The costmap data is updated in the costmap_callback method of the LocalCostmapSubscriber class,
# where the msg.data from the OccupancyGrid message is passed to the BeamChecker instance. T
# he BeamChecker uses this data to perform operations like checking beam distances
# and determining obstacle costs.

# The mapping between 2D grid coordinates (i, j) and the 1D array index is done using the formula:
# idx = j * self.width + i
# Here:

# i is the column index.
# j is the row index.
# self.width is the number of columns in the grid.


class BeamChecker:
    """Logic class for beam checking without ROS dependencies."""

    def __init__(
        self,
        resolution: float,
        origin_x: float,
        origin_y: float,
        width: int,
        height: int,
        costmap,
    ) -> None:
        """
        Initialize a CostmapSubscriber instance.

        Args:
            resolution (float): The resolution of the costmap in meters per cell.
            origin_x (float): The x-coordinate of the costmap's origin.
            origin_y (float): The y-coordinate of the costmap's origin.
            width (int): The width of the costmap in number of cells.
            height (int): The height of the costmap in number of cells.
            costmap: The costmap grid or data structure representing obstacle costs.
            cost_threshold (int, optional): The cost value threshold to determine obstacles. Defaults to 50.
        """
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.width = width
        self.height = height
        self.costmap = costmap
        self.cost_threshold = 95
        self.max_scan_range = 1.5
        self.verbose:bool = False

    def update_data(self, costmap):
        """Fast update of just the occupancy data (assumes dimensions same)."""
        self.costmap = costmap

    def reset_costmap(self, resolution, origin_x, origin_y, width, height, costmap):
        """Full reset if the costmap dimensions or layout change."""
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.width = width
        self.height = height
        self.costmap = costmap

    def world_to_map(self, x, y):
        # i = int((x - self.origin_x) / self.resolution)
        # j = int((y - self.origin_y) / self.resolution)
        i = round((x - self.origin_x) / self.resolution)
        j = round((y - self.origin_y) / self.resolution)
        if 0 <= i < self.width and 0 <= j < self.height:
            return (i, j)
        else:
            if self.verbose:
                print(
                    f"world to map: x={x:.2f} y={y:.2f} i={i:.2f} j={j:.2f}"
                    f"{self.origin_y:0.1f} [cond1: {0 <= i < self.width} cond2: {0 <= j < self.height}]"
                )
            return None

    def map_cost(self, i, j):
        idx = j * self.width + i
        return self.costmap[idx]

    def check_beams(self, robot_x, robot_y, robot_yaw, angles, widths):
        if self.verbose:
            print(f"====== anglestocheck={angles} x={robot_x:0.2} y={robot_y:0.2} yaw={robot_yaw:0.2}")
        if self.costmap is None:
            return [None] * len(angles)
        step_size = self.resolution / 2.0
        distances = []
        for angle, _ in zip(angles, widths):
            if self.verbose:
                print(f"=================stepsize={step_size} angle={angle:1.2}")
            distance = self.max_scan_range
            global_angle = angle + robot_yaw
            steps = int(self.max_scan_range / step_size)
            for step in range(steps):
                if self.verbose:
                    print(f"Check beams step {step}")
                d = step * step_size
                x = robot_x + d * math.cos(global_angle)
                y = robot_y + d * math.sin(global_angle)
                result = self.world_to_map(x, y)

                # If beam reaches outside map, we assume there's a wall beyond the map — that's the max distance
                result_outside_map = result is None

                # If beam is inside map, we check if the cost is above the threshold (indicating an obstacle)
                result_as_string = "<out>"
                cost_as_string = "<out>"
                if result:
                    obstacle_in_map = self.map_cost(*result) > self.cost_threshold
                    result_as_string = result
                    cost_as_string = f"{self.map_cost(*result):<3.0f}"
                if self.verbose:
                    print(f"{step:<4}{d:>4.2f}  {x:>4.1f},{y:>4.1f}, {global_angle:3.1f}°  {result_as_string}->{cost_as_string}")
                if result_outside_map or obstacle_in_map:
                    # distance = d
                    distance = math.dist((robot_x, robot_y), (x, y))
                    if self.verbose:
                        print(f"Beam measured distance: {distance:2.2}")
                    break
            distances.append(distance)
        if self.verbose:
            print("Exiting check_beams")
            self.mp()
        return distances

    def mpold(self):
        print(
            "\n".join(
                " ".join(
                    f"{self.costmap[i * self.width + j]:>3}" for j in range(self.width)
                )
                for i in reversed(range(self.height))
            )
        )
    def mp(self):
        # Column header
        header = "    " + " ".join(f"{j:>3}" for j in range(self.width))
        print(header)
        for i in reversed(range(self.height)):
            row = " ".join(f"{self.costmap[i * self.width + j]:>3}" for j in range(self.width))
            print(f"{i:>3}:" + " " + row)

class LocalCostmapSubscriber(Node):
    """ROS 2 wrapper node for local costmap subscription and beam analysis."""

    MAX_SCAN_RANGE = 1.5
    MIN_CRASH_RANGE = 0.5

    def __init__(
        self,
        tf_buffer=None,
        timer_period=0.25,
        target_frame="odom",
        source_frame="base_link",
        create_timer=True,
    ):
        super().__init__("local_costmap_subscriber")

        self.target_frame = target_frame
        self.source_frame = source_frame
        self.default_max_scan_range = LocalCostmapSubscriber.MAX_SCAN_RANGE  # How far to look to find an obstacle
        self.default_min_crash_distance = LocalCostmapSubscriber.MIN_CRASH_RANGE  # just used for color of marker
        self.default_step_size = None
        self.update_counter = 0

        self.subscription = self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self.costmap_callback, 10
        )

        self.marker_pub = self.create_publisher(MarkerArray, "/beam_markers", 10)
        self.beam_pub = self.create_publisher(BeamDistances, "/beam_distances", 10)

        self.tf_buffer = tf_buffer if tf_buffer else tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.beam_checker = None  # Will be initialized after first costmap

        if create_timer:
            self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("LocalCostmapSubscriber initialized.")

    def costmap_callback(self, msg):
        self.update_counter += 1
        #print(f"Map Callback: res={msg.info.resolution:0.1} org: x={msg.info.origin.position.x:0.2} y={msg.info.origin.position.y:0.2}")
        # if self.beam_checker is None:
        self.beam_checker = BeamChecker(
            resolution=msg.info.resolution,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
            width=msg.info.width,
            height=msg.info.height,
            costmap=msg.data,
        )
        self.get_logger().info(
            f"Costmap Callback: (cb: {self.update_counter})."
        )
        # else:
        #     self.get_logger().info(f"Updated BeamChecker (cb: {self.update_counter})")

        #     self.beam_checker.update_data(msg.data)

    def get_robot_pose(self):
        try:
            now = Time()
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                now,
                timeout=Duration(seconds=0.2),
            )
            t = transform.transform.translation
            q = transform.transform.rotation
            roll, pitch, yaw = tf_transformations.euler_from_quaternion(
                [q.x, q.y, q.z, q.w]
            )
#            print(f"Odom: {t.x:0.2} {t.y:0.2} {yaw:0.2}")
            return t.x, t.y, yaw
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn("TF lookup failed.")
            return None

    def timer_callback(self):
        if self.beam_checker is None:
            return
        pose = self.get_robot_pose()
        if pose is None:
            return
        x, y, yaw = pose
        angles = np.linspace(-math.pi / 2, math.pi / 2, 9)
        widths = [0.1] * len(angles)
        distances = self.beam_checker.check_beams(x, y, yaw, angles, widths)
        self.publish_beam_distances(angles, distances)
        self.publish_beam_markers(angles, distances)
#        print(f" angles:<{','.join(f'{x:.2f}' for x in angles)}> distances:<{','.join(f'{x:.2f}' for x in distances)}>")

    def publish_beam_distances(self, angles, distances):
        msg = BeamDistances()
        msg.angles = list(angles)
        msg.distances = list(distances)
        self.beam_pub.publish(msg)

    def publish_beam_markers(self, angles, distances, frame_id="base_link"):
        marker_array = MarkerArray()
        for idx, (angle, distance) in enumerate(zip(angles, distances)):
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)

            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = Time().to_msg()
            marker.ns = "beam_rays"
            marker.id = idx
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.scale.x = 0.005
            marker.scale.y = 0.02
            marker.scale.z = 0.03
            marker.color.r = 1.0 if distance < self.default_min_crash_distance else 0.0
            marker.color.g = 0.0 if distance < self.default_min_crash_distance else 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            start = Point(x=0.0, y=0.0, z=0.1)
            end = Point(x=x, y=y, z=0.1)
            marker.points = [start, end]

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

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
