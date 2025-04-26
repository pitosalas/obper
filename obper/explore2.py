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
from geometry_msgs.msg import Twist
from robot_msgs.msg import BeamDistances
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import tf_transformations
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
import numpy as np
import math
import random
import logging

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
        self.goal_point : Point | None = None
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
        self.occ_marker = self.create_publisher(Marker, "/occupancy", 10)
        self.goal_marker = self.create_publisher(Marker, "/goal", 10)     

        self.timer = self.create_timer(0.2, self.control_loop)  # 5 Hz
        self.cmd_timer = self.create_timer(0.05, self.publish_current_twist)  # 20 Hz
        self.current_twist = Twist()

        # Compact format, include ALL levels (DEBUG and above)
        logging.basicConfig(
            format='[%(levelname)s] %(message)s',
            level=logging.DEBUG  # <== Important: allow DEBUG, INFO, WARN, ERROR
        )

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

        x = pose.x + best_distance * math.cos(pose.y + best_angle)
        y = pose.y + best_distance * math.sin(pose.y + best_angle)
        self.goal_point : Point = Point(x=x, y=y)
        self.set_state("TURN")
        self.get_logger().info(
            f"New goal: ({x:.2f}, {y:.2f}) from angle={math.degrees(best_angle):.1f}°, distance={best_distance:.2f}"
        )

    def set_state(self, state):
        if state not in ["IDLE", "TURN", "DRIV"]:
            self.get_logger().debug(f"Invalid state: {state}")
            return
        self.current_state = state
        self.get_logger().debug(f"State changed to: {self.current_state}")

    def get_robot_pose(self) -> Point | None:
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform("odom", "base_link", now)
            t = trans.transform.translation
            q = trans.transform.rotation
            roll, pitch, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])            
            return Point(x=t.x, y=t.y, z=yaw)
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
        self.update_visited_map(pose.x, pose.y)
        self.publish_visited_cells()
        self.publish_goal_marker()

        if self.current_state == "TURN":
            angle_to_goal = math.atan2(
                self.goal_point.y - pose.y, self.goal_point.x - pose.x
            )
            angle_diff = self.normalize_angle(angle_to_goal - pose.y)
            print(f"angle to goal: {math.degrees(angle_to_goal):.1f}°, "
                  f"angle diff: {math.degrees(angle_diff):.1f}°")
            if abs(angle_diff) < 0.1:
                self.set_state("DRIV")
                self.publish_twist(0.0, 0.0)
            else:
                self.publish_twist(0.0, self.angular_speed * np.sign(angle_diff))

        elif self.current_state == "DRIV":
            dx = self.goal_point.x - pose.x
            dy = self.goal_point.y - pose.y
            dist = math.hypot(dx, dy)
            if dist < self.goal_tolerance:
                self.publish_twist(0.0, 0.0)
                self.set_state("IDLE")
            else:
#                angle_to_goal = math.atan2(dy, dx)
#                angle_diff = self.normalize_angle(angle_to_goal - pose.y)
#                ang_z = np.clip(self.angular_speed * angle_diff, -1.0, 1.0)
                self.publish_twist(self.linear_speed, 0,0)
        self.log_loop_data(pose)

    def log_loop_data(self, pose: Point):
        minf = float("-inf")
        lx = self.current_twist.linear.x
        az = self.current_twist.angular.z
        if pose is None:
            px, py = minf, minf
        else:
            px = pose.x
            py = pose.y
        if self.goal_point is None:
            gx, gy = minf, minf
        else:
            gx = self.goal_point.x
            gy = self.goal_point.y
        current_beam_as_str = ",".join(
            f"{a:.1f}:{d:.1f}" for a, d in self.current_beams
        )
        dist = math.hypot(gx - px, gy - py) if self.goal_point else minf
        self.get_logger().info(
            f"{self.loop_count:3d},{self.current_state},{lx:.1f},{az:.1f}, {px:.2f},{py:.2f},{gx:.1f},{gy:.1f},{dist:.1f},[{current_beam_as_str}]"
        )

    def create_marker(
        self, ns, marker_id, marker_type, color: ColorRGBA, scale: Vector3):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.scale = scale     
        marker.color = color      
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
        marker = self.create_marker(    
            "visited", 0, Marker.CUBE_LIST,
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
            Vector3(x=self.grid_resolution, y=self.grid_resolution, z=0.1))


        # for j in range(self.grid_size):
        #     for i in range(self.grid_size):
        #         if self.visited_map[j, i]:
        #             pt = Point()
        #             pt.x = (i - self.grid_size / 2) * self.grid_resolution
        #             pt.y = (j - self.grid_size / 2) * self.grid_resolution
        #             pt.z = 0.01
        #             marker.points.append(pt)

        # marker_array.markers.append(marker)

    def publish_goal_marker(self):
        if self.goal_point is None:
            return
        marker = self.create_marker(
            "goal", 1, Marker.SPHERE,
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
            Vector3(x=0.1, y=0.1, z=0.1)
        )
        marker.pose.position.x = self.goal_point.x
        marker.pose.position.y = self.goal_point.y
        marker.pose.position.z = 0.0
        self.goal_marker.publish(marker)

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
