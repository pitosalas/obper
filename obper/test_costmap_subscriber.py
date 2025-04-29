#!/usr/bin/env python3
# File: obper/test_costmap_subscriber.py
# Authors: Pito Salas and ChatGPT
# License: MIT
# Version: 1.2
# Last Revised: 2025-04-27

import numpy as np
import math
from nav_msgs.msg import OccupancyGrid
from linorobot2_ws.src.obper.obper.costmap_subscriber import BeamChecker

def create_simple_costmap(width, height):
    """Creates a simple OccupancyGrid with free space below 'obstacle_rows' and obstacles above."""
    msg = OccupancyGrid()
    msg.info.width = width
    msg.info.height = height
    msg.info.resolution = 0.1
    msg.info.origin.position.x = 0.0
    msg.info.origin.position.y = 0.0
    data = []
    for j in range(height):
        for i in range(width):
            data.append(0)
    msg.data = data
    return msg

def print_costmap_with_robot(costmap_msg, robot_x, robot_y, robot_marker='R'):
    """Prints the costmap as text, marking the robot's current position."""
    width = costmap_msg.info.width
    height = costmap_msg.info.height
    resolution = costmap_msg.info.resolution
    origin_x = costmap_msg.info.origin.position.x
    origin_y = costmap_msg.info.origin.position.y
    data = costmap_msg.data

    # Compute robot's cell (i, j)
    i_robot = int((robot_x - origin_x) / resolution)
    j_robot = int((robot_y - origin_y) / resolution)

    print("\nCostmap (0=free, #=obstacle, R=robot):")
    for j in reversed(range(height)):
        row = ""
        for i in range(width):
            idx = j * width + i
            if i == i_robot and j == j_robot:
                row += robot_marker
            elif data[idx] > 50:
                row += "#"
            else:
                row += "."
        print(row)
    print()

def wall_at_x(costmap_msg, x_wall):
    """Creates a vertical wall at x in the costmap."""
    width = costmap_msg.info.width
    height = costmap_msg.info.height
    resolution = costmap_msg.info.resolution
    x_wall_cell = int((x_wall) / resolution)

    for y in range(height):
        for x in range(width):
            if x == x_wall_cell:
                idx = y * width + x
                costmap_msg.data[idx] = 100  # Mark as obstacle

def assert_beam(beam_index, expected_distance, distances):
    """Asserts that the beam at index 'beam_index' has the expected distance."""
    if not math.isclose(distances[beam_index], expected_distance, rel_tol=0.05):
        print(f"Beam {beam_index} failed: {distances[beam_index]:.2f}m vs expected {expected_distance:.2f}m")
    else:
        print(f"✅ Beam {beam_index} passed: {distances[beam_index]:.2f}m")

def main():
    """Test BeamChecker directly without ROS 2 or LocalCostmapSubscriber."""
    costmap_msg = create_simple_costmap(21, 21)
    wall_at_x(costmap_msg, 2.0)
    robot_pose = (0.0, 1.0, 0.0)  # x, y, yaw

    angles = np.linspace(-math.pi/2, math.pi/2, 9)
    widths = [0.1] * len(angles)
    robot_x, robot_y, robot_yaw = robot_pose
    
    beam_checker = BeamChecker(
        resolution=costmap_msg.info.resolution,
        origin_x=costmap_msg.info.origin.position.x,
        origin_y=costmap_msg.info.origin.position.y,
        width=costmap_msg.info.width,
        height=costmap_msg.info.height,
        costmap=costmap_msg.data
    )

    distances = beam_checker.check_beams(robot_x, robot_y, robot_yaw, angles, widths)
    print_costmap_with_robot(costmap_msg, robot_x=robot_x, robot_y=robot_y)
    assert_beam(4, 2.0, distances)

if __name__ == "__main__":
    main()