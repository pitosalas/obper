#!/usr/bin/env python3
# File: obper/test_costmap_subscriber.py
# Authors: Pito Salas and ChatGPT
# License: MIT
# Version: 2.3
# Last Revised: 2025-05-01

import math
import numpy as np
from nav_msgs.msg import OccupancyGrid
from obper.costmap_subscriber import BeamChecker

# Create an empty OccupancyGrid with given physical size and resolution.
# width_m and height_m are in meters; resolution is meters per cell.
# Returns an OccupancyGrid with all cells set to 0 (free).
def create_empty_costmap(width_m: float, height_m: float, resolution: float) -> OccupancyGrid:
    width = int(width_m / resolution)  # cells in x
    height = int(height_m / resolution)  # cells in y

    msg = OccupancyGrid()
    msg.info.width = width  # info.width is width of map in number cells
    msg.info.height = height  # info.height is height of map in number of cells
    msg.info.resolution = resolution
    msg.info.origin.position.x = 0.0
    msg.info.origin.position.y = 0.0
    msg.data = [0] * (width * height)  # 0 = free
    return msg

# Mark specified (i, j) grid cells as occupied in the OccupancyGrid.
# Each (i, j) is a column,row index into the 2D grid. Value defaults to 100 (occupied).
def mark_cells(costmap_msg: OccupancyGrid, cells: list[tuple[int, int]], value: int = 100) -> None:
    for i, j in cells:
        if 0 <= i < costmap_msg.info.width and 0 <= j < costmap_msg.info.height:
            idx = j * costmap_msg.info.width + i
            costmap_msg.data[idx] = value
        else:
            print(f'Error in mark_cells {i},{j}')

# add_vertical_wall: Add a vertical wall at x_m (in meters) by marking a full column in the costmap as occupied.
def add_vertical_wall(costmap_msg, x_m):
    i = round(x_m / costmap_msg.info.resolution)
    i = max(0, min(i, costmap_msg.info.width - 1)) 
    mark_cells(costmap_msg, [(i, j) for j in range(costmap_msg.info.height)])

# add_horizontal_wall: wall at y_m (in meters) by marking a full row in the costmap as occupied.
def add_horizontal_wall(costmap_msg: OccupancyGrid, y_m: float) -> None:
    j = round(y_m / costmap_msg.info.resolution)
    j = max(0, min(j, costmap_msg.info.height - 1))  # Clamp to valid grid index
    mark_cells(costmap_msg, [(i, j) for i in range(costmap_msg.info.width)])

# add_diagonal_wall: Mark a diagonal line of obstacles across the costmap from "/" or "\\" direction.
def add_diagonal_wall(costmap_msg: OccupancyGrid, direction: str = "/") -> None:
    w = costmap_msg.info.width
    h = costmap_msg.info.height
    size = min(w, h)
    if direction == "/":
        cells = [(i, h - 1 - i) for i in range(size)]  # top-left to bottom-right
    elif direction == "\\":
        cells = [(i, i) for i in range(size)]          # bottom-left to top-right
    else:
        raise ValueError(f"Invalid diagonal direction: {direction!r}. Use '/' or '\\\\'.")
    mark_cells(costmap_msg, cells)

# add_box_around: Mark a square obstacle centered at (x_m, y_m) with total width in meters.
def add_box_around(costmap_msg: OccupancyGrid, x_m: float, y_m: float, width_m: float) -> None:
    res = costmap_msg.info.resolution
    i0 = round(x_m / res)
    j0 = round(y_m / res)
    half = round((width_m / 2) / res)
    wall = [
        (i0 + dx, j0 + dy)
        for dx in range(-half, half + 1)
        for dy in range(-half, half + 1)
        if abs(dx) == half or abs(dy) == half
    ]
    mark_cells(costmap_msg, wall)
    
def print_costmap_with_robot(costmap_msg, robot_x, robot_y):
    """
    Prints the costmap to the console using ASCII characters, marking the robot position.
    Assumes bottom-left origin and standard ROS axis: +x right, +y up.

    '#' = obstacle (>=100), '.' = free (<100), 'R' = robot
    """
    width = costmap_msg.info.width
    height = costmap_msg.info.height
    resolution = costmap_msg.info.resolution
    data = costmap_msg.data

    # Convert robot's world coordinates to map grid indices
    rx = int(robot_x / resolution)
    ry = int(robot_y / resolution)

    for y in reversed(range(height)):  # Print from top row to bottom
        row = ''
        for x in range(width):
            i = y * width + x
            if x == rx and y == ry:
                row += 'R'
            elif data[i] >= 100:
                row += '#'
            else:
                row += '.'
        print(row)


def beam_hits_straight_wall(
    robot_x,
    robot_y,
    robot_yaw,
    beam_angles,
    wall_x0,
    wall_y0,
    wall_angle,
    max_range,
    wall_bounds,
):
    distances = []
    wx = math.cos(wall_angle)  # Wall direction vector x-component
    wy = math.sin(wall_angle)  # Wall direction vector y-component

    for angle in beam_angles:
        # Compute beam direction vector
        dx = math.cos(robot_yaw + angle)
        dy = math.sin(robot_yaw + angle)

        # Compute denominator of the intersection formula
        denom = dx * wy - dy * wx

        if abs(denom) < 1e-8:
            # Case 1: Beam is parallel to the wall (no intersection)
            # Append max_range to indicate no intersection within range
            distances.append(max_range)
            continue

        # Vector from robot to a point on the wall
        dx0 = wall_x0 - robot_x
        dy0 = wall_y0 - robot_y

        # Compute parameter t along the beam direction where intersection occurs
        t = (dx0 * wy - dy0 * wx) / denom

        if t < 0 or t > max_range:
            # Case 2: Intersection point is behind the robot or beyond max_range
            # Append max_range to indicate no valid intersection within range
            distances.append(max_range)
            continue

        # Compute the intersection point coordinates
        x_hit = robot_x + t * dx
        y_hit = robot_y + t * dy

        # Check if the intersection point lies within the wall bounds
        (xmin, xmax), (ymin, ymax) = wall_bounds
        if not (xmin <= x_hit <= xmax and ymin <= y_hit <= ymax):
            # Case 3: Intersection point is outside the wall segment
            # Append max_range to indicate intersection is outside wall bounds
            distances.append(max_range)
        else:
            # Case 4: Valid intersection within wall bounds
            # Append the distance t to the intersection point
            distances.append(t)
    return distances


def assert_all_beams(expected, actual, tolerance=0.10):
    for i, (e, a) in enumerate(zip(expected, actual)):
        if not math.isclose(e, a, rel_tol=tolerance):
            print(f'❌ Measured Beam {i:2d}: {a:.2f}m vs expected {e:.2f}m')
        else:
            print(f'✅ Measured Beam {i:2d}: {a:.2f}m')


def test_case(name, costmap_msg, robot_x, robot_y, robot_yaw, expected, max_scan_range):
    print(f'\n===== {name} =====')
    beam_checker = BeamChecker(
        resolution=costmap_msg.info.resolution,
        origin_x=costmap_msg.info.origin.position.x,
        origin_y=costmap_msg.info.origin.position.y,
        width=costmap_msg.info.width,
        height=costmap_msg.info.height,
        costmap=costmap_msg.data,
        cost_threshold=50,
    )
    angles = np.linspace(-math.pi / 2, math.pi / 2, len(expected))
    widths = [0.1] * len(angles)
    print_costmap_with_robot(costmap_msg, robot_x, robot_y)
    actual = beam_checker.check_beams(robot_x, robot_y, robot_yaw, angles, widths, max_scan_range)
    assert_all_beams(expected, actual)


def main():
    map_w, map_h, res = 3.0, 3.0, 0.05
    angles = np.linspace(-math.pi / 2, math.pi / 2, 9)
    wall_bounds = ((0.0, map_w), (0.0, map_h))
    max_range = 3.0
    # Test 1
    x1, y1, yaw1 = 1.5, 1.5, 0.0
    msg1 = create_empty_costmap(map_w, map_h, res)
    wall_x = 3.0
    wall_y = 0.0
    add_vertical_wall(msg1, wall_x)
    expected1 = beam_hits_straight_wall(
        x1, y1, yaw1, angles, wall_x, wall_y, math.pi / 2, max_range, wall_bounds
    )
    test_case('Test 1', msg1, x1, y1, yaw1, expected1, max_range)
    # Test 2
    x2, y2, yaw2 = 1.5, 1.0, math.pi / 2
    msg2 = create_empty_costmap(map_w, map_h, res)
    add_horizontal_wall(msg2, 2.0)
    expected2 = beam_hits_straight_wall(x2, y2, yaw2, angles, 0.0, 2.0, 0.0, max_range, wall_bounds)
    test_case('Test 2', msg2, x2, y2, yaw2, expected2, max_range)
    # # Test 3
    #     x3, y3, yaw3 = 0.0, 1.0, 0.0
    #     msg3 = create_empty_costmap(map_w, map_h, res)
    #     add_diagonal_wall(msg3, '/')
    #     expected3 = beam_hits_straight_wall(x3, y3, yaw3, angles, 0.0, 3.0, -math.pi*3/4, max_range, wall_bounds)
    #     test_case("Test 3", msg3, x3, y3, yaw3, expected3, max_range)
    # Test 4
    x4, y4, yaw4 = 1.5, 1.5, 0.0
    msg4 = create_empty_costmap(map_w, map_h, res)
    add_box_around(msg4, x4, y4, radius_m=0.05)
    expected4 = [0.05] * len(angles)
    test_case('Test 4', msg4, x4, y4, yaw4, expected4, max_range)


if __name__ == '__main__':
    main()
