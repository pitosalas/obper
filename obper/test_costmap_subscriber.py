# File: obper/test_costmap_subscriber.py
# License: MIT
# Authors: Pito Salas and ChatGPT
# Version: 2.4
# Last Revised: 2025-05-03

from dataclasses import dataclass
import math
import numpy as np
from nav_msgs.msg import OccupancyGrid
from obper.costmap_subscriber import BeamChecker

# Pose: robot position and orientation
@dataclass
class Pose:
    x: float
    y: float
    yaw: float

# WallSegment: infinite line + bounding box
@dataclass
class WallSegment:
    x0: float
    y0: float
    angle: float
    bounds: ((float, float), (float, float))

# beam_distances_to_wall_segment: return distances where beams hit a wall segment or max_range
def beam_distances_to_wall_segment(
    robot_pose: Pose,
    beam_angles: list[float],
    wall: WallSegment,
    max_range: float
) -> list[float]:
    distances = []
    wx = math.cos(wall.angle)
    wy = math.sin(wall.angle)
    for angle in beam_angles:
        dx = math.cos(robot_pose.yaw + angle)
        dy = math.sin(robot_pose.yaw + angle)
        denom = dx * wy - dy * wx
        if abs(denom) < 1e-8:
            distances.append(max_range)
            continue
        dx0 = wall.x0 - robot_pose.x
        dy0 = wall.y0 - robot_pose.y
        t = (dx0 * wy - dy0 * wx) / denom
        if t < 0 or t > max_range:
            distances.append(max_range)
            continue
        x_hit = robot_pose.x + t * dx
        y_hit = robot_pose.y + t * dy
        (xmin, xmax), (ymin, ymax) = wall.bounds
        if not (xmin <= x_hit <= xmax and ymin <= y_hit <= ymax):
            distances.append(max_range)
        else:
            distances.append(t)
    return distances

# create_empty_costmap: return free costmap grid
def create_empty_costmap(width_m: float, height_m: float, resolution: float) -> OccupancyGrid:
    width = int(width_m / resolution)
    height = int(height_m / resolution)
    msg = OccupancyGrid()
    msg.info.width = width
    msg.info.height = height
    msg.info.resolution = resolution
    msg.info.origin.position.x = 0.0
    msg.info.origin.position.y = 0.0
    msg.data = [0] * (width * height)
    return msg

# mark_cells: set specific grid cells to a value
def mark_cells(msg: OccupancyGrid, cells: list[tuple[int, int]], value: int = 100):
    for i, j in cells:
        if 0 <= i < msg.info.width and 0 <= j < msg.info.height:
            idx = j * msg.info.width + i
            msg.data[idx] = value

# add_vertical_wall: mark vertical obstacle
def add_vertical_wall(msg: OccupancyGrid, x_m: float):
    i = round(x_m / msg.info.resolution)
    i = max(0, min(i, msg.info.width - 1))
    mark_cells(msg, [(i, j) for j in range(msg.info.height)])

# add_horizontal_wall: mark horizontal obstacle
def add_horizontal_wall(msg: OccupancyGrid, y_m: float):
    j = round(y_m / msg.info.resolution)
    j = max(0, min(j, msg.info.height - 1))
    mark_cells(msg, [(i, j) for i in range(msg.info.width)])

# add_diagonal_wall: mark diagonal line
def add_diagonal_wall(msg: OccupancyGrid, direction: str = '/'):
    w = msg.info.width
    h = msg.info.height
    size = min(w, h)
    if direction == '/':
        cells = [(i, h - 1 - i) for i in range(size)]
    elif direction == '\\':
        cells = [(i, i) for i in range(size)]
    else:
        raise ValueError(f"bad direction {direction}")
    mark_cells(msg, cells)

# add_box_around: mark square border around point
def add_box_around(msg: OccupancyGrid, x_m: float, y_m: float, width_m: float):
    res = msg.info.resolution
    i0 = round(x_m / res)
    j0 = round(y_m / res)
    half = round((width_m / 2) / res)
    wall = [(i0 + dx, j0 + dy)
            for dx in range(-half, half + 1)
            for dy in range(-half, half + 1)
            if abs(dx) == half or abs(dy) == half]
    mark_cells(msg, wall)

# print_costmap_with_robot: ASCII map for debugging
def print_costmap_with_robot(msg: OccupancyGrid, rx: float, ry: float):
    w = msg.info.width
    h = msg.info.height
    res = msg.info.resolution
    ix = int(rx / res)
    iy = int(ry / res)
    for j in reversed(range(h)):
        row = ""
        for i in range(w):
            idx = j * w + i
            if i == ix and j == iy:
                row += "R"
            elif msg.data[idx] >= 100:
                row += "#"
            else:
                row += "."
        print(row)

# assert_all_beams: print check vs expected
def assert_all_beams(expected, actual, tolerance=0.10):
    for i, (e, a) in enumerate(zip(expected, actual)):
        if not math.isclose(e, a, abs_tol=tolerance):
            print(f'❌ Beam {i:2d}: {a:.2f}m vs expected {e:.2f}m')
        else:
            print(f'✅ Beam {i:2d}: {a:.2f}m')

# test_case: run one beam test
def test_case(name: str, costmap_msg: OccupancyGrid, pose: Pose, expected: list[float], max_scan_range: float):
    print(f'\n===== {name} =====')
    checker = BeamChecker(
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
    print_costmap_with_robot(costmap_msg, pose.x, pose.y)
    actual = checker.check_beams(pose.x, pose.y, pose.yaw, angles, widths, max_scan_range)
    assert_all_beams(expected, actual)

# main: define and run test cases
def main():
    map_w, map_h, res = 3.0, 3.0, 0.05
    max_range = 3.0
    wall_bounds = ((0.0, map_w), (0.0, map_h))
    angles = np.linspace(-math.pi / 2, math.pi / 2, 9)

    # Test 1
    pose1 = Pose(1.5, 1.5, 0.0)
    msg1 = create_empty_costmap(map_w, map_h, res)
    add_vertical_wall(msg1, 3.0)
    wall1 = WallSegment(3.0, 0.0, math.pi / 2, wall_bounds)
    expected1 = beam_distances_to_wall_segment(pose1, angles.tolist(), wall1, max_range)
    test_case("Test 1", msg1, pose1, expected1, max_range)

    # Test 2
    pose2 = Pose(1.5, 1.0, math.pi / 2)
    msg2 = create_empty_costmap(map_w, map_h, res)
    add_horizontal_wall(msg2, 2.0)
    wall2 = WallSegment(0.0, 2.0, 0.0, wall_bounds)
    expected2 = beam_distances_to_wall_segment(pose2, angles.tolist(), wall2, max_range)
    test_case("Test 2", msg2, pose2, expected2, max_range)

    # Test 3
    pose3 = Pose(1.5, 1.5, 0.0)
    msg3 = create_empty_costmap(map_w, map_h, res)
    add_box_around(msg3, pose3.x, pose3.y, 0.10)
    expected3 = [0.05] * len(angles)
    test_case("Test 3", msg3, pose3, expected3, max_range)

    # Test 4: diagonal wall (/ direction)
    pose4 = Pose(0.0, 1.0, 0.0)
    msg4 = create_empty_costmap(map_w, map_h, res)
    add_diagonal_wall(msg4, '/')
    wall4 = WallSegment(
        x0=map_w / 2,
        y0=map_h / 2,
        angle=-math.pi * 3 / 4,
        bounds=((0.0, map_w), (0.0, map_h))
    )
    expected4 = beam_distances_to_wall_segment(pose4, angles.tolist(), wall4, max_range)
    test_case("Test 4", msg4, pose4, expected4, max_range)

if __name__ == "__main__":
    main()