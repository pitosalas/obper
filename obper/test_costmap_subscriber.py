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
from obper.costmap_subscriber import LocalCostmapSubscriber

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
    bounds: tuple[tuple[float, float], tuple[float, float]]

# beam_distances_to_wall_segment: Computes beam distance to a wall segment (bounded line) with debug prints.
def beam_distances_to_wall_segment(robot_pose, beam_angles, wall):
    distances = []
    max_range = LocalCostmapSubscriber.MAX_SCAN_RANGE
    epsilon = 1e-6

    (xmin, xmax), (ymin, ymax) = wall.bounds

    for angle in beam_angles:
        # Part 1: Beam direction in world frame
        theta = robot_pose.yaw + angle
        dx = math.cos(theta)
        dy = math.sin(theta)
        x0 = robot_pose.x
        y0 = robot_pose.y

        print(f"\nAngle {angle:.2f} rad, direction=({dx:.2f}, {dy:.2f}), origin=({x0:.2f}, {y0:.2f})")

        intersections = []

        # Part 2: Check vertical intersections (x = xmin/xmax)
        for x_edge in [xmin, xmax]:
            if abs(dx) > epsilon:
                t = (x_edge - x0) / dx
                y = y0 + t * dy
                if 0 <= t <= max_range and ymin <= y <= ymax:
                    intersections.append(t)
                    print(f"  ↪ Intersects vertical x={x_edge:.2f} at t={t:.3f}, y={y:.2f}")

        # Part 3: Check horizontal intersections (y = ymin/ymax)
        for y_edge in [ymin, ymax]:
            if abs(dy) > epsilon:
                t = (y_edge - y0) / dy
                x = x0 + t * dx
                if 0 <= t <= max_range and xmin <= x <= xmax:
                    intersections.append(t)
                    print(f"  ↪ Intersects horizontal y={y_edge:.2f} at t={t:.3f}, x={x:.2f}")

        # Part 4: Select closest valid intersection or max range
        if intersections:
            t_min = min(intersections)
            print(f"  ↪ Closest intersection at t={t_min:.3f}")
            distances.append(t_min)
        else:
            print(f"  ↪ No intersection in bounds. Using max range {max_range}")
            distances.append(max_range)

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
    origin_x = msg.info.origin.position.x
    res = msg.info.resolution
    i = round((x_m - origin_x) / res)
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
    rx = rx - msg.info.origin.position.x
    ry = ry - msg.info.origin.position.y
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
def test_case(name: str, costmap_msg: OccupancyGrid, pose: Pose, expected: list[float]):
    print(f'\n===== {name} =====')
    checker = BeamChecker(
        resolution=costmap_msg.info.resolution,
        origin_x=costmap_msg.info.origin.position.x,
        origin_y=costmap_msg.info.origin.position.y,
        width=costmap_msg.info.width,
        height=costmap_msg.info.height,
        costmap=costmap_msg.data
    )
    angles = np.linspace(-math.pi / 2, math.pi / 2, len(expected))
    widths = [0.1] * len(angles)
    print_costmap_with_robot(costmap_msg, pose.x, pose.y)
    actual = checker.check_beams(pose.x, pose.y, pose.yaw, angles, widths)
    assert_all_beams(expected, actual)

# main: define and run test cases
def main():
    map_w, map_h, res = 3.0, 3.0, 0.05
    wall_bounds = ((0.0, map_w), (0.0, map_h))
    angles = np.linspace(-math.pi / 2, math.pi / 2, 9)

    # # Test 1
    # pose1 = Pose(2.0, 1.5, 0.0)
    # msg1 = create_empty_costmap(map_w, map_h, res)
    # add_vertical_wall(msg1, 3.0)
    # wall1 = WallSegment(3.0, 0.0, math.pi / 2, wall_bounds)
    # expected1 = beam_distances_to_wall_segment(pose1, angles.tolist(), wall1)
    # test_case("Test 1", msg1, pose1, expected1)

    # # Test 2
    # pose2 = Pose(1.5, 1.0, math.pi / 2)
    # msg2 = create_empty_costmap(map_w, map_h, res)
    # add_horizontal_wall(msg2, 2.0)
    # wall2 = WallSegment(0.0, 2.0, 0.0, wall_bounds)
    # expected2 = beam_distances_to_wall_segment(pose2, angles.tolist(), wall2)
    # test_case("Test 2", msg2, pose2, expected2)

    # # Test 3
    # pose3 = Pose(1.5, 1.5, 0.0)
    # msg3 = create_empty_costmap(map_w, map_h, res)
    # add_box_around(msg3, pose3.x, pose3.y, 0.10)
    # expected3 = [0.05] * len(angles)
    # test_case("Test 3", msg3, pose3, expected3)

    # # Test 4: diagonal wall (/ direction)
    # pose4 = Pose(0.0, 1.0, 0.0)
    # msg4 = create_empty_costmap(map_w, map_h, res)
    # add_diagonal_wall(msg4, '/')
    # wall4 = WallSegment(
    #     x0=map_w / 2,
    #     y0=map_h / 2,
    #     angle=-math.pi * 3 / 4,
    #     bounds=((0.0, map_w), (0.0, map_h))
    # )
    # expected4 = beam_distances_to_wall_segment(pose4, angles.tolist(), wall4)
    # test_case("Test 4", msg4, pose4, expected4)

    # Test 5: origin shifted to (-1.0, 0.0)
    pose5 = Pose(0.5, 0.5, 0.0)  # robot is at (1.5, 1.5) in world coords
    msg5 = create_empty_costmap(map_w, map_h, res)
    msg5.info.origin.position.x = -1.0  # shift origin left
    add_vertical_wall(msg5, 1.5)  # wall at x=1.5 in world
    wall5 = WallSegment(x0=2.0, y0=0.0, angle=math.pi / 2, bounds=wall_bounds)
    expected5 = beam_distances_to_wall_segment(pose5, angles.tolist(), wall5)
    test_case("Test 5", msg5, pose5, expected5)

    # # Test 5a: same as Test 5 but with origin at (0.0, 0.0)
    # pose5a = Pose(1.0, 1.5, 0.0)
    # msg5a = create_empty_costmap(map_w, map_h, res)
    # # origin is default (0.0, 0.0)
    # add_vertical_wall(msg5a, 2.0)
    # wall5a = WallSegment(x0=2.0, y0=0.0, angle=math.pi / 2, bounds=wall_bounds)
    # expected5a = beam_distances_to_wall_segment(pose5a, angles.tolist(), wall5a)
    # test_case("Test 5a", msg5a, pose5a, expected5a)

    # # Test 6: origin shifted to (-1.0, -1.0)
    # pose6 = Pose(0.5, 0.5, math.pi / 2)  # robot at (0.5, 0.5) world
    # msg6 = create_empty_costmap(map_w, map_h, res)
    # msg6.info.origin.position.x = -1.0
    # msg6.info.origin.position.y = -1.0
    # add_horizontal_wall(msg6, 2.0)      # wall at y=2.0 in world
    # wall6 = WallSegment(x0=0.0, y0=2.0, angle=0.0, bounds=wall_bounds)
    # expected6 = beam_distances_to_wall_segment(pose6, angles.tolist(), wall6)
    # test_case("Test 6", msg6, pose6, expected6)

    # # Test 6a: same as Test 6 but with origin at (0.0, 0.0)
    # pose6a = Pose(0.5, 0.5, math.pi / 2)
    # msg6a = create_empty_costmap(map_w, map_h, res)
    # # origin is default (0.0, 0.0)
    # add_horizontal_wall(msg6a, 2.0)
    # wall6a = WallSegment(x0=0.0, y0=2.0, angle=0.0, bounds=wall_bounds)
    # expected6a = beam_distances_to_wall_segment(pose6a, angles.tolist(), wall6a)
    # test_case("Test 6a", msg6a, pose6a, expected6a)


if __name__ == "__main__":
    main()
