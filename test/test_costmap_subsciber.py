# File: test/test_costmap_subscriber.py
# Authors: Pito Salas and ChatGPT
# License: MIT
# Version: 1.0
# Last Revised: 2025-04-27

import pytest
import numpy as np
from obper.costmap_subscriber import LocalCostmapSubscriber
from nav_msgs.msg import OccupancyGrid

def create_simple_costmap(width, height, obstacle_rows):
    msg = OccupancyGrid()
    msg.info.width = width
    msg.info.height = height
    msg.info.resolution = 0.1
    msg.info.origin.position.x = 0.0
    msg.info.origin.position.y = 0.0
    data = []
    for j in range(height):
        for i in range(width):
            if j >= obstacle_rows:
                data.append(100)
            else:
                data.append(0)
    msg.data = data
    return msg

def generate_beams(num_beams, fov):
    return np.linspace(-fov/2, fov/2, num_beams)

def test_wall_ahead_180deg_scan():
    node = LocalCostmapSubscriber(cost_threshold=50)
    costmap_msg = create_simple_costmap(10, 10, obstacle_rows=5)
    node.costmap_callback(costmap_msg)
    node.get_robot_yaw = lambda: 0.0  # Facing +X

    angles = generate_beams(num_beams=9, fov=np.pi)  # 180 degrees
    widths = [0.1] * len(angles)

    distances = node.check_beams(angles, widths, max_range=2.0)

    expected_distance = 0.5  # Wall at 0.5m ahead

    for dist in distances:
        assert dist == pytest.approx(expected_distance, rel=1e-2)