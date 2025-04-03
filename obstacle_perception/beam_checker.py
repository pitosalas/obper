import math

class BeamChecker:
    def __init__(self, resolution, origin_x, origin_y, width, height):
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.width = width
        self.height = height

    def world_to_map(self, x, y):
        mx = int((x - self.origin_x) / self.resolution)
        my = int((y - self.origin_y) / self.resolution)
        return mx, my

    def is_direction_blocked(self, costmap, angle_rad, max_range_m, lethal_cost=254):
        if costmap is None:
            return False
        cost_data = costmap.data
        width = costmap.metadata.size_x
        height = costmap.metadata.size_y

        start_x = width // 2
        start_y = height // 2

        end_x = int(start_x + (max_range_m / self.resolution) * math.cos(angle_rad))
        end_y = int(start_y + (max_range_m / self.resolution) * math.sin(angle_rad))

        dx = abs(end_x - start_x)
        dy = abs(end_y - start_y)
        sx = 1 if end_x > start_x else -1
        sy = 1 if end_y > start_y else -1
        err = dx - dy

        x, y = start_x, start_y

        while 0 <= x < width and 0 <= y < height:
            idx = y * width + x
            if idx >= len(cost_data):
                break
            if cost_data[idx] >= lethal_cost:
                return True

            if x == end_x and y == end_y:
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

        return False
