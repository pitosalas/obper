#!/usr/bin/env python3
# Authors: Pito Salas and ChatGPT

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class CostmapSubscriber(Node):
    def __init__(self):
        super().__init__('costmap_subscriber')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',
            self.map_callback,
            10
        )
        self.get_logger().info('Subscribed to /local_costmap/costmap')

    def map_callback(self, msg: OccupancyGrid):
        width = msg.info.width
        height = msg.info.height
        data = msg.data

        self.get_logger().info(f'Received costmap {width}x{height}')
        self.display_costmap(width, height, data)

    def display_costmap(self, width, height, data):
        def symbol_for(value):
            if value < 0 or value <= 20:
                return ' '
            elif value <= 40:
                return '.'
            elif value <= 60:
                return '+'
            elif value <= 80:
                return '*'
            elif value <= 100:
                return 'X'
            else:
                return '?'

        for y in range(height):
            row = ''
            for x in range(width):
                i = y * width + x
                row += symbol_for(data[i])
            print(row)

def main(args=None):
    rclpy.init(args=args)
    node = CostmapSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
