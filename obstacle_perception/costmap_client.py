from rclpy.node import Node
from nav2_msgs.msg import Costmap

class CostmapClient(Node):
    def __init__(self):
        super().__init__('costmap_client')
        self.costmap_sub = self.create_subscription(
            Costmap,
            '/local_costmap/costmap_raw',
            self.costmap_callback,
            10
        )
        self.latest_costmap = None

    def costmap_callback(self, msg):
        self.latest_costmap = msg

    def get_costmap(self):
        return self.latest_costmap
