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

def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = CostmapClient()

    def print_costmap():
        cm = node.get_costmap()
        if cm:
            node.get_logger().info(f"Received costmap: {cm.metadata.size_x} x {cm.metadata.size_y}")
        else:
            node.get_logger().info("Waiting for costmap...")

    node.create_timer(1.0, print_costmap)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()