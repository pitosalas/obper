import rclpy
from rclpy.node import Node
from obstacle_perception.costmap_client import CostmapClient
from obstacle_perception.beam_checker import BeamChecker
from obstacle_perception.msg import ObstacleStatus
import math

class ObstaclePerceptionNode(Node):
    def __init__(self):
        super().__init__('obstacle_perception_node')
        self.publisher = self.create_publisher(ObstacleStatus, '/obstacle/status', 10)
        self.client = CostmapClient()
        self.beam_checker = BeamChecker(0.05, 0.0, 0.0, 100, 100)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        costmap = self.client.get_costmap()
        if costmap is None:
            return

        msg = ObstacleStatus()
        msg.front_blocked = self.beam_checker.is_direction_blocked(costmap, 0.0, 1.0)
        msg.left_blocked = self.beam_checker.is_direction_blocked(costmap, math.pi / 2, 1.0)
        msg.right_blocked = self.beam_checker.is_direction_blocked(costmap, -math.pi / 2, 1.0)
        msg.back_blocked = self.beam_checker.is_direction_blocked(costmap, math.pi, 1.0)

        # Recommended action
        if msg.front_blocked:
            if not msg.left_blocked:
                msg.recommended_direction = "left"
            elif not msg.right_blocked:
                msg.recommended_direction = "right"
            elif not msg.back_blocked:
                msg.recommended_direction = "back"
            else:
                msg.recommended_direction = "wait"
        else:
            msg.recommended_direction = ""

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstaclePerceptionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
