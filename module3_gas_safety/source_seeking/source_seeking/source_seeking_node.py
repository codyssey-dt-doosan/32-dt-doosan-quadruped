"""농도 구배로 누출원을 추적한다."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32


class SourceSeekingNode(Node):
    def __init__(self) -> None:
        super().__init__("source_seeking")
        self.create_subscription(Float32, "/gas/concentration", self.gas_concentration_cb, 10)
        self.pub_0 = self.create_publisher(PoseStamped, "/source_seeking/goal", 10)
        self.create_timer(0.5, self._tick)
        self.get_logger().info("source_seeking started (채현)")

    def gas_concentration_cb(self, msg: Float32) -> None:
        del msg

    def _tick(self) -> None:
        self.pub_0.publish(PoseStamped())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SourceSeekingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
