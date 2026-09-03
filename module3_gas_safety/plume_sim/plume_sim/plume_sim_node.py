"""탱크 누출원의 농도장을 가상 플랜트에 올린다."""

from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32


class PlumeSimNode(Node):
    """가우시안 플룸. 공장 탱크 부근 (8, -6)을 누출원으로 둔다."""

    def __init__(self) -> None:
        super().__init__("plume_sim")
        self.declare_parameter("source_x", 8.0)
        self.declare_parameter("source_y", -6.0)
        self.declare_parameter("sigma", 3.0)
        self.declare_parameter("peak", 100.0)
        self._xy = (0.0, 0.0)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self._pub = self.create_publisher(Float32, "/gas/concentration", 10)
        self.create_timer(0.2, self._tick)
        self.get_logger().info("plume_sim started (채현)")

    def odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._xy = (p.x, p.y)

    def _tick(self) -> None:
        sx = self.get_parameter("source_x").value
        sy = self.get_parameter("source_y").value
        sigma = self.get_parameter("sigma").value
        peak = self.get_parameter("peak").value
        dx = self._xy[0] - sx
        dy = self._xy[1] - sy
        conc = peak * math.exp(-(dx * dx + dy * dy) / (2.0 * sigma * sigma))
        msg = Float32()
        msg.data = float(conc)
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlumeSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
