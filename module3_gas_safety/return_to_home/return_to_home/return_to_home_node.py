"""알람/배터리/임무 종료 시 홈 포즈로 복귀한다."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class ReturnToHomeNode(Node):
    def __init__(self) -> None:
        super().__init__("return_to_home")
        self.create_subscription(String, "/mission/status", self.mission_status_cb, 10)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.pub_0 = self.create_publisher(PoseStamped, "/return_to_home/goal", 10)
        self.create_timer(0.5, self._tick)
        self.get_logger().info("return_to_home started (채현)")

    def mission_status_cb(self, msg: String) -> None:
        del msg
    def odom_cb(self, msg: Odometry) -> None:
        del msg

    def _tick(self) -> None:
        self.pub_0.publish(PoseStamped())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReturnToHomeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
