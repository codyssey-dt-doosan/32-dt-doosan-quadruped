"""CoM/접지력 MPC로 cmd_vel 또는 관절 명령을 낸다."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray


class MpcControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("mpc_controller")
        self.create_subscription(Float32MultiArray, "/elevation_map", self.elevation_map_cb, 10)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.pub_0 = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_timer(0.5, self._tick)
        self.get_logger().info("mpc_controller started (도훈)")

    def elevation_map_cb(self, msg: Float32MultiArray) -> None:
        del msg
    def odom_cb(self, msg: Odometry) -> None:
        del msg

    def _tick(self) -> None:
        self.pub_0.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MpcControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
