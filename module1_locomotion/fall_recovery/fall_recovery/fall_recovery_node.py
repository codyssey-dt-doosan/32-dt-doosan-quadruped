"""IMU로 전도를 감지하고 일어서기 시퀀스를 수행한다."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String


class FallRecoveryNode(Node):
    def __init__(self) -> None:
        super().__init__("fall_recovery")
        self.create_subscription(Imu, "/imu", self.imu_cb, 10)
        self.pub_0 = self.create_publisher(String, "/fall_recovery/status", 10)
        self.create_timer(0.5, self._tick)
        self.get_logger().info("fall_recovery started (도훈)")

    def imu_cb(self, msg: Imu) -> None:
        del msg

    def _tick(self) -> None:
        out = String(); out.data = "idle"; self.pub_0.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FallRecoveryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
