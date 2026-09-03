"""LiDAR/깊이 포인트로 지형 elevation grid를 갱신한다."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray


class ElevationMapNode(Node):
    def __init__(self) -> None:
        super().__init__("elevation_map")
        self.create_subscription(PointCloud2, "/points", self.points_cb, 10)
        self.pub_0 = self.create_publisher(Float32MultiArray, "/elevation_map", 10)
        self.create_timer(0.5, self._tick)
        self.get_logger().info("elevation_map started (도훈)")

    def points_cb(self, msg: PointCloud2) -> None:
        del msg

    def _tick(self) -> None:
        out = Float32MultiArray(); out.data = []; self.pub_0.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ElevationMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
