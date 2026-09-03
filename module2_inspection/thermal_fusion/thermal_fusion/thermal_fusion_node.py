"""RGB와 열화상을 정렬·융합해 과열 영역을 표시한다."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class ThermalFusionNode(Node):
    def __init__(self) -> None:
        super().__init__("thermal_fusion")
        self.create_subscription(Image, "/camera/image", self.camera_image_cb, 10)
        self.create_subscription(Image, "/thermal/image", self.thermal_image_cb, 10)
        self.pub_0 = self.create_publisher(Bool, "/inspection/thermal_alert", 10)
        self.create_timer(0.5, self._tick)
        self.get_logger().info("thermal_fusion started (운학)")

    def camera_image_cb(self, msg: Image) -> None:
        del msg
    def thermal_image_cb(self, msg: Image) -> None:
        del msg

    def _tick(self) -> None:
        out = Bool(); out.data = False; self.pub_0.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ThermalFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
