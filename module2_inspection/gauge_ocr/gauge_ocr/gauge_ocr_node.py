"""카메라 영상에서 게이지 ROI를 잡고 지침/숫자를 판독한다."""

from collections import deque
import math

from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from gauge_ocr.overlay import draw_overlay
from gauge_ocr.reader import GaugeSpec, read_gauge


class GaugeOcrNode(Node):
    def __init__(self) -> None:
        super().__init__("gauge_ocr")
        self.declare_parameter("world", "corridor")
        self.declare_parameter("phi_min", 225.0)
        self.declare_parameter("sweep", 270.0)
        self.declare_parameter("v_min", 0.0)
        self.declare_parameter("v_max", 10.0)
        self.declare_parameter("window", 15)              # 중앙값 프레임 수
        self.declare_parameter("stop_speed", 0.05)        # m/s 미만이면 정차
        self.declare_parameter("require_stop", True)      # False면 항상 판독 (bag·단독 테스트용)
        self.declare_parameter("publish_debug", True)

        self._bridge = CvBridge()
        self._buf = deque(maxlen=self.get_parameter("window").value)
        self._speed = 0.0

        self.create_subscription(
            Image, "/camera/image", self.camera_image_cb, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.pub_0 = self.create_publisher(Float32, "/inspection/gauge", 10)
        self.pub_debug = self.create_publisher(
            Image, "/inspection/gauge_debug", qos_profile_sensor_data
        )
        self.create_timer(0.5, self._tick)
        self.get_logger().info("gauge_ocr started (운학)")

    def _spec(self) -> GaugeSpec:
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        return GaugeSpec(g("phi_min"), g("sweep"), g("v_min"), g("v_max"))

    def _stopped(self) -> bool:
        """정차 중에만 판독. 게이지가 화면에 없으면 read_gauge가 NaN을 내므로 위치는 따로 안 본다.

        patrol_path가 dwell_s를 아직 구현하지 않아 /mission/status로는 정차를 알 수 없다.
        """
        if not self.get_parameter("require_stop").value:
            return True
        return self._speed < self.get_parameter("stop_speed").value

    def odom_cb(self, msg: Odometry) -> None:
        v = msg.twist.twist.linear
        self._speed = math.hypot(v.x, v.y)

    def camera_image_cb(self, msg: Image) -> None:
        if not self._stopped():
            self._buf.clear()                             # 이동하면 이전 게이지 값을 버린다
            return
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().warn(f"cv_bridge: {e}")
            return
        value, dbg = read_gauge(bgr, self._spec())
        if not math.isnan(value):
            self._buf.append(value)
        want_debug = self.get_parameter("publish_debug").value
        if want_debug and self.pub_debug.get_subscription_count() > 0:
            out = self._bridge.cv2_to_imgmsg(draw_overlay(bgr, value, dbg), encoding="bgr8")
            out.header = msg.header
            self.pub_debug.publish(out)

    def _tick(self) -> None:
        if not self._buf:
            return                                        # 판독값 없음 → 발행 안 함 (대시보드는 마지막 값 유지)
        out = Float32()
        out.data = float(np.median(self._buf))
        self.pub_0.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GaugeOcrNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
