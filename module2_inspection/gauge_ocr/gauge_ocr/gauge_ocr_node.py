"""카메라 영상에서 게이지를 찾아 값을 판독하고, 점검 위치·상태를 알린다."""

from collections import deque
import math

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String

from gauge_ocr.make_viewpoints import compute_viewpoints
from gauge_ocr.naming import current_gauge_name
from gauge_ocr.overlay import draw_overlay
from gauge_ocr.reader import GaugeSpec, read_gauge
from inspection_common.state_msg import format_state
from inspection_common.world_map import body_height_for, load_world, resolve_world

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class GaugeOcrNode(Node):
    def __init__(self) -> None:
        super().__init__("gauge_ocr")
        self.declare_parameter("world", "corridor")
        self.declare_parameter("world_file", "")          # 비우면 설치된 simulation 패키지에서 찾음
        self.declare_parameter("models_path", "")
        self.declare_parameter("locomotion", "velocity")  # velocity | legged → 몸통 높이
        self.declare_parameter("phi_min", 225.0)
        self.declare_parameter("sweep", 270.0)
        self.declare_parameter("v_min", 0.0)
        self.declare_parameter("v_max", 10.0)
        self.declare_parameter("window", 15)              # 중앙값 프레임 수
        self.declare_parameter("stop_speed", 0.05)        # m/s 미만이면 정차
        self.declare_parameter("require_stop", True)      # False면 항상 판독 (bag·단독 테스트용)
        self.declare_parameter("publish_debug", True)
        self.declare_parameter("margin_deg", 5.0)         # 점검 위치 계산
        self.declare_parameter("arrive_tol", 0.4)
        self.declare_parameter("robot_radius", 0.35)
        self.declare_parameter("min_dial_px", 20.0)

        self._bridge = CvBridge()
        self._buf = deque(maxlen=self.get_parameter("window").value)
        self._speed = 0.0
        self._xy = None
        self._status = ""
        self._last_conf = 0.0
        self._viewpoints = []
        self._no_map = False

        self.create_subscription(
            Image, "/camera/image", self.camera_image_cb, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_subscription(String, "/mission/status", self.status_cb, 10)
        self.pub_0 = self.create_publisher(Float32, "/inspection/gauge", 10)
        self.pub_debug = self.create_publisher(
            Image, "/inspection/gauge_debug", qos_profile_sensor_data
        )
        self.pub_vp = self.create_publisher(PoseArray, "/inspection/gauge_viewpoints", LATCHED)
        self.pub_state = self.create_publisher(String, "/inspection/gauge_state", 10)
        self._load_viewpoints()
        self.create_timer(0.5, self._tick)
        self.get_logger().info("gauge_ocr started (운학)")

    def _p(self, name: str):
        return self.get_parameter(name).value

    def _spec(self) -> GaugeSpec:
        return GaugeSpec(self._p("phi_min"), self._p("sweep"), self._p("v_min"), self._p("v_max"))

    def _load_viewpoints(self) -> None:
        """시뮬 맵에서 점검 위치를 계산해 한 번 발행(latched). 실패해도 판독은 계속."""
        try:
            wm = load_world(*resolve_world(self._p("world"), self._p("world_file"), self._p("models_path")))
        except Exception as e:  # noqa: BLE001 — 맵이 없어도 판독은 해야 한다
            self.get_logger().warn(f"맵을 못 읽어 점검 위치 기능을 끕니다: {e}")
            self._no_map = True
            return
        self._viewpoints = compute_viewpoints(
            wm, body_height_for(self._p("locomotion")), margin_deg=self._p("margin_deg"),
            arrive_tol=self._p("arrive_tol"), robot_radius=self._p("robot_radius"),
            min_dial_px=self._p("min_dial_px"),
        )
        msg = PoseArray()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        for v in self._viewpoints:
            if not v.ok:
                self.get_logger().warn(f"{v.name}: 점검 위치 불가 ({v.reason})")
                continue
            p = Pose()
            p.position.x, p.position.y = v.x, v.y
            p.orientation.z, p.orientation.w = math.sin(v.yaw / 2), math.cos(v.yaw / 2)
            msg.poses.append(p)
        self.pub_vp.publish(msg)

    def _stopped(self) -> bool:
        """정차 중에만 판독. 게이지가 화면에 없으면 read_gauge가 NaN을 내므로 위치는 따로 안 본다."""
        if not self._p("require_stop"):
            return True
        return self._speed < self._p("stop_speed")

    def odom_cb(self, msg: Odometry) -> None:
        v = msg.twist.twist.linear
        self._speed = math.hypot(v.x, v.y)
        p = msg.pose.pose.position
        self._xy = (p.x, p.y)

    def status_cb(self, msg: String) -> None:
        self._status = msg.data

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
        self._last_conf = float(dbg.get("conf", 0.0))
        if not math.isnan(value):
            self._buf.append(value)
        if self._p("publish_debug") and self.pub_debug.get_subscription_count() > 0:
            out = self._bridge.cv2_to_imgmsg(draw_overlay(bgr, value, dbg), encoding="bgr8")
            out.header = msg.header
            self.pub_debug.publish(out)

    def _state_text(self) -> str:
        extra = ["no_map"] if self._no_map else []
        if not self._stopped():
            return format_state("IDLE", "moving", *extra)
        name = current_gauge_name(self._status, self._xy, self._viewpoints)
        if self._buf:
            value = float(np.median(self._buf))
            return format_state("READING", name, f"{value:.2f} bar", f"conf {self._last_conf:.2f}", *extra)
        return format_state("NO_GAUGE", name, *extra)

    def _tick(self) -> None:
        self.pub_state.publish(String(data=self._state_text()))
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
