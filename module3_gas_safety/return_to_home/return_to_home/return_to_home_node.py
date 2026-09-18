"""알람/배터리/임무 종료 시 홈 포즈로 복귀한다.

복귀 규칙 (ReturnManager):
  배터리% <= return_safety_factor × 집까지 경로 길이(m) × 소모율(%/m) + battery_reserve
  또는 최대 거리·임무 시간 초과, 위험 농도, 누출원 발견 후 대기 끝, 탐색 영역 소진, /mission/status "return_home" 요청.
집까지 경로는 지나온 지점 그래프의 최단 경로 (실제로 걸어 본 곳만 이으므로 지도 없이도 안전).
  구독  /odom, /battery, /gas/concentration, /source_seeking/state, /mission/status
  발행  /return_to_home/goal (PoseStamped, 복귀 중에만), /return_to_home/status (String "상태 | 이유 | 배터리/기준 | 집까지")
        /mission/status (상태가 바뀔 때), /cmd_vel (drive_mode=cmd_vel이고 복귀 중일 때만)
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32, String

from return_to_home.drive import DriveParams, GoalDriver
from return_to_home.home_path import ReturnManager, ReturnParams
from return_to_home.params import declare_dataclass_params


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class ReturnToHomeNode(Node):
    def __init__(self) -> None:
        super().__init__("return_to_home")
        self.declare_parameter("world", "corridor")
        rate = self.declare_parameter("rate", 10.0).value
        self.drive_mode = self.declare_parameter("drive_mode", "goal").value
        self.mgr = ReturnManager(declare_dataclass_params(self, ReturnParams))
        self.driver = GoalDriver(declare_dataclass_params(self, DriveParams))
        self._odom: Odometry | None = None
        self._battery: float | None = None
        self._conc: float | None = None
        self._seeker_state = ""
        self._request = None
        self.create_subscription(String, "/mission/status", self.mission_status_cb, 10)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_subscription(BatteryState, "/battery", self.battery_cb, 10)
        self.create_subscription(Float32, "/gas/concentration", self.gas_cb, 10)
        self.create_subscription(String, "/source_seeking/state", self.seeker_cb, 10)
        self.pub_0 = self.create_publisher(PoseStamped, "/return_to_home/goal", 10)
        self.pub_status = self.create_publisher(String, "/return_to_home/status", 10)
        self.pub_mission = self.create_publisher(String, "/mission/status", 10)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10) if self.drive_mode == "cmd_vel" else None
        self.create_timer(1.0 / rate, self._tick)
        self._last_state = "IDLE"
        self.get_logger().info("return_to_home started (채현) drive=%s" % self.drive_mode)

    def mission_status_cb(self, msg: String) -> None:
        if "return_home" in msg.data and not msg.data.startswith("return_to_home"):
            self._request = "requested on /mission/status"

    def odom_cb(self, msg: Odometry) -> None:
        self._odom = msg

    def battery_cb(self, msg: BatteryState) -> None:
        self._battery = 100.0 * msg.percentage

    def gas_cb(self, msg: Float32) -> None:
        self._conc = float(msg.data)

    def seeker_cb(self, msg: String) -> None:
        self._seeker_state = msg.data.split(" ", 1)[0]

    def _tick(self) -> None:
        if self._odom is None:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        pose = self._odom.pose.pose
        x, y, yaw = pose.position.x, pose.position.y, yaw_of(pose.orientation)
        m = self.mgr
        m.update(t, x, y, self._battery, self._conc, self._seeker_state)
        if self._request and m.state == "IDLE":
            m.start(self._request)
        self._request = None
        target = m.target()
        if m.state == "RETURNING":
            if self.driver.blocked(t, x, y, yaw, target):
                m.on_blocked()
                if self.drive_mode == "cmd_vel":
                    self.driver.start_recovery(t)
            if target is not None:
                g = PoseStamped()
                g.header.stamp = self.get_clock().now().to_msg()
                g.header.frame_id = self._odom.header.frame_id or "odom"
                g.pose.position.x, g.pose.position.y = float(target[0]), float(target[1])
                a = math.atan2(target[1] - y, target[0] - x)
                g.pose.orientation.z, g.pose.orientation.w = math.sin(a / 2.0), math.cos(a / 2.0)
                self.pub_0.publish(g)
            if self.pub_cmd:
                vx, wz = self.driver.command(t, x, y, yaw, target)
                cmd = Twist()
                cmd.linear.x, cmd.angular.z = float(vx), float(wz)
                self.pub_cmd.publish(cmd)
        if m.state != self._last_state:
            if m.state == "HOME" and self.pub_cmd:
                self.pub_cmd.publish(Twist())
            text = m.reason if m.state == "HOME" else m.return_reason
            self.get_logger().warn("[%s] %s" % (m.state, text))
            self.pub_mission.publish(String(data="return_to_home: %s (%s)" % (m.state, text)))
            self._last_state = m.state
        self.pub_status.publish(String(data="%s | %s | battery %s / return at %s | home %.1f m" % (
            m.state, m.reason or "-",
            "%.1f%%" % self._battery if self._battery is not None else "n/a",
            "%.1f%%" % m.threshold if m.threshold is not None else "n/a",
            m.home_path_length())))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReturnToHomeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
