"""elevation map 전방 헤딩 스캔으로 장애물 피해 /patrol/goal로 간다. 진짜 MPC 아님(향후)."""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, String


def wrap_angle(a: float) -> float:
    """(-pi, pi]로 감음."""
    r = a % (2 * math.pi)
    if r > math.pi:
        r -= 2 * math.pi
    return r


# ponytail: 지평 1스텝 헤딩 스캔. 운동학 MPC 필요하면 이 함수만 receding-horizon 최적화로 교체
def pick_heading(
    grid: np.ndarray,
    goal_rel: float,
    resolution: float,
    size: float,
    lookahead: float,
    half_width: float,
    obstacle_h: float,
    scan_max: float,
    scan_step: float,
) -> float | None:
    """로봇 프레임 헤딩(rad) 반환. goal_rel에 가까운 순서로 후보 검사, 전부 막히면 None.

    후보 = goal_rel + [0, +step, -step, +2step, -2step, ...] (|offset| <= scan_max).
    후보 h가 막힘 = 그리드 셀 중 (값이 NaN 아니고) 값 > obstacle_h 이고
      along = cx*cos(h) + cy*sin(h) 가 [0, lookahead] 이고
      |lateral| = |-cx*sin(h) + cy*cos(h)| <= half_width
    인 셀이 하나라도 있음. (cx, cy)는 셀 중심: x=(col+0.5)*res-size/2, y=(row+0.5)*res-size/2.
    """
    n = grid.shape[0]
    centers = (np.arange(n) + 0.5) * resolution - size / 2
    cx, cy = np.meshgrid(centers, centers)  # cx: col(전방 x), cy: row(좌우 y)
    occ = ~np.isnan(grid) & (grid > obstacle_h)
    cx_occ, cy_occ = cx[occ], cy[occ]

    k_max = int(scan_max / scan_step) if scan_step > 0 else 0
    offsets = [0.0] + [s * k * scan_step for k in range(1, k_max + 1) for s in (1.0, -1.0)]

    for off in offsets:
        h = wrap_angle(goal_rel + off)
        along = cx_occ * math.cos(h) + cy_occ * math.sin(h)
        lateral = -cx_occ * math.sin(h) + cy_occ * math.cos(h)
        blocked = np.any((along >= 0) & (along <= lookahead) & (np.abs(lateral) <= half_width))
        if not blocked:
            return h
    return None


def goal_to_cmd(
    dist: float, heading: float, v_max: float, w_max: float, k_ang: float, stop_dist: float
) -> tuple[float, float]:
    """(v, w). dist < stop_dist → (0, 0).
    w = clip(k_ang*heading, -w_max, w_max), v = v_max*max(0, cos(heading))."""
    if dist < stop_dist:
        return (0.0, 0.0)
    w = max(-w_max, min(w_max, k_ang * heading))
    v = v_max * max(0.0, math.cos(heading))
    return (v, w)


def blocked_cmd(goal_rel: float, w_max: float) -> tuple[float, float]:
    """헤딩 스캔이 전부 막혔을 때 제자리 선회 명령. goal_rel 부호 방향으로 w_max 선회(0이면 +)."""
    w = math.copysign(w_max, goal_rel if goal_rel != 0.0 else 1.0)
    return (0.0, w)


HALT_STATES = frozenset({"fallen", "recovering", "failed"})


def halt_for_fall(status: str | None) -> bool:
    """fall_recovery 상태가 전도·회복 중이면 True. None(미수신)은 주행."""
    return status in HALT_STATES


class MpcControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("mpc_controller")
        self.declare_parameter("world", "corridor")  # launch가 넘김, 사용 안 함
        self.declare_parameter("goal_topic", "/patrol/goal")
        self.declare_parameter("v_max", 0.5)
        self.declare_parameter("w_max", 1.0)
        self.declare_parameter("k_ang", 1.5)
        self.declare_parameter("stop_dist", 0.3)
        self.declare_parameter("lookahead", 1.5)
        self.declare_parameter("half_width", 0.35)
        self.declare_parameter("obstacle_h", 0.15)
        self.declare_parameter("scan_max_deg", 60.0)
        self.declare_parameter("scan_step_deg", 10.0)
        self.declare_parameter("timeout", 1.0)
        self.declare_parameter("resolution", 0.1)
        self.declare_parameter("size", 4.0)

        goal_topic = self.get_parameter("goal_topic").value
        self.goal_msg: PoseStamped | None = None
        self.goal_time = None
        self.odom_msg: Odometry | None = None
        self.odom_time = None
        self.grid: np.ndarray | None = None
        self.grid_time = None
        self._blocked = False
        self._grid_warned = False
        self.fall_status: str | None = None

        self.create_subscription(PoseStamped, goal_topic, self.goal_cb, 10)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_subscription(Float32MultiArray, "/elevation_map", self.elevation_map_cb, 10)
        self.create_subscription(String, "/fall_recovery/status", self.fall_status_cb, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_timer(0.1, self._tick)
        self.get_logger().info("mpc_controller started (도훈)")

    def goal_cb(self, msg: PoseStamped) -> None:
        self.goal_msg = msg
        self.goal_time = self.get_clock().now()

    def odom_cb(self, msg: Odometry) -> None:
        self.odom_msg = msg
        self.odom_time = self.get_clock().now()

    def fall_status_cb(self, msg: String) -> None:
        if halt_for_fall(msg.data) != halt_for_fall(self.fall_status):
            self.get_logger().info(
                f"fall_recovery {msg.data}: " + ("정지" if halt_for_fall(msg.data) else "주행 재개")
            )
        self.fall_status = msg.data

    def elevation_map_cb(self, msg: Float32MultiArray) -> None:
        try:
            rows, cols = msg.layout.dim[0].size, msg.layout.dim[1].size
            grid = np.array(msg.data, dtype=np.float32).reshape(rows, cols)
        except Exception as e:
            if not self._grid_warned:
                self._grid_warned = True
                self.get_logger().warn(f"elevation_map 메시지 형식 오류, 무시: {e}")
            return  # grid/grid_time 갱신 안 함 → 기존 timeout 경로가 정지시킴
        self.grid = grid
        self.grid_time = self.get_clock().now()

    def _stale(self, t, timeout: float) -> bool:
        if t is None:
            return True
        return (self.get_clock().now() - t).nanoseconds / 1e9 > timeout

    def _tick(self) -> None:
        if halt_for_fall(self.fall_status):
            self.pub.publish(Twist())
            return
        timeout = self.get_parameter("timeout").value
        if (
            self._stale(self.goal_time, timeout)
            or self._stale(self.odom_time, timeout)
            or self._stale(self.grid_time, timeout)
        ):
            self.pub.publish(Twist())
            return

        resolution = self.get_parameter("resolution").value
        size = self.get_parameter("size").value
        lookahead = self.get_parameter("lookahead").value
        half_width = self.get_parameter("half_width").value
        obstacle_h = self.get_parameter("obstacle_h").value
        scan_max = math.radians(self.get_parameter("scan_max_deg").value)
        scan_step = math.radians(self.get_parameter("scan_step_deg").value)
        v_max = self.get_parameter("v_max").value
        w_max = self.get_parameter("w_max").value
        k_ang = self.get_parameter("k_ang").value
        stop_dist = self.get_parameter("stop_dist").value

        x = self.odom_msg.pose.pose.position.x
        y = self.odom_msg.pose.pose.position.y
        q = self.odom_msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        gx = self.goal_msg.pose.position.x
        gy = self.goal_msg.pose.position.y
        dist = math.hypot(gx - x, gy - y)
        goal_rel = wrap_angle(math.atan2(gy - y, gx - x) - yaw)

        h = pick_heading(
            self.grid, goal_rel, resolution, size, lookahead, half_width, obstacle_h, scan_max, scan_step
        )

        blocked = h is None
        if blocked != self._blocked:
            self._blocked = blocked
            if blocked:
                self.get_logger().info("경로 막힘: 제자리 회전으로 전환")
            else:
                self.get_logger().info("경로 재탐색: 헤딩 스캔 재개")

        if h is None:
            v, w = blocked_cmd(goal_rel, w_max)  # 막히면 제자리에서 goal 쪽으로 선회
        else:
            v, w = goal_to_cmd(dist, h, v_max, w_max, k_ang, stop_dist)

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MpcControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
