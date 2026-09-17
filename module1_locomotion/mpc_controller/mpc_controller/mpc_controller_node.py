"""elevation map 위에서 /patrol/goal로 가는 /cmd_vel. planner=mpc(샘플링 MPC, 기본) | heading_scan(지평 1스텝 폴백)."""

import math
import time

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


def wrap_angle_arr(a: np.ndarray) -> np.ndarray:
    """벡터판 wrap_angle: [-pi, pi)."""
    return (a + np.pi) % (2 * np.pi) - np.pi


def scan_offsets(scan_max: float, scan_step: float) -> list[float]:
    """후보 헤딩 오프셋 [0, +step, -step, +2step, -2step, …], |offset| <= scan_max. step<=0이면 [0]."""
    k_max = int(scan_max / scan_step + 1e-9) if scan_step > 0 else 0  # 35°/7° = 4.999… 방지
    return [0.0] + [s * k * scan_step for k in range(1, k_max + 1) for s in (1.0, -1.0)]


# 지평 1스텝 헤딩 스캔. planner=heading_scan 폴백·비교용
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

    for off in scan_offsets(scan_max, scan_step):
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


def inflate(occ: np.ndarray, cells: int) -> np.ndarray:
    """bool 점유 그리드를 체비쇼프 반경 cells(정사각 창)만큼 팽창. 발자국 반경 보정용."""
    if cells <= 0:
        return occ.copy()
    n = occ.shape[0]
    pad = np.zeros((n + 2 * cells, n + 2 * cells), dtype=bool)
    pad[cells : cells + n, cells : cells + n] = occ
    out = np.zeros_like(occ, dtype=bool)
    for di in range(-cells, cells + 1):
        for dj in range(-cells, cells + 1):
            out |= pad[cells + di : cells + di + n, cells + dj : cells + dj + n]
    return out


def rollout(v: np.ndarray, w: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """유니사이클 롤아웃. v, w: (M, H) 스텝별 입력. 반환 x, y, theta: (M, H) 각 스텝 후 포즈, 시작 (0,0,0).
    스텝 k 이동은 스텝 시작 헤딩 theta_{k-1}로 계산."""
    theta = np.cumsum(w * dt, axis=1)
    theta_prev = np.concatenate([np.zeros((v.shape[0], 1)), theta[:, :-1]], axis=1)
    x = np.cumsum(v * np.cos(theta_prev) * dt, axis=1)
    y = np.cumsum(v * np.sin(theta_prev) * dt, axis=1)
    return x, y, theta


# ponytail: 샘플링 MPC(2구간 입력 729시퀀스). 동역학·접지력 MPC 필요하면 planner 값 추가해 같은 계약으로 별도 함수
def plan_mpc(
    grid: np.ndarray,
    goal_xy: tuple[float, float],
    *,
    resolution: float,
    size: float,
    v_max: float,
    w_max: float,
    half_width: float,
    obstacle_h: float,
    horizon: int,
    dt: float,
    n_w: int,
    w_turn: float,
    w_head: float = 0.5,
) -> tuple[float, float] | None:
    """로봇 프레임 goal_xy로 가는 (v, w) 첫 명령. 유니사이클 지평 horizon×dt, 2구간 (v,w) 샘플링.
    충돌 = 발자국 반경(half_width) 팽창 점유 셀 위 포즈. 정지 시퀀스 제외. 유효 시퀀스 없으면 None.
    비용 = 지평 goal 거리 평균 + w_head·|종단 heading 오차| + w_turn·Σ|w|dt."""
    n = grid.shape[0]
    occ = inflate(~np.isnan(grid) & (grid > obstacle_h), int(math.ceil(half_width / resolution)))

    vs = np.array([0.0, v_max / 2, v_max])
    ws = np.linspace(-w_max, w_max, n_w)
    V, W = np.meshgrid(vs, ws, indexing="ij")
    cand = np.stack([V.ravel(), W.ravel()], axis=1)  # (m, 2)
    m = len(cand)
    i1, i2 = np.meshgrid(np.arange(m), np.arange(m), indexing="ij")
    i1, i2 = i1.ravel(), i2.ravel()  # (m*m,)
    h1 = horizon // 2
    h2 = horizon - h1
    v_seq = np.concatenate([np.repeat(cand[i1, 0:1], h1, axis=1), np.repeat(cand[i2, 0:1], h2, axis=1)], axis=1)
    w_seq = np.concatenate([np.repeat(cand[i1, 1:2], h1, axis=1), np.repeat(cand[i2, 1:2], h2, axis=1)], axis=1)

    x, y, theta = rollout(v_seq, w_seq, dt)
    col = np.floor((x + size / 2) / resolution).astype(int)
    row = np.floor((y + size / 2) / resolution).astype(int)
    inside = (col >= 0) & (col < n) & (row >= 0) & (row < n)
    hit = np.zeros(x.shape, dtype=bool)
    hit[inside] = occ[row[inside], col[inside]]
    stationary = (cand[i1, 0] == 0.0) & (cand[i2, 0] == 0.0)
    valid = ~hit.any(axis=1) & ~stationary
    if not valid.any():
        return None

    gx, gy = goal_xy
    # 지평 평균 거리(종단 거리만 쓰면 goal 근처에서 '대기 후 전진'이 '전진 후 대기'와 동률 → 정지 명령 선택 → 영구 정지)
    # heading 항: goal이 뒤에 있으면 2 s 안엔 전진이 거리를 늘려 회전 페널티가 이김 → (0,0) 영구 정지. 회전을 보상해야 함
    head_err = np.abs(wrap_angle_arr(np.arctan2(gy - y[:, -1], gx - x[:, -1]) - theta[:, -1]))
    cost = (
        np.hypot(x - gx, y - gy).mean(axis=1)
        + w_head * head_err
        + w_turn * np.abs(w_seq).sum(axis=1) * dt
    )
    cost[~valid] = np.inf
    k = int(np.argmin(cost))
    return (float(v_seq[k, 0]), float(w_seq[k, 0]))


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
        self.declare_parameter("fall_timeout", 2.0)  # status 2 Hz. fall_recovery 죽으면 이 시간 후 주행 복귀
        self.declare_parameter("planner", "mpc")  # mpc | heading_scan
        self.declare_parameter("horizon", 10)
        self.declare_parameter("dt", 0.2)
        self.declare_parameter("n_w", 9)
        self.declare_parameter("w_turn", 0.1)
        self.declare_parameter("w_head", 0.5)
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
        self._plan_ms_sum = 0.0
        self._plan_ms_max = 0.0
        self._plan_count = 0
        self.fall_status: str | None = None
        self.fall_time = None

        self.create_subscription(PoseStamped, goal_topic, self.goal_cb, 10)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_subscription(Float32MultiArray, "/elevation_map", self.elevation_map_cb, 10)
        self.create_subscription(String, "/fall_recovery/status", self.fall_status_cb, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_timer(0.1, self._tick)
        self.get_logger().info(f"mpc_controller started (도훈) planner={self.get_parameter('planner').value}")

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
        self.fall_time = self.get_clock().now()

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
        fall_timeout = self.get_parameter("fall_timeout").value
        if not self._stale(self.fall_time, fall_timeout) and halt_for_fall(self.fall_status):
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

        planner = self.get_parameter("planner").value
        t0 = time.perf_counter()
        if planner == "mpc":
            if dist < stop_dist:
                res: tuple[float, float] | None = (0.0, 0.0)
            else:
                res = plan_mpc(
                    self.grid,
                    (dist * math.cos(goal_rel), dist * math.sin(goal_rel)),
                    resolution=resolution,
                    size=size,
                    v_max=v_max,
                    w_max=w_max,
                    half_width=half_width,
                    obstacle_h=obstacle_h,
                    horizon=self.get_parameter("horizon").value,
                    dt=self.get_parameter("dt").value,
                    n_w=self.get_parameter("n_w").value,
                    w_turn=self.get_parameter("w_turn").value,
                    w_head=self.get_parameter("w_head").value,
                )
            blocked = res is None
            v, w = blocked_cmd(goal_rel, w_max) if blocked else res  # 막히면 제자리에서 goal 쪽으로 선회
        else:
            h = pick_heading(
                self.grid, goal_rel, resolution, size, lookahead, half_width, obstacle_h, scan_max, scan_step
            )
            blocked = h is None
            v, w = blocked_cmd(goal_rel, w_max) if blocked else goal_to_cmd(dist, h, v_max, w_max, k_ang, stop_dist)
        ms = (time.perf_counter() - t0) * 1e3
        self._plan_ms_sum += ms
        self._plan_ms_max = max(self._plan_ms_max, ms)
        self._plan_count += 1
        if self._plan_count % 100 == 0:
            self.get_logger().info(
                f"{planner} 계산 평균 {self._plan_ms_sum / 100:.1f} ms, 최대 {self._plan_ms_max:.1f} ms"
            )
            self._plan_ms_sum = 0.0
            self._plan_ms_max = 0.0

        if blocked != self._blocked:
            self._blocked = blocked
            if blocked:
                self.get_logger().info("경로 막힘: 제자리 회전으로 전환")
            else:
                self.get_logger().info("경로 재탐색: 계획 재개")

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
