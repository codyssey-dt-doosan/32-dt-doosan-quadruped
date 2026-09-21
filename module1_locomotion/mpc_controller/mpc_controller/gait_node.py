"""다리 구동 go2 보행 (도훈, C안 옵트인) — /cmd_vel의 (v, w)를 trot 관절 목표 12개로 50 Hz 발행.

legged.launch.py가 띄운 변형 모델 전용. 균형·속도 피드백 없는 개루프: 대각 쌍(FL·RR / FR·RL)이 번갈아
발을 들고, 지지 발을 뒤로 쓸어 전진·좌우 보폭 차이로 회전한다. 남는 추종 오차는 위층(mpc)이 odom으로 닫는다.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64

from mpc_controller.legged_model import LEGS

# go2/model.sdf는 joint에 <pose>가 없어 축이 링크 박스 중심 → 링크 길이 0.20이 아니라 축 간 거리
L1, L2 = 0.18, 0.10  # thigh 축→calf 축, calf 축→발끝
CALF_MIN = -1.57
R_MAX = L1 + L2 - 1e-3
R_MIN = math.sqrt(L1 * L1 + L2 * L2 + 2 * L1 * L2 * math.cos(CALF_MIN)) + 1e-3  # calf 한계에서의 축-발 거리
DIAGONAL_PHASE = {"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5}
SIDE = {"FL": 1.0, "RL": 1.0, "FR": -1.0, "RR": -1.0}  # 좌 +y / 우 −y


def ik(x: float, d: float) -> tuple:
    """thigh 축 기준 발 목표(앞 +x, 아래 d) → (thigh, calf) rad. 도달 범위 밖은 방향 유지하고 거리만 클램프."""
    r = math.hypot(x, d)
    if r < 1e-9:
        x, d, r = 0.0, R_MIN, R_MIN
    rc = min(R_MAX, max(R_MIN, r))
    x, d, r = x * rc / r, d * rc / r, rc
    phi = math.atan2(-x, d)
    thigh = phi + math.acos((L1 * L1 + r * r - L2 * L2) / (2 * L1 * r))
    shank = phi - math.acos((L2 * L2 + r * r - L1 * L1) / (2 * L2 * r))
    return thigh, shank - thigh


def swing_height(phase: float, duty: float, lift: float) -> float:
    """보행 위상 0~1에서 발 들어올림(m). phase < duty는 지지(0), 나머지는 반사인."""
    if duty >= 1.0 or phase < duty:
        return 0.0
    return lift * math.sin(math.pi * (phase - duty) / (1.0 - duty))


def leg_depth(standup: float, swing: float, d_nominal: float) -> float:
    """기립 진행(0 곧은 다리 ~ 1 명목 자세)과 스윙 높이 → thigh 축-발 수직 거리."""
    return R_MAX + (d_nominal - R_MAX) * min(1.0, max(0.0, standup)) - swing


def stride_half(v: float, w: float, side: float, t_stance: float, turn_arm: float, x_max: float) -> float:
    """몸체 속도 (v, w) → 그 다리의 반보폭 x0(m). 발 위치의 몸체 속도 v − w·y를 지지 시간만큼 쓴다.

    turn_arm은 기하 y(0.14)가 아니라 보정값: 앞뒤 발 횡마찰이 yaw를 붙잡아 0.14로는 42%만 돈다(실측).
    보행 주파수에 따라 달라져 2 Hz에선 0.33, 기본 3 Hz에선 0.38에서 제자리 회전 100%.
    """
    x0 = (v - w * side * turn_arm) * t_stance / 2.0
    return max(-x_max, min(x_max, x0))


def foot_x(phase: float, duty: float, x0: float) -> float:
    """보행 위상 0~1에서 발 x(m, 앞 +). 지지는 +x0 → −x0로 쓸고(몸체가 앞으로), 스윙은 코사인으로 되돌린다."""
    if duty >= 1.0:
        return 0.0
    if phase < duty:
        return x0 * (1.0 - 2.0 * phase / duty)
    return -x0 * math.cos(math.pi * (phase - duty) / (1.0 - duty))


class GaitNode(Node):
    def __init__(self) -> None:
        super().__init__("gait")
        self.declare_parameter("d_nominal", 0.25)
        self.declare_parameter("lift", 0.04)
        self.declare_parameter("stride_hz", 3.0)  # 2 Hz는 v 0.5에서 보폭이 길어 기울기 8.6°(실측) → 3 Hz
        self.declare_parameter("duty", 0.5)
        self.declare_parameter("standup_s", 2.0)
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("force_trot", False)  # /cmd_vel 없이도 trot (헤드리스 실험용)
        self.declare_parameter("turn_arm", 0.38)  # 회전 보정 손잡이(마찰·게인·stride_hz 바뀌면 다시 맞출 값)
        self.declare_parameter("v_gain", 1.0)  # 전진 보정 손잡이
        self.declare_parameter("x_max", 0.10)  # 반보폭 한계(d_nominal 0.25에서 도달 한계 0.124 안쪽)
        self.cmd = (0.0, 0.0)
        self.v = 0.0
        self.w = 0.0
        self.last_cmd = None
        self.t0 = None
        self.phase = 0.0
        self.scale = 0.0
        self.dt = 0.02
        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)
        self.pubs = {
            f"{leg}_{part}_joint": self.create_publisher(Float64, f"/legged/{leg}_{part}_joint", 10)
            for leg in LEGS
            for part in ("hip", "thigh", "calf")
        }
        self.create_timer(self.dt, self._tick)
        self.get_logger().info("gait started (도훈, C안 제자리 trot)")

    def _cmd_cb(self, msg: Twist) -> None:
        self.cmd = (msg.linear.x, msg.angular.z)
        self.last_cmd = self.get_clock().now()

    def _tick(self) -> None:
        now = self.get_clock().now()
        if self.t0 is None:
            self.t0 = now
        standup = (now - self.t0).nanoseconds * 1e-9 / self.get_parameter("standup_s").value
        fresh = (
            self.last_cmd is not None
            and (now - self.last_cmd).nanoseconds * 1e-9 <= self.get_parameter("cmd_timeout").value
        )
        want_v, want_w = self.cmd if fresh and standup >= 1.0 else (0.0, 0.0)
        # 명령 점프에 발이 튀지 않게 0.3 s 시정수로 따라간다
        self.v += (want_v - self.v) * self.dt / 0.3
        self.w += (want_w - self.w) * self.dt / 0.3
        moving = abs(want_v) + abs(want_w) > 1e-3 or abs(self.v) + abs(self.w) > 0.02  # 감속 중에도 발은 계속 든다
        active = standup >= 1.0 and (self.get_parameter("force_trot").value or moving)
        # 들어올림은 0.2 s 시정수로 따라가게 해서 출발·정지 때 발이 튀지 않게
        self.scale += ((1.0 if active else 0.0) - self.scale) * self.dt / 0.2
        hz = self.get_parameter("stride_hz").value
        duty = self.get_parameter("duty").value
        if active or self.scale > 1e-3:
            self.phase = (self.phase + hz * self.dt) % 1.0
        lift = self.scale * self.get_parameter("lift").value
        v = self.get_parameter("v_gain").value * self.v
        for leg in LEGS:
            phase = (self.phase + DIAGONAL_PHASE[leg]) % 1.0
            x0 = self.scale * stride_half(
                v, self.w, SIDE[leg], duty / hz, self.get_parameter("turn_arm").value, self.get_parameter("x_max").value
            )
            depth = leg_depth(standup, swing_height(phase, duty, lift), self.get_parameter("d_nominal").value)
            thigh, calf = ik(foot_x(phase, duty, x0), depth)
            for part, value in (("hip", 0.0), ("thigh", thigh), ("calf", calf)):
                self.pubs[f"{leg}_{part}_joint"].publish(Float64(data=value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GaitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
