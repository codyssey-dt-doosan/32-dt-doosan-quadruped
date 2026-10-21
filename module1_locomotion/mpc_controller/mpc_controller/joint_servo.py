"""관절 서보(C안 ⑤-3, 옵트인 walk.launch.py 전용). walk_node 명령 (q_des, kp, kd, τ_ff)×12를 1 kHz /joint_states마다
τ = kp(q_des−q) − kd·q̇ + τ_ff(±TAU_MAX)로 발행. 판단 없음 — 별도 프로세스로 떼어 MPC·추정 부하(GIL)와 분리.

명령이 sim 시간 cmd_timeout(50 ms) 넘게 끊기면 기립 자세 PD로 버틴다. 첫 명령 전엔 아무것도 발행하지 않는다(토크 0).
"""
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray

from mpc_controller.balance_node import KD_START, KP_START, TAU_MAX
from mpc_controller.leg_kinematics import Q_NOM
from mpc_controller.legged_model import JOINTS, ros_force_topic

N = 12
CMD_LEN = 1 + 4 * N
Q_NOM12 = np.array(Q_NOM * 4)


def pack_cmd(t, q_des, kp, kd, tau_ff) -> list:
    """[t, q_des×12, kp×12, kd×12, τ_ff×12] — 스칼라는 12로 broadcast."""
    parts = [np.broadcast_to(np.asarray(a, float), (N,)) for a in (q_des, kp, kd, tau_ff)]
    return [float(t), *(float(x) for x in np.concatenate(parts))]


def unpack_cmd(data):
    """49개 → (t, q_des, kp, kd, tau_ff). 길이가 다르거나 비유한값이 있으면 None."""
    a = np.asarray(data, float)
    if a.shape != (CMD_LEN,) or not np.all(np.isfinite(a)):
        return None
    return float(a[0]), a[1:13], a[13:25], a[25:37], a[37:49]


def servo_torque(cmd, q, qd, tau_max: float = TAU_MAX) -> np.ndarray:
    _, q_des, kp, kd, tau_ff = cmd
    return np.clip(kp * (q_des - q) - kd * qd + tau_ff, -tau_max, tau_max)


def is_stale(t_cmd: float, now: float, timeout: float = 0.05) -> bool:
    return now - t_cmd > timeout + 1e-12


def hold_cmd(now: float):
    return now, Q_NOM12, np.full(N, KP_START), np.full(N, KD_START), np.zeros(N)


def servo_step(cmd, now: float, q, qd, timeout: float = 0.05):
    """(τ 또는 None, 끊김 여부). 명령 없으면 (None, False) — 발행 안 함."""
    if cmd is None:
        return None, False
    stale = is_stale(cmd[0], now, timeout)
    return servo_torque(hold_cmd(now) if stale else cmd, q, qd), stale


def gap_summary(xs) -> tuple:
    """s 단위 목록 → (p50, p99, max) ms."""
    if len(xs) == 0:
        return 0.0, 0.0, 0.0
    a = np.asarray(xs, float) * 1000
    return float(np.median(a)), float(np.percentile(a, 99)), float(a.max())


class JointServo(Node):
    def __init__(self) -> None:
        super().__init__("joint_servo")
        self.timeout = float(self.declare_parameter("cmd_timeout", 0.05).value)
        self.cmd = None
        self.was_stale = False
        self.stats = {"sat": 0, "stale": 0, "reject": 0}
        self.gaps, self.lats = deque(maxlen=2000), deque(maxlen=2000)
        self.wall = None
        self.pubs = [self.create_publisher(Float64, ros_force_topic(j), 10) for j in JOINTS]
        self.status_pub = self.create_publisher(Float64MultiArray, "/joint_servo/status", 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Float64MultiArray, "/walk/joint_cmd", self._cmd_cb, qos)
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 1)  # 최신만(깊이 10이면 낡은 상태로 PD, walk_node 교훈)
        self.create_timer(1.0, self._status)
        self.get_logger().info("joint_servo started (도훈, C안 ⑤-3)")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _cmd_cb(self, msg: Float64MultiArray) -> None:
        cmd = unpack_cmd(msg.data)
        if cmd is None:
            self.stats["reject"] += 1
            return
        self.cmd = cmd
        self.lats.append(self._now() - cmd[0])

    def _joint_cb(self, msg: JointState) -> None:
        pos, vel = dict(zip(msg.name, msg.position)), dict(zip(msg.name, msg.velocity))
        if any(j not in pos for j in JOINTS):
            return
        wall = time.monotonic()
        if self.wall is not None:
            self.gaps.append(wall - self.wall)
        self.wall = wall
        q = np.array([pos[j] for j in JOINTS])
        qd = np.array([vel.get(j, 0.0) for j in JOINTS])
        now = self._now()
        tau, stale = servo_step(self.cmd, now, q, qd, self.timeout)
        if tau is None:
            return
        if stale and not self.was_stale:
            self.stats["stale"] += 1
            self.get_logger().warning(f"명령 끊김 {(now - self.cmd[0]) * 1000:.0f} ms → 기립 자세 PD hold")
        self.was_stale = stale
        self.stats["sat"] += int(np.any(np.abs(tau) >= TAU_MAX))
        for pub, u in zip(self.pubs, tau):
            pub.publish(Float64(data=float(u)))

    def _status(self) -> None:
        g, lat = gap_summary(self.gaps), gap_summary(self.lats)
        s = self.stats
        self.get_logger().info(
            f"servo status gap_ms={g[0]:.1f}/{g[1]:.1f}/{g[2]:.1f} lat_ms={lat[1]:.1f} stale={s['stale']} reject={s['reject']} sat={s['sat']}"
        )
        self.status_pub.publish(Float64MultiArray(data=[float(s["sat"]), float(s["stale"]), float(s["reject"]), *g, lat[1]]))
        self.gaps.clear()
        self.lats.clear()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
