"""PPO 보행 정책(옵트인). Genesis에서 학습한 numpy MLP를 IMU 4틱(sim 50 Hz)마다 돌려 /walk/joint_cmd로 joint_servo에 보낸다.

정합 값(관절 순서·q_nom·kp·kd·action_scale·관측 순서·스케일·명령 범위)은 모두 설치된 sim2sim.json에서 읽고,
정책 파일에 박힌 사본과 다르면 기동을 거부한다(명령을 안 보내므로 서보 토크 0).
기동은 balance_node와 같은 램프(0~2 s 관절 PD → 2~3 s kp·kd를 정책 값으로 보간) 후 정책 주도.
"""
import math
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray

from mpc_controller import rl_policy, sim2sim
from mpc_controller.balance_node import BLEND_S, KD_START, KP_START, STANDUP_S, blend, gz_unpause, q_startup, supervise
from mpc_controller.joint_servo import pack_cmd
from mpc_controller.leg_kinematics import LIMITS, Q_NOM
from mpc_controller.legged_model import JOINTS

Q_NOM12 = np.array(Q_NOM * 4)
Q_LO = np.array([lo for lo, _ in LIMITS] * 4)
Q_HI = np.array([hi for _, hi in LIMITS] * 4)


def joints_by_name(names, pos, vel):
    """JOINTS 순서로 재정렬. 하나라도 없으면 None, 속도가 없으면 0."""
    p, v = dict(zip(names, pos)), dict(zip(names, vel))
    if any(j not in p for j in JOINTS):
        return None
    return np.array([p[j] for j in JOINTS], float), np.array([v.get(j, 0.0) for j in JOINTS], float)


def projected_gravity(quat_xyzw):
    """Rᵀ·(0, 0, −1) = −(R의 셋째 행). 노름 < 0.5이거나 비유한값이면 None."""
    q = np.asarray(quat_xyzw, float)
    n = np.linalg.norm(q)
    if not np.all(np.isfinite(q)) or n < 0.5:
        return None
    x, y, z, w = q / n
    return -np.array([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)])


def build_obs(cfg, ang_vel_B, grav_B, cmd_vw, q, qd, last_action) -> np.ndarray:
    """Genesis go2_box_env._update_observation과 같은 순서·스케일."""
    s = cfg["obs"]["scales"]
    v, w = cmd_vw
    parts = {
        "ang_vel": np.asarray(ang_vel_B, float) * s["ang_vel"],
        "projected_gravity": np.asarray(grav_B, float),
        "commands": np.array([v * s["lin_vel"], 0.0, w * s["ang_vel"]]),
        "dof_pos": (np.asarray(q, float) - np.asarray(cfg["q_nom"])) * s["dof_pos"],
        "dof_vel": np.asarray(qd, float) * s["dof_vel"],
        "last_action": np.asarray(last_action, float),
    }
    return np.concatenate([parts[k] for k in cfg["obs"]["order"]])


def cmd_or_zero(cmd_vw, age_s, cfg) -> tuple:
    c = cfg["commands"]
    if age_s > c["timeout_s"]:
        return 0.0, 0.0
    v, w = cmd_vw
    return float(np.clip(v, *c["lin_vel_x"])), float(np.clip(w, *c["ang_vel"]))


def action_to_q(cfg, action) -> np.ndarray:
    """Genesis _q_des와 같은 식: q_nom + s·clip(a)를 관절 한계로 자른다."""
    a = np.clip(action, -cfg["clip_actions"], cfg["clip_actions"])
    return np.clip(np.asarray(cfg["q_nom"]) + cfg["action_scale"] * a, Q_LO, Q_HI)


def startup_gains(t, cfg):
    """기동 램프 (q_des, kp, kd). 램프가 끝나면 None(정책 차례)."""
    if t < STANDUP_S:
        return q_startup(t), KP_START, KD_START
    if t < STANDUP_S + BLEND_S:
        lam = blend(t)
        return Q_NOM12, KP_START + (cfg["kp"] - KP_START) * lam, KD_START + (cfg["kd"] - KD_START) * lam
    return None


def rl_supervise(tilt_deg, imu_age_s, joint_age_s, finite, cfg) -> str:
    if not finite:
        return "hold"
    return supervise(0.0, tilt_deg, imu_age_s, joint_age_s, tilt_max_deg=cfg["tilt_max_deg"])


def resolve_policy(path: str, share_dir: Path) -> Path:
    """path가 비면 share_dir/policies/*.npz 중 이름순 마지막."""
    if path:
        p = Path(path)
    else:
        found = sorted((share_dir / "policies").glob("*.npz"))
        p = found[-1] if found else share_dir / "policies" / "*.npz"
    if not p.is_file():
        raise FileNotFoundError(
            f"정책 파일 없음: {p} — rl_training/export.py로 만든 .npz를 mpc_controller/policies/에 두고 다시 빌드하거나 policy:=<경로>"
        )
    return p


def status_line(mode: str, inf_ms: tuple, fail: int, servo, tilt: float) -> str:
    """servo = /joint_servo/status [sat, stale, reject, gap_p50, gap_p99, gap_max, lat_p99], None이면 0."""
    s = servo or [0.0] * 7
    return (
        f"status mode={mode} inf_ms={inf_ms[0]:.2f}/{inf_ms[1]:.2f} fail={fail} sat={int(s[0])} tilt={tilt:.2f} "
        f"servo_gap_ms={s[3]:.1f}/{s[4]:.1f}/{s[5]:.1f} servo_stale={int(s[1])} servo_reject={int(s[2])} servo_lat_ms={s[6]:.1f}"
    )


class RlNode(Node):
    def __init__(self) -> None:
        super().__init__("rl")
        p = self.declare_parameters("", [("world", "corridor"), ("policy", ""), ("start_delay", 3.0)])
        self.p = {d.name: d.value for d in p}
        self.cfg = sim2sim.load()
        try:
            path = resolve_policy(self.p["policy"], Path(get_package_share_directory("mpc_controller")))
        except FileNotFoundError as e:
            self.get_logger().error(str(e))
            raise SystemExit(1)
        self.policy, policy_cfg = rl_policy.load(path)
        bad = sim2sim.diff(policy_cfg, self.cfg)
        if bad:
            self.get_logger().error(f"정책의 sim2sim이 설치 sim2sim.json과 다름: {', '.join(bad)} — {path}")
            raise SystemExit(1)
        try:
            ok = self.policy.act(np.zeros(sim2sim.obs_dim(self.cfg))).shape == (len(JOINTS),)
        except ValueError:
            ok = False
        if not ok:
            self.get_logger().error(f"정책 입출력 차원이 ({sim2sim.obs_dim(self.cfg)} → {len(JOINTS)})이 아님 — {path}")
            raise SystemExit(1)

        self.every = self.cfg["imu_hz"] // self.cfg["policy_hz"]  # IMU 200 Hz / 4 = 정책 50 Hz
        self.q = self.qd = None
        self.t0 = self.joint_stamp = self.imu_stamp = self.cmd_stamp = None
        self.cmd = (0.0, 0.0)
        self.servo = None
        self.last_action = np.zeros(len(JOINTS))
        self.q_des, self.kp, self.kd = Q_NOM12, KP_START, KD_START
        self.k = 0  # 정책 구간 IMU 틱
        self.mode = "run"
        self.tilt = 0.0
        self.fail = 0
        self.inf_ms = deque(maxlen=200)
        self.cmd_pub = self.create_publisher(Float64MultiArray, "/walk/joint_cmd", QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 1)  # 최신만(walk_node 교훈)
        self.create_subscription(Imu, "/imu", self._imu_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)
        self.create_subscription(Float64MultiArray, "/joint_servo/status", self._servo_cb, 10)
        self.create_timer(1.0, self._status)
        threading.Timer(self.p["start_delay"], self._unpause).start()
        self.get_logger().info(f"walk started (rl) policy={path}")

    def _unpause(self) -> None:
        err = gz_unpause(self.p["world"])
        self.get_logger().error(f"gz 재개 실패: {err}") if err else self.get_logger().info("gz 재개")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _cmd_cb(self, msg: Twist) -> None:
        v, w = msg.linear.x, msg.angular.z
        if math.isfinite(v) and math.isfinite(w):
            self.cmd, self.cmd_stamp = (v, w), self._now()

    def _servo_cb(self, msg: Float64MultiArray) -> None:
        s = list(msg.data)
        if len(s) == 7 and all(math.isfinite(x) for x in s):
            self.servo = s

    def _joint_cb(self, msg: JointState) -> None:
        r = joints_by_name(msg.name, msg.position, msg.velocity)
        if r is None:
            return
        self.q, self.qd = r
        self.joint_stamp = self._now()
        if self.t0 is None:
            self.t0 = self.joint_stamp

    # ---- 200 Hz: 기동 램프 / (50 Hz) 정책 → 서보 명령 ----
    def _imu_cb(self, msg: Imu) -> None:
        o, av = msg.orientation, msg.angular_velocity
        grav = projected_gravity([o.x, o.y, o.z, o.w])
        if grav is None:
            return  # 자세 미준비·무효는 버린다 — 계속되면 서보가 50 ms 뒤 기립 자세로 버틴다
        now = self._now()
        imu_age = 0.0 if self.imu_stamp is None else now - self.imu_stamp  # 직전 IMU와의 간격
        self.imu_stamp = now
        self.tilt = math.degrees(math.acos(min(1.0, max(-1.0, -grav[2]))))
        if self.q is None:
            return
        if self.mode == "run":
            finite = self._control(now, np.array([av.x, av.y, av.z]), grav)
            self.fail += int(not finite)
            joint_age = now - self.joint_stamp
            if rl_supervise(self.tilt, imu_age, joint_age, finite, self.cfg) == "hold":
                self.mode = "hold"
                self.get_logger().warning(
                    f"hold: tilt={self.tilt:.1f} imu_age={imu_age:.2f} joint_age={joint_age:.2f} finite={int(finite)}"
                )
        if self.mode == "hold":
            self.q_des, self.kp, self.kd = Q_NOM12, KP_START, KD_START
        self.cmd_pub.publish(Float64MultiArray(data=pack_cmd(now, self.q_des, self.kp, self.kd, 0.0)))

    def _control(self, now, w_B, grav) -> bool:
        """q_des·kp·kd 갱신. 입력이나 정책 출력에 비유한값이 있으면 False(갱신 안 함)."""
        if not (np.isfinite(w_B).all() and np.isfinite(self.q).all() and np.isfinite(self.qd).all()):
            return False
        start = startup_gains(now - self.t0, self.cfg)
        if start is not None:
            self.q_des, self.kp, self.kd = start
            return True
        infer = self.k % self.every == 0
        self.k += 1
        if not infer:
            return True
        age = math.inf if self.cmd_stamp is None else now - self.cmd_stamp
        obs = build_obs(self.cfg, w_B, grav, cmd_or_zero(self.cmd, age, self.cfg), self.q, self.qd, self.last_action)
        tic = time.perf_counter()
        a = self.policy.act(obs)
        self.inf_ms.append((time.perf_counter() - tic) * 1e3)
        if not np.isfinite(a).all():
            return False
        c = self.cfg["clip_actions"]
        self.last_action = np.clip(a, -c, c)  # Genesis: 다음 관측의 last_action = clip(a), 스케일 전
        self.q_des, self.kp, self.kd = action_to_q(self.cfg, a), self.cfg["kp"], self.cfg["kd"]
        return True

    def _status(self) -> None:
        b = self.inf_ms
        inf = (float(np.mean(b)), float(np.percentile(b, 99))) if b else (0.0, 0.0)
        self.get_logger().info(status_line(self.mode, inf, self.fail, self.servo, self.tilt))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = RlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
