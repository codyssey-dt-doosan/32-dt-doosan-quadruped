"""힘 제어 기립(C안 ③, 옵트인). 접지력 분배 QP → τ = −Jᵀf + 관절 PD → ApplyJointForce. balance.launch.py 전용.

기동: 0~2 s 관절 PD로 곧은 다리 → 명목 자세(스파이크 기준선), 2~3 s 블렌딩(λ smoothstep, Kp_j 120→0),
이후 QP 주도(감쇠만). IMU 콜백(200 Hz)이 추정→QP→τ_ff, joint_states 콜백(1 kHz)이 PD와 합쳐 발행.
프레임: B 몸체, G 중력 정렬(yaw 추종). 단일 스레드 executor라 공유 상태 경쟁 없음.
"""
import math
import subprocess
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64

from mpc_controller.body_estimate import com_target, estimate, yaw_aligned
from mpc_controller.force_qp import ForceQP
from mpc_controller.leg_kinematics import Q_NOM
from mpc_controller.legged_model import JOINTS, ros_force_topic

STANDUP_S, BLEND_S = 2.0, 1.0
KP_START, KD_START, KD_RUN = 120.0, 2.0, 1.0
TAU_MAX = 30.0
G = 9.8
Q_NOM12 = np.array(Q_NOM * 4)


def blend(t: float) -> float:
    s = min(1.0, max(0.0, (t - STANDUP_S) / BLEND_S))
    return s * s * (3.0 - 2.0 * s)


def gains(lam: float) -> tuple:
    return KP_START * (1.0 - lam), KD_START * (1.0 - lam) + KD_RUN * lam


def q_startup(t: float) -> np.ndarray:
    return Q_NOM12 * min(1.0, max(0.0, t / STANDUP_S))


def supervise(qp_fail_s, tilt_deg, imu_age_s, joint_age_s, *, hold_s=0.05, tilt_max_deg=45.0, timeout_s=0.1) -> str:
    bad = qp_fail_s > hold_s or tilt_deg > tilt_max_deg or imu_age_s > timeout_s or joint_age_s > timeout_s
    return "hold" if bad else "run"


def joint_torque(lam, kp, kd, q_des, q, qd, tau_ff, tau_max=TAU_MAX) -> np.ndarray:
    return np.clip(kp * (q_des - q) - kd * qd + lam * np.asarray(tau_ff), -tau_max, tau_max)


class BalanceNode(Node):
    def __init__(self) -> None:
        super().__init__("balance")
        p = self.declare_parameters(
            "",
            [
                ("world", "corridor"),
                ("start_delay", 3.0),  # 벽시계 s 뒤 gz 재개(노드가 먼저 떠 있어야 낙하 순간부터 PD)
                ("enable_qp", False),
                ("foot_r", 0.02),
                ("mass", 13.2),
                ("inertia", [0.22, 0.48, 0.58]),
                ("com_offset", [0.0, 0.0, -0.04]),
                ("height", 0.28),  # CoM 높이(법선). 0.266은 calf 한계(−1.57)에서의 최소 높이 = 하드스톱 — 목표로 쓰면 안 됨
                ("com_shift", 1.0),
                ("kp_pos", [50.0, 50.0, 100.0]),
                ("kd_pos", [5.0, 5.0, 10.0]),
                ("kp_rot", [100.0, 100.0, 50.0]),
                ("kd_rot", [5.0, 5.0, 5.0]),
                ("pitch_offset", 0.0),
                ("roll_offset", 0.0),
                ("mu", 0.5),
                ("f_min", 2.0),
                ("f_max", 120.0),
                ("weights", [1.0, 1.0, 2.0, 10.0, 10.0, 5.0]),
                ("alpha", 1e-3),
                ("beta", 1e-2),
            ],
        )
        self.p = {d.name: d.value for d in p}
        self.qp = ForceQP(self.p["mu"], self.p["f_min"], self.p["f_max"], self.p["weights"], self.p["alpha"], self.p["beta"])
        self.I_B = np.diag(self.p["inertia"])
        self.q = self.qd = None
        self.t0 = None
        self.joint_stamp = self.imu_stamp = None
        self.tau_ff = np.zeros(12)
        self.f_prev = np.zeros(12)
        self.qp_fail_since = None
        self.tilt_deg = 0.0
        self.mode = "run"
        self.stats = {"qp_ms": [], "fail": 0, "sat": 0, "sum_fz": 0.0, "sum_fn": 0.0, "sum_ft": 0.0}
        self.pubs = [self.create_publisher(Float64, ros_force_topic(j), 10) for j in JOINTS]
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        self.create_subscription(Imu, "/imu", self._imu_cb, 10)
        self.create_timer(1.0, self._status)
        threading.Timer(self.p["start_delay"], self._unpause).start()
        self.get_logger().info("balance started (도훈, C안 힘 제어 기립)")

    # ---- 시뮬 재개 (fall_recovery.reset_pose와 같은 subprocess 방식) ----
    def _unpause(self) -> None:
        cmd = [
            "gz", "service", "-s", f"/world/{self.p['world']}/control", "--reqtype", "gz.msgs.WorldControl",
            "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", "pause: false",
        ]
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=5.0)
        except (subprocess.TimeoutExpired, OSError) as e:
            self.get_logger().error(f"gz 재개 실패: {e}")
            return
        if r.returncode != 0:
            self.get_logger().error(f"gz 재개 실패: rc={r.returncode} out={r.stdout.strip()} err={r.stderr.strip()}")
            return
        self.get_logger().info("gz 재개")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _age(self, stamp) -> float:
        return 0.0 if stamp is None else self._now() - stamp

    # ---- 1 kHz: 관절 PD + τ_ff ----
    def _joint_cb(self, msg: JointState) -> None:
        pos, vel = dict(zip(msg.name, msg.position)), dict(zip(msg.name, msg.velocity))
        if any(j not in pos for j in JOINTS):
            return
        self.q = np.array([pos[j] for j in JOINTS])
        self.qd = np.array([vel.get(j, 0.0) for j in JOINTS])
        now = self._now()
        self.joint_stamp = now
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0
        lam = blend(t) if self.mode == "run" and self.p["enable_qp"] else 0.0
        kp, kd = gains(lam)
        q_des = q_startup(t) if self.mode == "run" else Q_NOM12
        tau = joint_torque(lam, kp, kd, q_des, self.q, self.qd, self.tau_ff)
        self.stats["sat"] += int(np.any(np.abs(tau) >= TAU_MAX))
        for pub, u in zip(self.pubs, tau):
            pub.publish(Float64(data=float(u)))
        self._check_mode()

    # ---- 200 Hz: 추정 → QP → τ_ff ----
    def _imu_cb(self, msg: Imu) -> None:
        self.imu_stamp = self._now()
        if self.q is None or self.t0 is None:
            return
        o = msg.orientation
        R = yaw_aligned(Rotation.from_quat([o.x, o.y, o.z, o.w]).as_matrix())
        self.tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, R[2, 2]))))
        w_B = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        if not self.p["enable_qp"] or self._now() - self.t0 < STANDUP_S:
            self._check_mode()
            return
        f = None
        try:
            est = estimate(self.q, self.qd, R, w_B, (True,) * 4, self.p["foot_r"], self.p["com_offset"])
            if est is not None:
                x_des = com_target(est.n, self.p["height"], self.p["com_shift"])
                a_des = np.array(self.p["kp_pos"]) * (x_des - est.x) - np.array(self.p["kd_pos"]) * est.v
                R_des = est.R_des @ Rotation.from_euler("xy", [self.p["roll_offset"], self.p["pitch_offset"]]).as_matrix()
                e_R = Rotation.from_matrix(R_des @ R.T).as_rotvec()
                alpha_des = np.array(self.p["kp_rot"]) * e_R - np.array(self.p["kd_rot"]) * (R @ w_B)
                I_G = R @ self.I_B @ R.T
                b = np.concatenate([self.p["mass"] * (a_des + G * np.array([0.0, 0.0, 1.0])), I_G @ alpha_des])
                tic = time.perf_counter()
                f = self.qp.solve(est.r, est.n, b, (True,) * 4, self.f_prev)
                self.stats["qp_ms"].append((time.perf_counter() - tic) * 1e3)
                del self.stats["qp_ms"][:-200]
                if f is not None:
                    self.stats["sum_fz"] = float(f[2::3].sum())
                    fm = f.reshape(4, 3)
                    self.stats["sum_fn"] = float((fm @ est.n).sum())
                    self.stats["sum_ft"] = float((fm @ est.t1).sum())
                    f_B = (R.T @ f.reshape(4, 3).T).T
                    self.tau_ff = np.concatenate([-est.J[i].T @ f_B[i] for i in range(4)])
        except Exception as e:
            f = None
            self.get_logger().warning(f"QP 경로 예외: {e}", throttle_duration_sec=1.0)
        if f is None:
            self.stats["fail"] += 1
            self.qp_fail_since = self.qp_fail_since or self._now()
        else:
            self.qp_fail_since = None
            self.f_prev = f
        self._check_mode()

    def _check_mode(self) -> None:
        if self.mode == "hold":
            return
        fail_s = 0.0 if self.qp_fail_since is None else self._now() - self.qp_fail_since
        if supervise(fail_s, self.tilt_deg, self._age(self.imu_stamp), self._age(self.joint_stamp)) == "hold":
            self.mode = "hold"
            self.tau_ff = np.zeros(12)
            self.get_logger().warning(
                f"hold: qp_fail_s={fail_s:.2f} tilt={self.tilt_deg:.1f} imu_age={self._age(self.imu_stamp):.2f} "
                f"joint_age={self._age(self.joint_stamp):.2f}"
            )

    def _status(self) -> None:
        ms = self.stats["qp_ms"][-200:]
        self.get_logger().info(
            f"status mode={self.mode} qp_ms={np.mean(ms) if ms else 0:.2f}/{np.percentile(ms, 99) if ms else 0:.2f} "
            f"fail={self.stats['fail']} sat={self.stats['sat']} sum_fz={self.stats['sum_fz']:.1f} "
            f"sum_fn={self.stats.get('sum_fn', 0):.1f} sum_ft={self.stats.get('sum_ft', 0):.1f} "
            f"calf_min={min(self.q[2::3]) if self.q is not None else 0:.3f} tilt={self.tilt_deg:.2f}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BalanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
