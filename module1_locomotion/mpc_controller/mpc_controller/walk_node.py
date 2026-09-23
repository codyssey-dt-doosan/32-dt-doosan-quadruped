"""SRB 지평 MPC 보행(C안 ④, 옵트인). /cmd_vel (v,w) → trot 스케줄 + Raibert 스윙 + 지평 QP 접지력 → τ. walk.launch.py 전용.

기동은 balance_node와 동일(0~2 s 관절 PD 램프 → 2~3 s 블렌딩) 후 MPC 주도. cmd 없으면 4발 지지(stand),
있으면 trot(walk). IMU 콜백(200 Hz)이 추정→gait→(4번에 1번) MPC→τ_ff, joint_states 콜백(1 kHz)이
지지=τ_ff−Kd q̇ / 스윙=관절 PD로 발행. 프레임: B 몸체, G 중력 정렬(현재 yaw 제거), 위치는 몸체 원점 기준.
"""
import math
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64

from mpc_controller.balance_node import STANDUP_S, TAU_MAX, blend, gains, gz_unpause, joint_torque, q_startup, supervise
from mpc_controller.body_estimate import FootMemory, com_target, estimate, yaw_aligned
from mpc_controller.gait import Gait, raibert, swing_foot, swing_s
from mpc_controller.leg_kinematics import HIP, Q_NOM, contact_jacobian, ik
from mpc_controller.legged_model import JOINTS, LEGS, ros_force_topic
from mpc_controller.srb_mpc import G, Q_DEFAULT, SrbMpc, reference, state

MPC_EVERY = 4  # IMU 200 Hz / 4 = 50 Hz
IK_FAIL_HOLD = 20  # 연속 IK 실패 틱(0.1 s) → hold
Q_NOM12 = np.array(Q_NOM * 4)


def swing_targets(gait: Gait, est, R, v_ref_G, w_cmd, p, foot_r, lift, k_r, step_max, q_prev):
    """스윙 발의 관절 목표 (12,)와 착지점 (4,3)·IK 실패 수. 지지 발은 q_prev 유지, 착지점은 메모리 값."""
    q_sw, p_land, fails = q_prev.copy(), p.copy(), 0
    n_B = R.T @ est.n
    for i, leg in enumerate(LEGS):
        s = swing_s(gait.leg_phase(i), gait.duty) if gait.active else None
        if s is None:
            continue
        hip_G = R @ np.array([HIP[leg][0], HIP[leg][1], 0.0])
        p_land[i] = raibert(hip_G, p.mean(axis=0), est.n, est.v, v_ref_G, w_cmd, gait.t_remaining(i), gait.t_stance, k_r, step_max)
        foot_G = swing_foot(p[i], p_land[i], s, lift, est.n)
        q_i = ik(leg, R.T @ (foot_G + foot_r * est.n))
        if q_i is None:
            fails += 1
        else:
            q_sw[3 * i : 3 * i + 3] = q_i
    return q_sw, p_land, fails


class WalkNode(Node):
    def __init__(self) -> None:
        super().__init__("walk")
        p = self.declare_parameters(
            "",
            [
                ("world", "corridor"),
                ("start_delay", 3.0),
                ("foot_r", 0.02),
                ("mass", 13.2),
                ("inertia", [0.22, 0.48, 0.58]),
                ("com_offset", [0.0, 0.0, -0.04]),
                ("height", 0.295),
                ("com_shift", 1.0),
                ("mu", 0.5),
                ("f_min", 2.0),
                ("f_max", 120.0),
                ("horizon", 10),
                ("mpc_dt", 0.03),
                ("q_weights", list(Q_DEFAULT)),
                ("alpha", 1e-5),
                ("stride_hz", 3.0),
                ("duty", 0.5),
                ("lift", 0.06),
                ("k_raibert", 0.03),
                ("step_max", 0.12),
                ("kp_swing", 120.0),
                ("kd_swing", 2.0),
                ("kd_stance", 1.0),
                ("v_max", 0.6),
                ("w_max", 1.2),
                ("cmd_timeout", 0.5),
            ],
        )
        self.p = {d.name: d.value for d in p}
        self.mpc = SrbMpc(self.p["mass"], self.p["inertia"], self.p["horizon"], self.p["mpc_dt"], self.p["q_weights"], self.p["alpha"], self.p["mu"], self.p["f_min"], self.p["f_max"])
        self.gait = Gait(self.p["stride_hz"], self.p["duty"])
        self.memory = FootMemory()
        self.q = self.qd = None
        self.t0 = self.joint_stamp = self.imu_stamp = self.last_imu = self.cmd_stamp = None
        self.tau_ff = np.zeros(12)
        self.q_sw = Q_NOM12.copy()
        self.stance = np.ones(4, bool)
        self.cmd = (0.0, 0.0)
        self.v_cmd = self.w_cmd = 0.0
        self.mpc_fail_since = None
        self.ik_fail_run = 0
        self.tilt_deg = 0.0
        self.mode = "run"
        self.imu_count = 0
        self.stats = {"qp_ms": [], "fail": 0, "sat": 0, "sum_fz": 0.0, "ik_fail": 0}
        self.pubs = [self.create_publisher(Float64, ros_force_topic(j), 10) for j in JOINTS]
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        self.create_subscription(Imu, "/imu", self._imu_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)
        self.create_timer(1.0, self._status)
        threading.Timer(self.p["start_delay"], self._unpause).start()
        self.get_logger().info("walk started (도훈, C안 지평 MPC 보행)")

    def _unpause(self) -> None:
        err = gz_unpause(self.p["world"])
        self.get_logger().error(f"gz 재개 실패: {err}") if err else self.get_logger().info("gz 재개")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _age(self, stamp) -> float:
        return 0.0 if stamp is None else self._now() - stamp

    def _cmd_cb(self, msg: Twist) -> None:
        self.cmd = (msg.linear.x, msg.angular.z)
        self.cmd_stamp = self._now()

    # ---- 1 kHz: 토크 합성 ----
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
        lam = blend(t) if self.mode == "run" else 0.0
        if lam < 1.0 or self.mode != "run":  # 기동·블렌딩·hold: balance와 동일 관절 PD
            kp, kd = gains(lam)
            q_des = q_startup(t) if self.mode == "run" else Q_NOM12
            tau = joint_torque(lam, kp, kd, q_des, self.q, self.qd, self.tau_ff)
        else:
            tau = np.empty(12)
            for i in range(4):
                s = slice(3 * i, 3 * i + 3)
                if self.stance[i]:
                    tau[s] = self.tau_ff[s] - self.p["kd_stance"] * self.qd[s]
                else:
                    tau[s] = self.p["kp_swing"] * (self.q_sw[s] - self.q[s]) - self.p["kd_swing"] * self.qd[s]
            tau = np.clip(tau, -TAU_MAX, TAU_MAX)
        self.stats["sat"] += int(np.any(np.abs(tau) >= TAU_MAX))
        for pub, u in zip(self.pubs, tau):
            pub.publish(Float64(data=float(u)))
        self._check_mode()

    # ---- 200 Hz: 추정 → gait → (50 Hz) MPC → τ_ff ----
    def _imu_cb(self, msg: Imu) -> None:
        now = self._now()
        self.imu_stamp = now
        if self.q is None or self.t0 is None:
            return
        dt = 0.005 if self.last_imu is None else min(0.02, max(1e-4, now - self.last_imu))
        self.last_imu = now
        o = msg.orientation
        R = yaw_aligned(Rotation.from_quat([o.x, o.y, o.z, o.w]).as_matrix())
        self.tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, R[2, 2]))))
        w_B = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        if self.mode != "run" or now - self.t0 < STANDUP_S:
            self._check_mode()
            return
        # 명령 램프(0.3 s) — 블렌딩 끝나기 전엔 0
        fresh = self.cmd_stamp is not None and now - self.cmd_stamp <= self.p["cmd_timeout"]
        want_v, want_w = self.cmd if fresh and blend(now - self.t0) >= 1.0 else (0.0, 0.0)
        want_v = max(-self.p["v_max"], min(self.p["v_max"], want_v))
        want_w = max(-self.p["w_max"], min(self.p["w_max"], want_w))
        self.v_cmd += (want_v - self.v_cmd) * dt / 0.3
        self.w_cmd += (want_w - self.w_cmd) * dt / 0.3
        want_active = abs(want_v) + abs(want_w) > 1e-3 or abs(self.v_cmd) + abs(self.w_cmd) > 0.02
        self.gait.tick(dt, want_active)
        contact = self.gait.contact()
        f = None
        try:
            est = estimate(self.q, self.qd, R, w_B, contact, self.p["foot_r"], self.p["com_offset"], plane_pts=self.memory.p)
            if est is not None:
                p_mem = self.memory.update(est.p, contact, est.v, float((R @ w_B)[2]), dt)
                v_ref_G = self.v_cmd * est.R_des[:, 0]
                self.q_sw, p_land, fails = swing_targets(
                    self.gait, est, R, v_ref_G, self.w_cmd, p_mem, self.p["foot_r"], self.p["lift"], self.p["k_raibert"], self.p["step_max"], self.q_sw
                )
                self.ik_fail_run = self.ik_fail_run + 1 if fails else 0
                self.stats["ik_fail"] += fails
                # 이륙한 발의 τ_ff는 즉시 0 (다음 MPC까지 기다리지 않음)
                self.stance = contact
                self.tau_ff = self.tau_ff * np.repeat(contact, 3)
                self.imu_count += 1
                if self.imu_count % MPC_EVERY == 0:
                    f = self._solve(est, R, w_B, p_mem, p_land, contact)
                else:
                    f = self.f_prev
        except Exception as e:
            self.get_logger().warning(f"MPC 경로 예외: {e}", throttle_duration_sec=1.0)
        if f is None:
            self.stats["fail"] += 1
            self.mpc_fail_since = self.mpc_fail_since or now
        else:
            self.mpc_fail_since = None
            self.f_prev = f
        self._check_mode()

    f_prev = None

    def _solve(self, est, R, w_B, p_mem, p_land, contact):
        N, dt = self.p["horizon"], self.p["mpc_dt"]
        x0 = state(R, est.x, w_B, est.v)
        # 참조 원점은 발 중심(com_shift) — 현재 CoM을 쓰면 크리프를 못 잡음(실측 xy 4.6 cm)
        X_ref = reference(
            com_target(est.n, self.p["height"], self.p["com_shift"]), self.v_cmd, self.w_cmd, est.R_des, est.n, self.p["height"], N, dt
        )
        sched = self.gait.schedule(N, dt)
        c_com = R @ np.asarray(self.p["com_offset"], float)
        feet = np.where(contact[:, None], p_mem, p_land)  # ponytail: 지평 안 발 위치는 현재 접촉점 또는 착지점 하나로 고정
        disp = np.cumsum(X_ref[:, 9:12] * dt, axis=0)  # 실제 CoM에서 참조 속도로 예측한 k스텝 변위 — 참조 '위치' 오차를 레버암에 넣으면 그 오차×Σf_z만큼 가짜 모멘트가 생겨 몸체가 기운다(실측 8 cm → 7.6°)
        r = np.array([feet - c_com - disp[k] for k in range(N)])
        f = self.mpc.solve(x0, X_ref, r, sched, est.n, R)
        self.stats["qp_ms"].append(self.mpc.last_ms)
        del self.stats["qp_ms"][:-200]
        if f is None:
            return None
        self.stats["sum_fz"] = float(f[2::3].sum())
        f_B = (R.T @ f.reshape(4, 3).T).T
        n_B = R.T @ est.n
        tau = np.zeros(12)
        for i, leg in enumerate(LEGS):
            if contact[i]:
                tau[3 * i : 3 * i + 3] = -contact_jacobian(leg, self.q[3 * i : 3 * i + 3], self.p["foot_r"], n_B).T @ f_B[i]
        self.tau_ff = tau
        return f

    def _check_mode(self) -> None:
        if self.mode == "hold":
            return
        fail_s = 0.0 if self.mpc_fail_since is None else self._now() - self.mpc_fail_since
        if supervise(fail_s, self.tilt_deg, self._age(self.imu_stamp), self._age(self.joint_stamp)) == "hold" or self.ik_fail_run > IK_FAIL_HOLD:
            self.mode = "hold"
            self.tau_ff = np.zeros(12)
            self.get_logger().warning(
                f"hold: mpc_fail_s={fail_s:.2f} tilt={self.tilt_deg:.1f} imu_age={self._age(self.imu_stamp):.2f} "
                f"joint_age={self._age(self.joint_stamp):.2f} ik_fail_run={self.ik_fail_run}"
            )

    def _status(self) -> None:
        ms = self.stats["qp_ms"][-200:]
        self.get_logger().info(
            f"status mode={self.mode} qp_ms={np.mean(ms) if ms else 0:.2f}/{np.percentile(ms, 99) if ms else 0:.2f} "
            f"fail={self.stats['fail']} sat={self.stats['sat']} sum_fz={self.stats['sum_fz']:.1f} sum_fn=0.0 sum_ft=0.0 "
            f"calf_min={min(self.q[2::3]) if self.q is not None else 0:.3f} tilt={self.tilt_deg:.2f} "
            f"ik_fail={self.stats['ik_fail']} phase={self.gait.phase:.2f} active={int(self.gait.active)} "
            f"v_cmd={self.v_cmd:.2f} w_cmd={self.w_cmd:.2f}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WalkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
