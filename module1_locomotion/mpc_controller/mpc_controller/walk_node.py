"""SRB 지평 MPC 보행(C안 ④, 옵트인). /cmd_vel (v,w) → trot 스케줄 + Raibert 스윙 + 지평 QP 접지력 → τ. walk.launch.py 전용.

기동은 balance_node와 동일(0~2 s 관절 PD 램프 → 2~3 s 블렌딩) 후 MPC 주도. cmd 없으면 4발 지지(stand),
있으면 trot(walk). IMU 콜백(200 Hz)이 추정→gait→(4번에 1번) MPC→τ_ff, joint_states 콜백(1 kHz)이
지지=τ_ff−Kd q̇ / 스윙=관절 PD로 발행. 프레임: B 몸체, G 중력 정렬(현재 yaw 제거), 위치는 몸체 원점 기준.
"""
import csv
import math
import threading
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import Contacts
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64

from mpc_controller.balance_node import STANDUP_S, TAU_MAX, blend, gains, gz_unpause, joint_torque, q_startup, supervise
from mpc_controller.body_estimate import FootMemory, com_target, estimate, terrain_frame, yaw_aligned
from mpc_controller.gait import Gait, raibert, swing_foot, swing_s
from mpc_controller.leg_kinematics import HIP, HIP_Y, Q_NOM, R_MAX, SIDE, contact_jacobian, foot_pos, ik_clamped, reach
from mpc_controller.legged_model import JOINTS, LEGS, contact_ros_topic, ros_force_topic
from mpc_controller.srb_mpc import G, Q_DEFAULT, SrbMpc, reference, state

MPC_EVERY = 4  # IMU 200 Hz / 4 = 50 Hz
IK_FAIL_HOLD = 20  # 연속 IK 실패 틱(0.1 s) → hold
Q_NOM12 = np.array(Q_NOM * 4)


def sensor_touching(stamps, now: float, hold: float) -> np.ndarray:
    """발별 마지막 '접촉 있음' 수신 시각 → 지금 닿아 있나. gz contact 센서는 접촉이 있을 때만 발행하므로 신선도로 판정."""
    return np.array([t is not None and now - t <= hold for t in stamps])


def touchdown_delay(leg_phase: float, duty: float, hz: float):
    """센서 착지 순간 → 스케줄 지지 시작 대비 지연 s(+ 늦음, − 이름). 스윙 전반(s < 0.5) 접촉은 긁힘이라 None."""
    if leg_phase < duty:
        return leg_phase / hz
    return None if swing_s(leg_phase, duty) < 0.5 else (leg_phase - 1.0) / hz


def td_summary(td: list) -> str:
    """착지 지연 목록(s) → 'med/p10/p90/n' ms."""
    if not td:
        return "-"
    a = np.sort(np.asarray(td)) * 1000
    return f"{np.median(a):.0f}/{np.percentile(a, 10):.0f}/{np.percentile(a, 90):.0f}/{len(a)}"


def swing_targets(gait: Gait, est, R, v_ref_G, w_cmd, p, foot_r, lift, k_r, step_max, q_prev, contact, r_max: float = R_MAX, history=None, r_anchor: float = 0.06):
    """스윙 발의 관절 목표 (12,)와 착지점 (4,3)·IK 실패 수·클램프 수·마지막 실패 상세(dict, 없으면 None)·기록 기준점 높이 (4,).
    지지 발은 q_prev 유지, 착지점은 메모리 값.

    contact: 운동학 조기 착지가 반영된 접촉 마스크 — 접촉 발은 지지로 취급(q_prev 유지).
    history: StepHistory — 착지 xy r_anchor 안에 밟은 자리가 있으면 그 높이 평면에 투영(⑤-1). anc는 그 점의 평면 o 기준
    높이 (c−o)·n, 기록 미사용·지지 발은 nan.
    """
    q_sw, p_land, fails, clamps, detail = q_prev.copy(), p.copy(), 0, 0, None
    o = p.mean(axis=0)
    anc = np.full(4, np.nan)
    for i, leg in enumerate(LEGS):
        s = swing_s(gait.leg_phase(i), gait.duty) if gait.active else None
        if s is None:
            continue
        hip_G = R @ np.array([HIP[leg][0], HIP[leg][1] + SIDE[leg] * HIP_Y, 0.0])  # 착지 기준 = 중립 발 위치(hip 롤 축이 아님 — 축에 두면 4 cm 안쪽 착지로 무릎이 한계까지 굽음, 실측)
        args = (hip_G, o, est.n, est.v, v_ref_G, w_cmd, gait.t_remaining(i), gait.t_stance, k_r, step_max)
        p_land[i] = raibert(*args)
        c = history.nearest(p_land[i][:2], r_anchor) if history is not None else None
        if c is not None:
            p_land[i] = raibert(*args, anchor=c)
            anc[i] = (c - o) @ est.n
        foot_G = swing_foot(p[i], p_land[i], s, lift, est.n)
        center_B = R.T @ (foot_G + foot_r * est.n)
        q_i, clamped = ik_clamped(leg, center_B, r_max)
        if q_i is None:
            fails += 1
            detail = {
                "leg": leg, "s": s, "p_lift": p[i].round(3).tolist(), "p_land": p_land[i].round(3).tolist(),
                "center_B": center_B.round(3).tolist(), "n": est.n.round(3).tolist(),
            }
        else:
            q_sw[3 * i : 3 * i + 3] = q_i
            clamps += int(clamped)
    return q_sw, p_land, fails, clamps, detail, anc


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
                ("height", 0.30),  # 정지 0.30: 0.32는 다리 도달 93%로 특이점 근처라 정지 중 좌굴·전도(실측)
                ("height_walk", 0.30),  # 보행도 0.30: 0.32는 실제 몸체가 7 mm 더 떠 착지점에서 다리가 다 펴져 착지가 17 ms 늦음(센서 실측) → 공중 발에 힘 → 전도
                ("com_shift", 1.0),
                ("mu", 0.5),
                ("f_min", 2.0),
                ("f_max", 120.0),
                ("horizon", 10),
                ("mpc_dt", 0.03),
                ("q_weights", list(Q_DEFAULT)),
                ("q_weights_walk", [50.0, 50.0, 10.0, 20.0, 20.0, 50.0, 1.0, 1.0, 1.0, 8.0, 8.0, 1.0, 0.0]),  # 보행 v_xy 8: 4면 89%, 8이면 v_gain 1.12와 함께 100%(실측). p_z 50: 20으로 낮추면 착지 시 높이 요동이 커져 복합 명령에서 전도(실측)
                ("alpha", 1e-5),
                ("stride_hz", 3.0),
                ("duty", 0.5),
                ("lift", 0.025),
                ("k_raibert", 0.03),
                ("step_max", 0.08),
                ("kp_swing", 120.0),
                ("kd_swing", 2.0),
                ("kd_stance", 1.0),
                ("v_max", 0.6),
                ("w_max", 1.2),
                ("cmd_timeout", 0.5),
                ("odom_timeout", 0.1),
                ("v_gain", 1.12),  # 전진 보정 손잡이 — MPC 추종이 89%로 일관(실측), gait_node의 v_gain과 같은 역할
                ("touch_h", 0.008),
                ("contact_source", "kinematic"),  # sensor: 조기 착지를 gz 발 contact 센서로 판정(walk.launch 기본). 노드 단독 기본은 센서 없어도 되는 kinematic
                ("contact_hold", 0.01),
                # 스윙 목표 무릎 최소 굽힘 rad: 늦은 발이 '1 cm 누르기'를 도달 한계까지 따라가 펴진 채 착지하면 무릎 토크 없이 하중을 받쳐
                # 몸체가 3.6 cm 높게 고착, MPC Σf_z ≈ mg/2(센서 판정 실측). 정상 높이에선 스윙 목표가 −0.90까지라 0.7은 평상시 무영향
                ("swing_knee_min", 0.7),
                ("dump_dir", "/tmp"),
            ],
        )
        self.p = {d.name: d.value for d in p}
        self.mpc = SrbMpc(self.p["mass"], self.p["inertia"], self.p["horizon"], self.p["mpc_dt"], self.p["q_weights"], self.p["alpha"], self.p["mu"], self.p["f_min"], self.p["f_max"])
        self.gait = Gait(self.p["stride_hz"], self.p["duty"])
        self.memory = FootMemory()
        self.q = self.qd = None
        self.t0 = self.joint_stamp = self.imu_stamp = self.last_imu = self.cmd_stamp = None
        self.dbg = deque(maxlen=400)  # hold 진입 시 덤프할 직전 프레임(≈2 s @ 200 Hz)
        self.odom_v_B = np.zeros(3)
        self.odom_stamp = None
        self.vz_f = 0.0
        self.n_f = None
        self.h_ref = self.p["height"]
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
        self.stats = {"qp_ms": [], "fail": 0, "sat": 0, "sum_fz": 0.0, "ik_fail": 0, "ik_clamp": 0, "v_est_max": 0.0, "plane_reset": 0, "early": 0, "mpc_extra": 0, "scuff": 0, "hjump": 0}
        # 스윙 발 튐 진단: IMU 틱 사이 관절 콜백의 최대 지연(sim)·간격(벽시계)·스윙 토크, 직전 틱 발 높이·목표 높이
        self.jlat_max = self.jgap_max = self.tau_sw_max = 0.0
        self.j_wall = None
        self.jlat_all = []  # status 주기마다 비움
        self.h_prev = self.h_tgt_prev = None
        self.td = []  # 센서 착지 지연 s(계측 전용 — walk.launch contact_sensors:=true)
        self.prev_touch = np.zeros(4, bool)
        self.pubs = [self.create_publisher(Float64, ros_force_topic(j), 10) for j in JOINTS]
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 1)  # 최신만: 깊이 10이면 MPC 풀이 동안 쌓인 1 kHz 메시지를 하나씩 처리해 스윙 PD가 최대 10 ms 낡은 상태로 돌고 진동(hjump 실측)
        self.create_subscription(Imu, "/imu", self._imu_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        if self.p["contact_source"] not in ("kinematic", "sensor"):
            raise ValueError(f"contact_source는 kinematic|sensor: {self.p['contact_source']}")
        # 발 contact 센서(변환본에 있을 때만 발행) — 판정(contact_source:=sensor)과 계측(착지 지연·긁힘) 공용
        self.touch_stamp = [None] * 4
        for i, leg in enumerate(LEGS):
            self.create_subscription(Contacts, contact_ros_topic(leg), lambda m, i=i: self._touch_cb(i, m), 10)
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

    def _touch_cb(self, i: int, msg) -> None:
        if msg.contacts:
            self.touch_stamp[i] = self._now()

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom_v_B = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, 0.0])
        self.odom_stamp = self._now()

    # ---- 1 kHz: 토크 합성 ----
    def _joint_cb(self, msg: JointState) -> None:
        pos, vel = dict(zip(msg.name, msg.position)), dict(zip(msg.name, msg.velocity))
        if any(j not in pos for j in JOINTS):
            return
        self.q = np.array([pos[j] for j in JOINTS])
        self.qd = np.array([vel.get(j, 0.0) for j in JOINTS])
        now = self._now()
        self.joint_stamp = now
        wall = time.monotonic()
        if self.j_wall is not None:
            self.jgap_max = max(self.jgap_max, wall - self.j_wall)
        self.j_wall = wall
        lat = now - (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        self.jlat_max = max(self.jlat_max, lat)
        self.jlat_all.append(lat)
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
                    self.tau_sw_max = max(self.tau_sw_max, float(np.abs(tau[s]).max()))
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
        target = self.p["height_walk"] if self.gait.active else self.p["height"]
        self.h_ref += (target - self.h_ref) * dt / 0.3  # 0.3 s 램프 — 정지/보행 목표 높이가 달라 급변하면 점프
        contact = self.gait.contact()
        trust = self.gait.trusted()
        trust = trust if trust.any() else contact
        # 첫 틱은 plane_pts 없이 접촉점 ≥3개가 필요해 trust로 좁히면 안 됨(memory.p is None일 때만 contact)
        mask = trust if self.memory.p is not None else contact
        f = None
        try:
            est = estimate(self.q, self.qd, R, w_B, mask, self.p["foot_r"], self.p["com_offset"], plane_pts=self.memory.p)
            if est is not None:
                # 속도는 /odom(시뮬 오도메트리) — 다리 오도메트리는 스케줄 접촉≠실제 접촉일 때 1 m/s까지 튀어 발 메모리 평면을 무너뜨린다(실측). 실기면 상태 추정기로 대체.
                if self.odom_stamp is not None and now - self.odom_stamp <= self.p["odom_timeout"]:
                    v_G = R @ self.odom_v_B
                    # v_z: 정지(gait 비활성)는 다리 오도메트리 클립+저역통과(τ 50 ms) — 높이 감쇠 없으면 정지 중 진동이 자라 점프(실측).
                    # 보행 중은 0 — 발 교대 틱 스파이크(≈1 m/s)가 저역통과를 거쳐도 50 ms짜리 '상승'으로 남아 QP가 지지력을 끊음(실측 2/4 전도)
                    self.vz_f += (float(np.clip(est.v[2], -0.5, 0.5)) - self.vz_f) * dt / 0.05
                    v_G[2] = 0.0 if self.gait.active else self.vz_f
                    est = est._replace(v=v_G)
                if est.n[2] < 0.87:  # 법선이 중력에서 30°+ 이탈 → 발 메모리 평면 붕괴(실측). 판정은 저역통과 전 원시 n 기준
                    self.memory.reset(est.p)
                    self.stats["plane_reset"] += 1
                    self.get_logger().warning(f"발 메모리 평면 리셋: n={est.n.round(3).tolist()}", throttle_duration_sec=1.0)
                # 법선 저역통과 τ 0.3 s — 불량 발 점 하나로 평면이 10~20° 꺾이면 rpy 가중 50이 몸체를 실제로 굴려
                # 전도(덤프 실측); 경사 전이는 0.3 s면 충분
                self.n_f = est.n if self.n_f is None else (self.n_f + (est.n - self.n_f) * dt / 0.3)
                self.n_f = self.n_f / np.linalg.norm(self.n_f)
                est = est._replace(n=self.n_f, R_des=terrain_frame(self.n_f))
                # 운동학 접촉 판정: 조기 착지만 본다. 공중 판정은 하지 않는다 — 평면이 같은 발들로 추정돼 몸체가 기울면
                # 실제로 닿은 발이 h>15 mm로 보여 힘이 끊기고 붕괴(실측)
                h = np.zeros(4)
                sensor = self.p["contact_source"] == "sensor"
                touching = sensor_touching(self.touch_stamp, now, self.p["contact_hold"])
                if self.gait.active:  # 계측: 센서 착지 순간의 스케줄 대비 지연·스윙 전반 긁힘
                    for i in np.flatnonzero(touching & ~self.prev_touch):
                        d = touchdown_delay(self.gait.leg_phase(i), self.gait.duty, self.gait.hz)
                        if d is None:
                            self.stats["scuff"] += 1
                        else:
                            self.td.append(d)
                self.prev_touch = touching
                if self.memory.p is not None:
                    o = self.memory.p.mean(axis=0)
                    h = (est.p - o) @ est.n
                    for i in range(4):
                        s = swing_s(self.gait.leg_phase(i), self.gait.duty) if self.gait.active else None
                        if not contact[i] and s is not None and s > 0.5 and (touching[i] if sensor else h[i] < self.p["touch_h"]):
                            contact[i] = True  # 조기 착지: 스윙 PD가 지면을 밀지 않게 즉시 지지로
                            self.stats["early"] += 1
                # 스윙 발 튐 진단: 현재 관절 목표 q_sw의 발 높이(평면 기준)·추종 오차, 한 틱(5 ms) 6 mm 넘는 높이 변화는 이벤트로 기록(정상 스윙 ≈1.8 mm)
                o_d = self.memory.p.mean(axis=0) if self.memory.p is not None else np.zeros(3)
                h_tgt = np.array([(R @ foot_pos(leg, self.q_sw[3 * i : 3 * i + 3]) - self.p["foot_r"] * est.n - o_d) @ est.n for i, leg in enumerate(LEGS)])
                swing = ~contact if self.gait.active else np.zeros(4, bool)
                qerr = float(max((np.abs(self.q_sw[3 * i : 3 * i + 3] - self.q[3 * i : 3 * i + 3]).max() for i in np.flatnonzero(swing)), default=0.0))
                diag = (round(self.jlat_max * 1000, 1), round(self.jgap_max * 1000, 1), round(self.tau_sw_max, 1), round(qerr, 3))
                if self.h_prev is not None:
                    for i in np.flatnonzero(swing & (np.abs(h - self.h_prev) > 0.006)):
                        self.stats["hjump"] += 1
                        self.get_logger().warning(
                            f"hjump {LEGS[i]} s={swing_s(self.gait.leg_phase(i), self.gait.duty) or -1:.2f} dh={(h[i] - self.h_prev[i]) * 1000:+.0f}mm "
                            f"dh_tgt={(h_tgt[i] - self.h_tgt_prev[i]) * 1000:+.0f}mm qerr={qerr:.3f} jlat={diag[0]}ms jgap={diag[1]}ms tau_sw={diag[2]}"
                        )
                self.h_prev, self.h_tgt_prev = h.copy(), h_tgt
                self.jlat_max = self.jgap_max = self.tau_sw_max = 0.0
                # 진단 덤프용 프레임 기록(hold 진입 시에만 파일로 씀) — est·contact·h 확정 직후
                self.dbg.append((
                    now - self.t0, self.gait.phase, int(self.gait.active), *contact.astype(int), *self.gait.contact().astype(int),
                    *np.round(h, 4),
                    *[(-1.0 if (s := swing_s(self.gait.leg_phase(i), self.gait.duty)) is None else round(s, 3)) for i in range(4)],
                    round(self.tilt_deg, 2), round(float(self.stats["sum_fz"]), 1), *np.round(est.v[:2], 3),
                    round(float(np.abs(self.tau_ff).max()), 2), *np.round(est.n, 3),
                    *np.round(h_tgt, 4), *diag,
                ))
                # 발 메모리 입력 클립: 스케줄 접촉 발이 실제론 공중일 때 다리 오도메트리 v_est가 튀어 메모리를 오염시킴(실측)
                v_clip = np.clip(est.v, -(self.p["v_max"] + 0.3), self.p["v_max"] + 0.3)
                wz_clip = float(np.clip((R @ w_B)[2], -(self.p["w_max"] + 0.5), self.p["w_max"] + 0.5))
                self.stats["v_est_max"] = max(self.stats["v_est_max"], float(np.linalg.norm(est.v[:2])))
                p_mem = self.memory.update(est.p, contact, v_clip, wz_clip, dt)
                v_ref = self.p["v_gain"] * self.v_cmd
                v_ref_G = v_ref * est.R_des[:, 0]
                self.q_sw, p_land, fails, clamps, detail, _ = swing_targets(
                    self.gait, est, R, v_ref_G, self.w_cmd, p_mem, self.p["foot_r"], self.p["lift"], self.p["k_raibert"], self.p["step_max"], self.q_sw, contact,
                    r_max=reach(self.p["swing_knee_min"]),
                )
                if fails:
                    self.get_logger().warning(f"IK 실패: {detail}", throttle_duration_sec=0.5)
                self.ik_fail_run = self.ik_fail_run + 1 if fails else 0
                self.stats["ik_fail"] += fails
                self.stats["ik_clamp"] += clamps
                # 접촉이 바뀐 틱은 즉시 풀어 새 지지 발에 τ_ff를 준다 — 주기 대기(≤20 ms) 동안 4발 피드포워드 0 지지 공백이 생겨 간헐 전도(덤프 실측)
                changed = not np.array_equal(contact, self.stance)
                # 이륙한 발의 τ_ff는 즉시 0 (다음 MPC까지 기다리지 않음)
                self.stance = contact
                self.tau_ff = self.tau_ff * np.repeat(contact, 3)
                self.imu_count += 1
                if self.imu_count % MPC_EVERY == 0 or changed:
                    if changed and self.imu_count % MPC_EVERY != 0:
                        self.stats["mpc_extra"] += 1
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
        self.mpc.set_weights(self.p["q_weights_walk"] if self.gait.active else self.p["q_weights"])
        x0 = state(R, est.x, w_B, est.v)
        # 정지: 발 중심 앵커(크리프 방지, 실측 3 mm/s) / 보행: 현재 CoM(순수 속도 추종 — 발 중심 앵커는 o가 몸체 뒤에 있어 22 N 뒤로 당겨 속도 60%에서 평형, 실측)
        p0 = est.x if self.gait.active else com_target(est.n, self.h_ref, self.p["com_shift"])
        v_ref = self.p["v_gain"] * self.v_cmd
        X_ref = reference(p0, v_ref, self.w_cmd, est.R_des, est.n, self.h_ref, N, dt)
        sched = self.gait.schedule(N, dt)
        sched[0] = contact  # k=0는 실제(운동학) 접촉 — 스케줄과 어긋나면 지지 발에 힘이 안 가고(τ_ff는 접촉 발에만) 전환 틱마다 지지 공백(덤프 실측)
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
            path = f"{self.p['dump_dir']}/walk_dump_{int(self._now())}.csv"
            with open(path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(
                    ["t", "phase", "active", "c_FL", "c_FR", "c_RL", "c_RR", "s_FL", "s_FR", "s_RL", "s_RR",
                     "h_FL", "h_FR", "h_RL", "h_RR", "sw_FL", "sw_FR", "sw_RL", "sw_RR", "tilt", "sum_fz",
                     "vx", "vy", "tau_max", "nx", "ny", "nz",
                     "ht_FL", "ht_FR", "ht_RL", "ht_RR", "jlat_ms", "jgap_ms", "tau_sw", "qerr"]
                )
                w.writerows(self.dbg)
            self.get_logger().warning(f"덤프: {path}")

    def _status(self) -> None:
        ms = self.stats["qp_ms"][-200:]
        self.get_logger().info(
            f"status mode={self.mode} qp_ms={np.mean(ms) if ms else 0:.2f}/{np.percentile(ms, 99) if ms else 0:.2f} "
            f"fail={self.stats['fail']} sat={self.stats['sat']} sum_fz={self.stats['sum_fz']:.1f} sum_fn=0.0 sum_ft=0.0 "
            f"calf_min={min(self.q[2::3]) if self.q is not None else 0:.3f} tilt={self.tilt_deg:.2f} "
            f"ik_fail={self.stats['ik_fail']} ik_clamp={self.stats['ik_clamp']} phase={self.gait.phase:.2f} active={int(self.gait.active)} "
            f"v_cmd={self.v_cmd:.2f} w_cmd={self.w_cmd:.2f} v_est_max={self.stats['v_est_max']:.2f} plane_reset={self.stats['plane_reset']} "
            f"early={self.stats['early']} mpc_extra={self.stats['mpc_extra']} "
            f"touch={''.join(str(int(b)) for b in sensor_touching(self.touch_stamp, self._now(), 1.0))} "
            f"td_ms={td_summary(self.td)} scuff={self.stats['scuff']} hjump={self.stats['hjump']} "
            f"jlat_ms={np.median(self.jlat_all or [0]) * 1000:.1f}/{np.percentile(self.jlat_all or [0], 99) * 1000:.1f}"
        )
        self.stats["v_est_max"] = 0.0
        self.jlat_all = []


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
