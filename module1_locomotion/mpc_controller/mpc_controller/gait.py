"""trot 보행 스케줄·착지·스윙(C안 ④). 순수 함수 + 위상 상태 하나. 접촉은 스케줄로 판정(센서 없음).

ponytail: 조기 접지·헛디딤 감지 없음(Cheetah도 스케줄). 원심항 없음. 20° 경사 기준 못 맞추면 그때.
"""
import math

import numpy as np

from mpc_controller.gait_node import DIAGONAL_PHASE
from mpc_controller.legged_model import LEGS
from mpc_controller.srb_mpc import rotz

OFFSETS = np.array([DIAGONAL_PHASE[leg] for leg in LEGS])


def contact_at(phase: float, duty: float) -> np.ndarray:
    return ((phase + OFFSETS) % 1.0) < duty


def swing_s(leg_phase: float, duty: float):
    """스윙 진행 0~1, 지지 중이면 None."""
    return None if leg_phase < duty else (leg_phase - duty) / (1.0 - duty)


def crossed_half(prev: float, cur: float) -> bool:
    """위상이 0 또는 0.5를 지났나(스윙 쌍 착지 시점)."""
    return int(prev * 2) != int(cur * 2) or cur < prev


class Gait:
    def __init__(self, hz: float, duty: float) -> None:
        self.hz, self.duty = float(hz), float(duty)
        self.phase, self.active = 0.0, False

    @property
    def t_stance(self) -> float:
        return self.duty / self.hz

    def tick(self, dt: float, want_active: bool) -> None:
        if want_active:
            self.active = True
        if not self.active:
            return
        prev = self.phase
        self.phase = (self.phase + self.hz * dt) % 1.0
        if not want_active and crossed_half(prev, self.phase):
            self.active = False
            self.phase = (math.floor(self.phase * 2) / 2.0) % 1.0  # 방금 지난 경계로 스냅 → 4발 지지

    def leg_phase(self, i: int) -> float:
        return (self.phase + OFFSETS[i]) % 1.0

    def contact(self) -> np.ndarray:
        return contact_at(self.phase, self.duty) if self.active else np.ones(4, bool)

    def schedule(self, N: int, dt: float) -> np.ndarray:
        if not self.active:
            return np.ones((N, 4), bool)
        return np.array([contact_at(self.phase + k * dt * self.hz, self.duty) for k in range(N)])

    def t_remaining(self, i: int) -> float:
        s = swing_s(self.leg_phase(i), self.duty)
        return 0.0 if s is None else (1.0 - s) * (1.0 - self.duty) / self.hz


def raibert(hip_G, o_G, n_G, v_est, v_ref, w_ref: float, t_rem: float, t_stance: float, k_r: float, step_max: float) -> np.ndarray:
    """착지점(G, 몸체 원점 기준): 착지 시각 hip 예측 + v_est·T_st/2 + k_r(v_est − v_ref), 평면(o, n) 투영, hip 반경 클램프."""
    hip, o, n = (np.asarray(a, float) for a in (hip_G, o_G, n_G))
    v_est, v_ref = np.asarray(v_est, float), np.asarray(v_ref, float)
    hip_pred = rotz(w_ref * t_rem) @ hip + v_ref * t_rem
    p = hip_pred + v_est * (t_stance / 2.0) + k_r * (v_est - v_ref)
    d = p[:2] - hip_pred[:2]
    dist = float(np.hypot(*d))
    if dist > step_max:
        p[:2] = hip_pred[:2] + d * (step_max / dist)
    return p - ((p - o) @ n) * n


def swing_foot(p_lift, p_land, s: float, lift: float, n) -> np.ndarray:
    p_lift, p_land, n = (np.asarray(a, float) for a in (p_lift, p_land, n))
    return p_lift + (p_land - p_lift) * s + lift * math.sin(math.pi * s) * n
