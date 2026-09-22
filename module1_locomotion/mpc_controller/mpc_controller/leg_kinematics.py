"""다리 기구학(C안 힘 제어). 몸체 프레임 B, 발은 **구 중심**(calf 축에서 L2 아래). ROS 의존 없음.

go2/model.sdf는 joint에 <pose>가 없어 축이 링크 박스 중심 → 길이는 링크 0.20이 아니라 축 간 거리.
"""
import math

import numpy as np

from mpc_controller.legged_model import LEGS  # noqa: F401  (호출자 편의 재수출)

HIP = {"FL": (0.19, 0.10), "FR": (0.19, -0.10), "RL": (-0.19, 0.10), "RR": (-0.19, -0.10)}
SIDE = {"FL": 1.0, "FR": -1.0, "RL": 1.0, "RR": -1.0}
HIP_Y, HIP_Z = 0.04, -0.08  # hip 축 → thigh 축 (y는 SIDE 곱)
L1, L2 = 0.18, 0.10  # thigh 축 → calf 축, calf 축 → 구 중심
LIMITS = ((-1.05, 1.05), (-0.66, 2.97), (-1.57, 1.57))
Q_NOM = (0.0, 0.45, -1.35)
FOOT_Z_NOM = -0.3042


def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _local(leg: str, q1: float, q2: float) -> np.ndarray:
    """hip 롤 전, hip 축 기준 구 중심."""
    s1, c1, s12, c12 = math.sin(q1), math.cos(q1), math.sin(q1 + q2), math.cos(q1 + q2)
    return np.array([-L1 * s1 - L2 * s12, SIDE[leg] * HIP_Y, HIP_Z - L1 * c1 - L2 * c12])


def foot_pos(leg: str, q) -> np.ndarray:
    q0, q1, q2 = q
    return np.array([HIP[leg][0], HIP[leg][1], 0.0]) + _rx(q0) @ _local(leg, q1, q2)


def jacobian(leg: str, q) -> np.ndarray:
    """∂foot_pos/∂q, 열 = (hip, thigh, calf)."""
    q0, q1, q2 = q
    c0, s0 = math.cos(q0), math.sin(q0)
    s1, c1, s12, c12 = math.sin(q1), math.cos(q1), math.sin(q1 + q2), math.cos(q1 + q2)
    rx = _rx(q0)
    drx = np.array([[0.0, 0.0, 0.0], [0.0, -s0, -c0], [0.0, c0, -s0]])
    d0 = drx @ _local(leg, q1, q2)
    d1 = rx @ np.array([-L1 * c1 - L2 * c12, 0.0, L1 * s1 + L2 * s12])
    d2 = rx @ np.array([-L2 * c12, 0.0, L2 * s12])
    return np.column_stack([d0, d1, d2])


def ik(leg: str, p_B) -> tuple | None:
    """B 프레임 구 중심 → (hip, thigh, calf). 도달 밖·관절 한계 밖이면 None."""
    dx, dy, dz = np.asarray(p_B, dtype=float) - (HIP[leg][0], HIP[leg][1], 0.0)
    ly = SIDE[leg] * HIP_Y
    rho2 = dy * dy + dz * dz - ly * ly
    if rho2 <= 0.0:
        return None
    lz = -math.sqrt(rho2)  # 다리는 hip 축 아래
    q0 = math.atan2(dz, dy) - math.atan2(lz, ly)
    x, d = dx, HIP_Z - lz  # thigh 축 기준 앞 +x, 아래 d
    r = math.hypot(x, d)
    if r > L1 + L2 or r < abs(L1 - L2) or r < 1e-9:
        return None
    phi = math.atan2(-x, d)
    q1 = phi + math.acos((L1 * L1 + r * r - L2 * L2) / (2 * L1 * r))
    q2 = (phi - math.acos((L2 * L2 + r * r - L1 * L1) / (2 * L2 * r))) - q1
    q = (q0, q1, q2)
    if any(not lo <= v <= hi for v, (lo, hi) in zip(q, LIMITS)):
        return None
    return q
