"""몸체 상태 추정(C안 ③): IMU 자세 + 다리 FK로 지형 법선·접촉점·CoM 위치/속도·목표 자세. 순수 함수.

프레임: B 몸체, G 중력 정렬(z 위, yaw는 몸체 추종). R_GB는 B→G. 발은 정지(미끄러짐 없음) 가정.
"""
import math
from typing import NamedTuple

import numpy as np

from mpc_controller.force_qp import tangents
from mpc_controller.leg_kinematics import foot_pos, jacobian
from mpc_controller.legged_model import LEGS


class Estimate(NamedTuple):
    n: np.ndarray  # 지형 법선 G
    t1: np.ndarray
    t2: np.ndarray
    p: np.ndarray  # (4,3) 접촉점 G, 몸체 원점 기준
    r: np.ndarray  # (4,3) CoM→접촉점 G
    x: np.ndarray  # CoM 위치 G, 접촉 발 중심 기준
    v: np.ndarray  # CoM 속도 G
    R_des: np.ndarray  # 지형 평행 목표 자세(B→G), x축은 헤딩 0(R_des[1,0]=0)인 접선(t1과 다름)
    c_B: np.ndarray  # (4,3) 구 중심 B
    J: np.ndarray  # (4,3,3)


def yaw_aligned(R_WB) -> np.ndarray:
    psi = math.atan2(R_WB[1, 0], R_WB[0, 0])
    c, s = math.cos(psi), math.sin(psi)
    return np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]]) @ R_WB


def terrain_normal(points_G):
    pts = np.asarray(points_G, float)
    if len(pts) < 3:
        return None
    c = pts - pts.mean(axis=0)
    n = np.cross(c[1] - c[0], c[2] - c[0]) if len(pts) == 3 else np.linalg.svd(c)[2][-1]
    n = n / np.linalg.norm(n)
    return n if n[2] > 0 else -n


def com_target(n, height: float, com_shift: float) -> np.ndarray:
    """발 중심 기준 CoM 목표: 법선 거리 height. com_shift 1 = 중력 방향 발 중심 위, 0 = 법선 투영."""
    n = np.asarray(n, float)
    return (1.0 - com_shift) * height * n + com_shift * np.array([0.0, 0.0, height / n[2]])


def estimate(q12, qd12, R_GB, omega_B, contact, foot_r: float, com_offset_B):
    q12, qd12, R, w_B = (np.asarray(a, float) for a in (q12, qd12, R_GB, omega_B))
    idx = [i for i, on in enumerate(contact) if on]
    if len(idx) < 3:
        return None
    c_B = np.array([foot_pos(leg, q12[3 * i : 3 * i + 3]) for i, leg in enumerate(LEGS)])
    J = np.array([jacobian(leg, q12[3 * i : 3 * i + 3]) for i, leg in enumerate(LEGS)])
    centers_G = c_B @ R.T
    n = terrain_normal(centers_G[idx])
    t1, t2 = tangents(n)
    p = centers_G - foot_r * n
    c_com = R @ np.asarray(com_offset_B, float)
    x = c_com - p[idx].mean(axis=0)
    # 발 정지: 0 = v_body + R(J q̇ + ω_B × c_B) → v_com = v_body + ω_G × c_com
    v_body = -np.mean([R @ (J[i] @ qd12[3 * i : 3 * i + 3] + np.cross(w_B, c_B[i])) for i in idx], axis=0)
    v = v_body + np.cross(R @ w_B, c_com)
    x_des = np.array([n[2], 0.0, -n[0]])  # 접선 중 수평 헤딩이 0인 방향 (R_des[1,0] = 0)
    x_des /= np.linalg.norm(x_des)
    R_des = np.column_stack([x_des, np.cross(n, x_des), n])
    return Estimate(n, t1, t2, p, p - c_com, x, v, R_des, c_B, J)
