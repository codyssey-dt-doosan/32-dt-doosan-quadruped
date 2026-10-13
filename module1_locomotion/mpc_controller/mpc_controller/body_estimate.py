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


def terrain_frame(n) -> np.ndarray:
    """지형 법선 n → 지형 평행 목표 자세 R_des(B→G), x축은 헤딩 0(R_des[1,0]=0)인 접선(t1과 다름)."""
    n = np.asarray(n, float)
    x_des = np.array([n[2], 0.0, -n[0]])  # 접선 중 수평 헤딩이 0인 방향 (R_des[1,0] = 0)
    x_des /= np.linalg.norm(x_des)
    return np.column_stack([x_des, np.cross(n, x_des), n])


def com_target(n, height: float, com_shift: float) -> np.ndarray:
    """발 중심 기준 CoM 목표: 법선 거리 height. com_shift 1 = 중력 방향 발 중심 위, 0 = 법선 투영."""
    n = np.asarray(n, float)
    return (1.0 - com_shift) * height * n + com_shift * np.array([0.0, 0.0, height / n[2]])


def transport(p, v_G, wz: float, dt: float) -> np.ndarray:
    """세계 고정점들의 G 좌표(몸체 원점 기준)를 몸체 이동 v_G·yaw 회전 wz만큼 갱신: Rz(−ω_z dt)(p − v dt)."""
    c, s = math.cos(-wz * dt), math.sin(-wz * dt)
    Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return (Rz @ (np.asarray(p, float) - np.asarray(v_G, float) * dt).T).T


def slope_vz(n, v_G) -> float:
    """몸체가 지형 평면(법선 n)을 따라 움직인다고 볼 때의 수직 속도(n·v = 0). 평지면 0."""
    return -(n[0] * v_G[0] + n[1] * v_G[1]) / n[2]


class FootMemory:
    """발마다 마지막 접촉점(G, 몸체 원점 기준). 비접촉 발은 몸체 이동·yaw 회전만큼 되돌려 세계 고정점을 유지한다.

    G가 매 틱 yaw를 제거하므로 세계 고정점의 G 좌표는 Rz(−ω_z dt)(p − v dt)로 바뀐다. 스윙 ≤0.17 s라 적분 오차 mm.
    """

    def __init__(self) -> None:
        self.p = None

    def update(self, p_G, contact, v_G, wz: float, dt: float) -> np.ndarray:
        p_G, contact = np.asarray(p_G, float), np.asarray(contact, bool)
        if self.p is None:
            self.p = p_G.copy()
            return self.p
        moved = transport(self.p, v_G, wz, dt)
        self.p = np.where(contact[:, None], p_G, moved)
        return self.p

    def reset(self, p_G) -> None:
        """평면이 무너졌을 때(법선이 중력에서 크게 이탈) 현재 FK 접촉점으로 기억을 되돌린다."""
        self.p = np.asarray(p_G, float).copy()


def estimate(q12, qd12, R_GB, omega_B, contact, foot_r: float, com_offset_B, plane_pts=None):
    q12, qd12, R, w_B = (np.asarray(a, float) for a in (q12, qd12, R_GB, omega_B))
    idx = [i for i, on in enumerate(contact) if on]
    if len(idx) < (1 if plane_pts is not None else 3):
        return None
    c_B = np.array([foot_pos(leg, q12[3 * i : 3 * i + 3]) for i, leg in enumerate(LEGS)])
    J = np.array([jacobian(leg, q12[3 * i : 3 * i + 3]) for i, leg in enumerate(LEGS)])
    centers_G = c_B @ R.T
    plane = np.asarray(plane_pts, float) if plane_pts is not None else None
    n = terrain_normal(plane if plane is not None else centers_G[idx])
    t1, t2 = tangents(n)
    p = centers_G - foot_r * n
    c_com = R @ np.asarray(com_offset_B, float)
    o = plane.mean(axis=0) if plane is not None else p[idx].mean(axis=0)
    x = c_com - o
    # 발 정지: 0 = v_body + R(J q̇ + ω_B × c_B) → v_com = v_body + ω_G × c_com
    v_body = -np.mean([R @ (J[i] @ qd12[3 * i : 3 * i + 3] + np.cross(w_B, c_B[i])) for i in idx], axis=0)
    v = v_body + np.cross(R @ w_B, c_com)
    R_des = terrain_frame(n)
    return Estimate(n, t1, t2, p, p - c_com, x, v, R_des, c_B, J)


class StepHistory:
    """최근 이륙 발의 접지점(G, 몸체 원점 기준) — 뒷발이 앞발이 밟은 높이로 착지하게(C안 ⑤-1).

    FootMemory와 같은 transport로 매 틱 이동, max_age s 지나면 버림(이동 적분 오차 누적 방지).
    """

    def __init__(self, max_n: int = 32, max_age: float = 2.0) -> None:
        self.max_n, self.max_age = max_n, max_age
        self.clear()

    def clear(self) -> None:
        self.p, self.age = np.zeros((0, 3)), np.zeros(0)

    def push(self, pts) -> None:
        pts = np.asarray(pts, float).reshape(-1, 3)
        self.p = np.vstack([self.p, pts])[-self.max_n :]
        self.age = np.concatenate([self.age, np.zeros(len(pts))])[-self.max_n :]

    def update(self, v_G, wz: float, dt: float) -> None:
        self.p, self.age = transport(self.p, v_G, wz, dt), self.age + dt
        keep = self.age <= self.max_age
        self.p, self.age = self.p[keep], self.age[keep]

    def nearest(self, xy, r: float):
        """xy 거리 r 이내 최근접 기록점(3,) 복사본, 없으면 None."""
        if not len(self.p):
            return None
        d = np.hypot(*(self.p[:, :2] - np.asarray(xy, float)[:2]).T)
        i = int(np.argmin(d))
        return self.p[i].copy() if d[i] <= r + 1e-12 else None
