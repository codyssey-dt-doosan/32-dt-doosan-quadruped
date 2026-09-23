"""SRB 지평 MPC(C안 ④). Cheetah-3 볼록 MPC: G 프레임, 상태 13 = [rpy, p, ω, ṗ, g], 입력 발 4×3 × N 스텝.

상태를 소거한 밀집 QP(변수 12N)를 osqp로 푼다. 원뿔 제약은 force_qp.cone_matrix를 스텝마다 쌓고, 비접촉 발은
법선 행 [0, 0]. 희소 패턴은 setup 때 고정(P 상삼각 전부·발 블록 전부), 값만 update. ω×Iω 무시, 소각 오일러,
I_G는 지평 안 고정 — ponytail: 경사 20° 기준 못 맞추면 그때 올린다.
"""
import math
import time

import numpy as np

from mpc_controller.force_qp import ForceQP, _csc_with_pattern, _Solver, cone_matrix, skew

N_X, N_U, N_C = 13, 12, 20
G = 9.8
Q_DEFAULT = np.array([50.0, 50.0, 10.0, 1.0, 1.0, 50.0, 1.0, 1.0, 1.0, 5.0, 5.0, 1.0, 0.0])


def rotz(psi: float) -> np.ndarray:
    c, s = math.cos(psi), math.sin(psi)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def continuous_A() -> np.ndarray:
    A = np.zeros((N_X, N_X))
    A[0:3, 6:9] = np.eye(3)  # θ̇ ≈ ω (yaw 0 선형화)
    A[3:6, 9:12] = np.eye(3)  # ṗ = v
    A[11, 12] = -1.0  # v̇_z = … − g
    return A


def continuous_B(I_inv: np.ndarray, r: np.ndarray, m: float) -> np.ndarray:
    B = np.zeros((N_X, N_U))
    for i in range(4):
        B[6:9, 3 * i : 3 * i + 3] = I_inv @ skew(r[i])
        B[9:12, 3 * i : 3 * i + 3] = np.eye(3) / m
    return B


def discretize(A: np.ndarray, B: np.ndarray, dt: float) -> tuple:
    """ZOH. A³ = 0이라 expm이 다항식으로 닫힘(테스트에서 scipy expm과 대조)."""
    A2 = A @ A
    Ad = np.eye(N_X) + A * dt + A2 * (dt * dt / 2.0)
    Bd = (np.eye(N_X) * dt + A * (dt * dt / 2.0) + A2 * (dt**3 / 6.0)) @ B
    return Ad, Bd


def condense(Ad: np.ndarray, Bds: list) -> tuple:
    """x_{k+1} = Ad^{k+1} x0 + Σ_{j≤k} Ad^{k−j} Bd_j u_j → X = Aqp x0 + Bqp U."""
    N = len(Bds)
    powers = [np.eye(N_X)]
    for _ in range(N):
        powers.append(Ad @ powers[-1])
    Aqp = np.vstack(powers[1:])
    Bqp = np.zeros((N * N_X, N * N_U))
    for k in range(N):
        for j in range(k + 1):
            Bqp[k * N_X : (k + 1) * N_X, j * N_U : (j + 1) * N_U] = powers[k - j] @ Bds[j]
    return Aqp, Bqp


def state(R: np.ndarray, x: np.ndarray, w_B: np.ndarray, v: np.ndarray) -> np.ndarray:
    from scipy.spatial.transform import Rotation

    rpy = Rotation.from_matrix(R).as_euler("xyz")
    return np.concatenate([rpy, x, R @ np.asarray(w_B, float), v, [G]])


def reference(p0, v_cmd: float, w_cmd: float, R_des: np.ndarray, n, height: float, N: int, dt: float) -> np.ndarray:
    """k = 1..N 참조. 접선 위치는 v를 적분, 법선 위치는 항상 height, rpy는 지형 평행 + yaw w·t."""
    from scipy.spatial.transform import Rotation

    n = np.asarray(n, float)
    rpy0 = Rotation.from_matrix(R_des).as_euler("xyz")
    t_fwd = R_des[:, 0]
    p = np.asarray(p0, float) - (np.asarray(p0, float) @ n) * n
    X = np.zeros((N, N_X))
    for k in range(1, N + 1):
        psi = w_cmd * k * dt
        v = v_cmd * (rotz(psi) @ t_fwd)
        p = p + v * dt
        X[k - 1] = np.concatenate([[rpy0[0], rpy0[1], psi], p + height * n, [0.0, 0.0, w_cmd], v, [G]])
    return X


class SrbMpc:
    def __init__(self, mass=13.2, inertia=(0.22, 0.48, 0.58), horizon=10, dt=0.03, q_weights=Q_DEFAULT, alpha=1e-5, mu=0.5, f_min=2.0, f_max=120.0, eps=1e-4):
        assert f_min <= f_max
        self.m, self.I_B, self.N, self.dt = float(mass), np.diag(np.asarray(inertia, float)), int(horizon), float(dt)
        self.Qbar = np.diag(np.tile(np.asarray(q_weights, float), self.N))
        self.alpha, self.mu, self.f_min, self.f_max, self.eps = float(alpha), float(mu), float(f_min), float(f_max), float(eps)
        self.A = continuous_A()
        self.last_ms = 0.0
        self._solver = None
        nu, nc = self.N * N_U, self.N * N_C
        self._p_rows, self._p_cols = np.triu_indices(nu)
        blk = [(N_C * k + 5 * i + a, N_U * k + 3 * i + b) for k in range(self.N) for i in range(4) for a in range(5) for b in range(3)]
        self._a_rows, self._a_cols = (np.array(v) for v in zip(*blk))
        self._nc = nc

    def _matrices(self, x0, X_ref, r, contact, n, R):
        I_inv = np.linalg.inv(R @ self.I_B @ R.T)
        Ad, Bd0 = discretize(self.A, continuous_B(I_inv, r[0], self.m), self.dt)
        Bds = [Bd0] + [discretize(self.A, continuous_B(I_inv, r[k], self.m), self.dt)[1] for k in range(1, self.N)]
        Aqp, Bqp = condense(Ad, Bds)
        BQ = Bqp.T @ self.Qbar
        P = 2.0 * (BQ @ Bqp + self.alpha * np.eye(self.N * N_U))
        q = 2.0 * BQ @ (Aqp @ x0 - X_ref.reshape(-1))
        cone = cone_matrix(n, self.mu)
        C = np.zeros((self._nc, self.N * N_U))
        l, u = np.full(self._nc, -np.inf), np.zeros(self._nc)
        for k in range(self.N):
            C[N_C * k : N_C * (k + 1), N_U * k : N_U * (k + 1)] = cone
            for i in range(4):
                if contact[k, i]:
                    l[N_C * k + 5 * i], u[N_C * k + 5 * i] = self.f_min, self.f_max
                else:
                    l[N_C * k + 5 * i] = 0.0
        return P, q, C, l, u

    def solve(self, x0, X_ref, r, contact, n, R):
        contact = np.asarray(contact, bool)
        if not contact.any():
            return None
        tic = time.perf_counter()
        P, q, C, l, u = self._matrices(np.asarray(x0, float), np.asarray(X_ref, float), np.asarray(r, float), contact, np.asarray(n, float), np.asarray(R, float))
        if self._solver is None:
            Pc, Cc = _csc_with_pattern(P, self._p_rows, self._p_cols), _csc_with_pattern(C, self._a_rows, self._a_cols)
            self._solver = _Solver(Pc, q, Cc, l, u, eps=self.eps, max_iter=400)
            self._p_order, self._a_order = ForceQP._csc_order(Pc), ForceQP._csc_order(Cc)
        else:
            self._solver.update(P[self._p_order], q, C[self._a_order], l, u)
        f = self._solver.solve()
        self.last_ms = (time.perf_counter() - tic) * 1e3
        return None if f is None else f[:N_U]
