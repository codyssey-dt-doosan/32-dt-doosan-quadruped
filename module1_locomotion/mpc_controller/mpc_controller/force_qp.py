"""접지력 분배 QP(C안 ③). G 프레임, 발 4개 × (x,y,z) = 12 변수. Cheetah 3 balance controller 꼴.

min (Af−b)ᵀS(Af−b) + α‖f‖² + β‖f−f_prev‖²   s.t. 발마다 f_min ≤ nᵀf ≤ f_max, |tᵀf| ≤ μ nᵀf (정사각 피라미드).
osqp 희소 패턴은 setup 때 고정(상삼각 전부·발 블록 전부)하고 값만 갱신한다 — scipy가 0을 버려 경사에서
update가 깨지는 걸 막기 위해.
"""
import numpy as np
import scipy.sparse as sp

try:
    import osqp
except ImportError as e:  # 옵트인 의존성 — package.xml에 없음
    raise RuntimeError("osqp 없음: conda install -n ros_env -c conda-forge osqp") from e

N_F = 12
N_C = 20  # 발마다 5행


def skew(v) -> np.ndarray:
    x, y, z = v
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def wrench_matrix(r) -> np.ndarray:
    """(4,3) CoM→접촉점 → (6,12): 위 3행 힘 합, 아래 3행 Σ r×f."""
    A = np.zeros((6, N_F))
    for i in range(4):
        A[0:3, 3 * i : 3 * i + 3] = np.eye(3)
        A[3:6, 3 * i : 3 * i + 3] = skew(r[i])
    return A


def tangents(n) -> tuple:
    n = np.asarray(n, float)
    t1 = np.array([1.0, 0.0, 0.0]) - n[0] * n
    t1 /= np.linalg.norm(t1)
    return t1, np.cross(n, t1)


def cone_matrix(n, mu: float) -> np.ndarray:
    """(20,12): 발마다 [nᵀ; t1ᵀ−μnᵀ; −t1ᵀ−μnᵀ; t2ᵀ−μnᵀ; −t2ᵀ−μnᵀ]."""
    t1, t2 = tangents(n)
    rows = np.array([n, t1 - mu * n, -t1 - mu * n, t2 - mu * n, -t2 - mu * n])
    C = np.zeros((N_C, N_F))
    for i in range(4):
        C[5 * i : 5 * i + 5, 3 * i : 3 * i + 3] = rows
    return C


def _csc_with_pattern(dense: np.ndarray, rows, cols) -> sp.csc_matrix:
    m = sp.csc_matrix((dense[rows, cols], (rows, cols)), shape=dense.shape)
    m.sort_indices()
    return m


class _Solver:
    """osqp 호출을 한 곳에 — Task 1에서 확인한 API 이름으로 맞춘다."""

    def __init__(self, P: sp.csc_matrix, q, A: sp.csc_matrix, l, u):
        self.prob = osqp.OSQP()
        # 테스트 케이스들은 100~200 iter에서 수렴(eps_abs=eps_rel=1e-9) — max_iter=4000은 20배 여유. 실시간 루프 p99는 Task 7에서 측정.
        self.prob.setup(P, q, A, l, u, verbose=False, warm_starting=True, eps_abs=1e-9, eps_rel=1e-9, max_iter=4000)

    def update(self, P_x, q, A_x, l, u) -> None:
        self.prob.update(q=q, l=l, u=u, Px=P_x, Ax=A_x)

    def solve(self):
        res = self.prob.solve(raise_error=False)  # osqp 1.1.3: raise_error 생략 시 PendingDeprecationWarning
        return np.array(res.x, dtype=float) if str(res.info.status).lower().startswith("solved") else None


class ForceQP:
    def __init__(self, mu=0.5, f_min=2.0, f_max=120.0, weights=(1, 1, 2, 10, 10, 5), alpha=1e-3, beta=1e-2):
        assert f_min <= f_max, "f_min > f_max면 osqp가 update를 조용히 무시한다"
        self.mu, self.f_min, self.f_max = float(mu), float(f_min), float(f_max)
        self.S = np.diag(np.asarray(weights, float))
        self.alpha, self.beta = float(alpha), float(beta)
        self._solver = None
        # 패턴: P 상삼각 전부, A 발 블록(5×3) 전부 — 값이 0이어도 자리를 유지
        self._p_rows, self._p_cols = np.triu_indices(N_F)
        blk = [(5 * i + k, 3 * i + j) for i in range(4) for k in range(5) for j in range(3)]
        self._a_rows, self._a_cols = (np.array(v) for v in zip(*blk))

    def _matrices(self, r, n, b, contact, f_prev):
        A = wrench_matrix(r)
        P = 2.0 * (A.T @ self.S @ A + (self.alpha + self.beta) * np.eye(N_F))
        q = -2.0 * (A.T @ self.S @ b + self.beta * np.asarray(f_prev, float))
        C = cone_matrix(n, self.mu)
        l, u = np.full(N_C, -np.inf), np.zeros(N_C)
        for i, on in enumerate(contact):
            l[5 * i], u[5 * i] = (self.f_min, self.f_max) if on else (0.0, 0.0)
        return P, q, C, l, u

    def solve(self, r_G, n_G, b_G, contact, f_prev):
        if not any(contact):
            return None
        P, q, C, l, u = self._matrices(np.asarray(r_G, float), np.asarray(n_G, float), np.asarray(b_G, float), contact, f_prev)
        if self._solver is None:
            self._solver = _Solver(
                _csc_with_pattern(P, self._p_rows, self._p_cols), q, _csc_with_pattern(C, self._a_rows, self._a_cols), l, u
            )
            self._p_order = self._csc_order(_csc_with_pattern(P, self._p_rows, self._p_cols))
            self._a_order = self._csc_order(_csc_with_pattern(C, self._a_rows, self._a_cols))
        else:
            self._solver.update(P[self._p_order], q, C[self._a_order], l, u)
        return self._solver.solve()

    @staticmethod
    def _csc_order(m: sp.csc_matrix) -> tuple:
        """csc 저장 순서의 (rows, cols) — dense[rows, cols]가 곧 data 배열."""
        cols = np.repeat(np.arange(m.shape[1]), np.diff(m.indptr))
        return m.indices, cols
