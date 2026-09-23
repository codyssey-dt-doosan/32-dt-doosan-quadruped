import numpy as np
import pytest

pytest.importorskip("osqp")

from scipy.linalg import expm
from scipy.spatial.transform import Rotation

from mpc_controller.force_qp import ForceQP
from mpc_controller.srb_mpc import G, N_U, N_X, Q_DEFAULT, SrbMpc, condense, continuous_A, continuous_B, discretize, reference, state

M, I_B = 13.2, np.diag([0.22, 0.48, 0.58])
R_FEET = np.array([[0.19, 0.14, -0.28], [0.19, -0.14, -0.28], [-0.19, 0.14, -0.28], [-0.19, -0.14, -0.28]])
Z = np.array([0.0, 0.0, 1.0])
I3 = np.eye(3)


def stand_ref(N, h=0.28):
    return reference(np.array([0.0, 0.0, h]), 0.0, 0.0, I3, Z, h, N, 0.03)


def test_discretize_matches_expm_and_closed_form():
    A, B = continuous_A(), continuous_B(np.linalg.inv(I_B), R_FEET, M)
    Ad, Bd = discretize(A, B, 0.03)
    Mx = np.zeros((N_X + N_U, N_X + N_U))
    Mx[:N_X, :N_X], Mx[:N_X, N_X:] = A, B
    E = expm(Mx * 0.03)
    assert np.allclose(Ad, E[:N_X, :N_X], atol=1e-12) and np.allclose(Bd, E[:N_X, N_X:], atol=1e-12)
    assert np.allclose(A @ A @ A, 0.0)  # 멱영 → 폐형식이 정확


def test_condense_reproduces_rollout():
    A, B = continuous_A(), continuous_B(np.linalg.inv(I_B), R_FEET, M)
    Ad, Bd = discretize(A, B, 0.03)
    N = 4
    Aqp, Bqp = condense(Ad, [Bd] * N)
    rng = np.random.default_rng(0)
    x0, U = rng.normal(size=N_X), rng.normal(size=N * N_U)
    x, xs = x0.copy(), []
    for k in range(N):
        x = Ad @ x + Bd @ U[k * N_U:(k + 1) * N_U]
        xs.append(x)
    assert np.allclose(Aqp @ x0 + Bqp @ U, np.concatenate(xs), atol=1e-10)


def test_reference_stand_and_walk():
    X = stand_ref(3)
    assert X.shape == (3, N_X) and np.allclose(X[:, :3], 0.0) and np.allclose(X[:, 3:6], [0, 0, 0.28]) and np.all(X[:, 12] == G)
    X = reference(np.array([0.0, 0.0, 0.28]), 0.3, 0.5, I3, Z, 0.28, 10, 0.03)
    assert X[0, 2] == pytest.approx(0.5 * 0.03) and X[9, 2] == pytest.approx(0.5 * 0.3)
    assert X[0, 9] == pytest.approx(0.3 * np.cos(0.015), abs=1e-6) and X[0, 8] == 0.5
    assert X[9, 3] > X[0, 3] and np.allclose(X[:, 5], 0.28)  # 전진, 높이 유지


def test_reference_on_slope_keeps_normal_height():
    n = np.array([-np.sin(0.26), 0.0, np.cos(0.26)])
    x_des = np.array([n[2], 0.0, -n[0]])
    R_des = np.column_stack([x_des, np.cross(n, x_des), n])
    X = reference(0.28 * n, 0.3, 0.0, R_des, n, 0.28, 5, 0.03)
    assert np.allclose(X[:, 3:6] @ n, 0.28) and np.allclose(X[:, 9:12] @ n, 0.0, atol=1e-12)
    assert X[0, 1] == pytest.approx(Rotation.from_matrix(R_des).as_euler("xyz")[1])


def test_state_layout():
    R = Rotation.from_euler("xyz", [0.1, -0.2, 0.0]).as_matrix()
    x = state(R, np.array([0, 0, 0.28]), np.array([0, 0.1, 0]), np.array([0.3, 0, 0]))
    assert np.allclose(x[:3], [0.1, -0.2, 0.0]) and np.allclose(x[6:9], R @ [0, 0.1, 0]) and x[9] == 0.3 and x[12] == G


def test_stand_matches_static_and_one_step_qp():
    mpc = SrbMpc(M, np.diag(I_B), 10, 0.03, Q_DEFAULT, 1e-5, 0.5, 2.0, 120.0)
    X = stand_ref(10)
    x0 = X[0].copy()
    r = np.repeat(R_FEET[None], 10, axis=0)
    f = mpc.solve(x0, X, r, np.ones((10, 4), bool), Z, I3)
    assert f is not None and f.shape == (12,)
    assert np.allclose(f[2::3], M * G / 4, atol=0.2) and np.allclose(f[0::3], 0.0, atol=0.2) and np.allclose(f[1::3], 0.0, atol=0.2)
    one = ForceQP(0.5, 2.0, 120.0).solve(R_FEET, Z, np.array([0, 0, M * G, 0, 0, 0.0]), (True,) * 4, np.zeros(12))
    # MPC는 α 1e-5가 dt² 스케일 위치 감도 대비 상대적으로 강해 발당 ≈0.12 N 낮춤, ForceQP는 α+β 0.011로 ≈0.05 N — 둘 다 mg/4의 0.5% 미만
    assert np.allclose(f, one, atol=0.2)
    assert mpc.last_ms > 0


def test_swing_feet_get_zero_force_and_forward_ref_pushes():
    mpc = SrbMpc(M, np.diag(I_B), 10, 0.03, Q_DEFAULT, 1e-5, 0.5, 2.0, 120.0)
    X = reference(np.array([0, 0, 0.28]), 0.3, 0.0, I3, Z, 0.28, 10, 0.03)
    x0 = np.array([0, 0, 0, 0, 0, 0.28, 0, 0, 0, 0, 0, 0, G], float)  # 정지 → 참조는 전진
    contact = np.ones((10, 4), bool)
    contact[:, 1] = contact[:, 2] = False  # FR·RL 스윙
    f = mpc.solve(x0, X, np.repeat(R_FEET[None], 10, axis=0), contact, Z, I3)
    # atol 1e-6은 osqp eps=1e-4(기본, 속도 우선 설계)로 도달 불가 — 실측 잔차 ≈2.2e-3, 5e-3으로 여유(브리프 원문 결함)
    assert np.allclose(f[3:9], 0.0, atol=5e-3)
    assert f[0] + f[9] > 1.0  # Σf_x > 0
    assert f[2] + f[11] == pytest.approx(M * G, rel=0.15)


def test_slope_stand_force_split():
    th = np.radians(15)
    n = np.array([-np.sin(th), 0, np.cos(th)])
    x_des = np.array([n[2], 0, -n[0]])
    R_des = np.column_stack([x_des, np.cross(n, x_des), n])
    mpc = SrbMpc(M, np.diag(I_B), 10, 0.03, Q_DEFAULT, 1e-5, 0.5, 2.0, 120.0)
    X = reference(0.28 * n, 0.0, 0.0, R_des, n, 0.28, 10, 0.03)
    r = np.repeat((R_FEET @ R_des.T)[None], 10, axis=0)
    f = mpc.solve(X[0], X, r, np.ones((10, 4), bool), n, R_des)
    fm = f.reshape(4, 3)
    assert (fm @ n).sum() == pytest.approx(M * G * np.cos(th), rel=0.05)
    assert abs((fm @ x_des).sum()) == pytest.approx(M * G * np.sin(th), rel=0.15)


def test_no_contact_returns_none_and_update_equals_fresh():
    mpc = SrbMpc(M, np.diag(I_B), 4, 0.03, Q_DEFAULT, 1e-5, 0.5, 2.0, 120.0)
    X = stand_ref(4)
    assert mpc.solve(X[0], X, np.repeat(R_FEET[None], 4, axis=0), np.zeros((4, 4), bool), Z, I3) is None
    r = np.repeat(R_FEET[None], 4, axis=0)
    a = mpc.solve(X[0], X, r, np.ones((4, 4), bool), Z, I3)
    th = np.radians(30)  # mu=0.5 → 원뿔 반각 atan(0.5)=26.57°. 10°로는 원뿔이 안 걸려 a와 b가 사실상 같아짐(브리프 원문 결함) → 30°로 실제로 활성화
    n = np.array([-np.sin(th), 0, np.cos(th)])
    b = mpc.solve(X[0], X, r, np.ones((4, 4), bool), n, I3)  # 경사로 update
    fresh = SrbMpc(M, np.diag(I_B), 4, 0.03, Q_DEFAULT, 1e-5, 0.5, 2.0, 120.0).solve(X[0], X, r, np.ones((4, 4), bool), n, I3)
    assert np.allclose(b, fresh, atol=1e-2) and not np.allclose(a, b, atol=1e-2)
