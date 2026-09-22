import math

import numpy as np
import pytest

pytest.importorskip("osqp")

from scipy.spatial.transform import Rotation

from mpc_controller.body_estimate import com_target, estimate, terrain_normal, yaw_aligned
from mpc_controller.leg_kinematics import FOOT_Z_NOM, Q_NOM, foot_pos, jacobian
from mpc_controller.legged_model import LEGS

Q12 = np.array(Q_NOM * 4)
ZERO12 = np.zeros(12)
I3 = np.eye(3)
ALL = (True,) * 4
COM = np.array([0.0, 0.0, -0.04])


def test_yaw_aligned_strips_yaw_only():
    R = Rotation.from_euler("zyx", [0.7, 0.2, -0.1]).as_matrix()
    G = yaw_aligned(R)
    assert abs(math.atan2(G[1, 0], G[0, 0])) < 1e-9
    assert np.allclose(G[2, :], R[2, :])  # 롤·피치(중력 방향의 몸체 표현)는 유지


def test_terrain_normal_points_up_and_handles_three_points():
    pts = np.array([[0, 0, 0], [1, 0, 0.2], [0, 1, 0], [1, 1, 0.2]], float)
    n = terrain_normal(pts)
    assert n[2] > 0 and np.allclose(n, [-0.2, 0, 1] / np.linalg.norm([-0.2, 0, 1]), atol=1e-9)
    assert np.allclose(terrain_normal(pts[:3]), n, atol=1e-9)
    assert terrain_normal(pts[:2]) is None


def test_identity_pose():
    e = estimate(Q12, ZERO12, I3, np.zeros(3), ALL, 0.02, COM)
    assert np.allclose(e.n, [0, 0, 1])
    assert e.x == pytest.approx((0.0, 0.0, -0.04 - (FOOT_Z_NOM - 0.02)), abs=1e-4)  # CoM은 발 접촉면 위 0.2842
    assert np.allclose(e.v, 0.0) and np.allclose(e.R_des, I3)
    assert np.allclose(e.r[0], [0.19, 0.14, FOOT_Z_NOM - 0.02 + 0.04], atol=1e-4)


@pytest.mark.parametrize("euler", [(0.0, 0.25, 0.0), (0.0, 0.0, 0.2), (0.0, -0.15, 0.1)])
def test_terrain_parallel_body_has_zero_orientation_error(euler):
    """발이 몸체 기준 같은 높이면(지형 평행) n = R ẑ, R_des = R."""
    R = yaw_aligned(Rotation.from_euler("zyx", euler).as_matrix())
    e = estimate(Q12, ZERO12, R, np.zeros(3), ALL, 0.02, COM)
    assert np.allclose(e.n, R[:, 2], atol=1e-9)
    assert np.allclose(e.R_des, R, atol=1e-9)


def test_velocity_from_body_rate():
    """발 고정, 몸체가 코를 숙이는 방향(ω_y > 0)으로 돌면 CoM은 앞(+x)으로 간다."""
    e = estimate(Q12, ZERO12, I3, np.array([0.0, 0.1, 0.0]), ALL, 0.02, COM)
    assert e.v[0] == pytest.approx(0.1 * (0.3042 - 0.04), abs=1e-4) and abs(e.v[1]) < 1e-9


def test_velocity_from_joint_rates():
    """관절이 발을 몸체 기준 아래로 1 cm/s 내리면 몸체는 위로 1 cm/s."""
    qd = np.zeros(12)
    for i, leg in enumerate(LEGS):
        qd[3 * i : 3 * i + 3] = np.linalg.solve(jacobian(leg, Q_NOM), [0.0, 0.0, -0.01])
    e = estimate(Q12, qd, I3, np.zeros(3), ALL, 0.02, COM)
    assert np.allclose(e.v, [0, 0, 0.01], atol=1e-6)


def test_contact_mask_uses_only_contact_feet():
    q = Q12.copy()
    q[3 * 3 + 1] = 1.2  # RR thigh를 크게 접어 발을 든다
    e = estimate(q, ZERO12, I3, np.zeros(3), (True, True, True, False), 0.02, COM)
    assert np.allclose(e.n, [0, 0, 1], atol=1e-9)
    assert e.x[2] == pytest.approx(-0.04 - (FOOT_Z_NOM - 0.02), abs=1e-4)


def test_too_few_contacts_returns_none():
    assert estimate(Q12, ZERO12, I3, np.zeros(3), (True, True, False, False), 0.02, COM) is None


def test_com_target():
    assert np.allclose(com_target([0, 0, 1], 0.28, 1.0), [0, 0, 0.28])
    n = np.array([math.sin(0.2), 0, math.cos(0.2)])
    assert np.allclose(com_target(n, 0.28, 0.0), 0.28 * n)  # 법선 투영
    g = com_target(n, 0.28, 1.0)
    assert g[0] == 0 and g[1] == 0 and g @ n == pytest.approx(0.28)  # 중력 방향 발 중심 위, 법선 거리 유지
