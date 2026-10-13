import math

import numpy as np
import pytest

pytest.importorskip("osqp")

from scipy.spatial.transform import Rotation

from mpc_controller.body_estimate import com_target, estimate, terrain_frame, terrain_normal, yaw_aligned
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


def test_terrain_frame_identity_and_slope():
    assert np.allclose(terrain_frame([0.0, 0.0, 1.0]), I3)
    n = np.array([-math.sin(math.radians(15)), 0.0, math.cos(math.radians(15))])
    x_des = np.array([n[2], 0.0, -n[0]])
    x_des /= np.linalg.norm(x_des)
    expected = np.column_stack([x_des, np.cross(n, x_des), n])
    assert np.allclose(terrain_frame(n), expected)


def test_identity_pose():
    e = estimate(Q12, ZERO12, I3, np.zeros(3), ALL, 0.02, COM)
    assert np.allclose(e.n, [0, 0, 1])
    assert e.x == pytest.approx((0.0, 0.0, -0.04 - (FOOT_Z_NOM - 0.02)), abs=1e-4)  # CoM은 발 접촉면 위 0.2842
    assert np.allclose(e.v, 0.0) and np.allclose(e.R_des, I3)
    assert np.allclose(e.r[0], [0.19, 0.14, FOOT_Z_NOM - 0.02 + 0.04], atol=1e-4)
    assert np.allclose(e._replace(v=np.array([0.3, 0.0, 0.01])).v, [0.3, 0.0, 0.01])  # /odom 속도 교체(NamedTuple._replace) 동작 확인


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
    assert e.v[0] == pytest.approx(0.1 * (-FOOT_Z_NOM - 0.04), abs=1e-4) and abs(e.v[1]) < 1e-9


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


def test_foot_memory_first_update_and_shift():
    from mpc_controller.body_estimate import FootMemory

    m = FootMemory()
    p = np.array([[0.19, 0.14, -0.3], [0.19, -0.14, -0.3], [-0.19, 0.14, -0.3], [-0.19, -0.14, -0.3]])
    assert np.allclose(m.update(p, np.ones(4, bool), np.zeros(3), 0.0, 0.005), p)
    # FR·RL 스윙: 몸체가 +x 0.3 m/s로 0.1 s 이동 → 세계 고정점은 몸체 기준 −0.03
    out = m.update(p + 1.0, np.array([True, False, False, True]), np.array([0.3, 0, 0]), 0.0, 0.1)
    assert np.allclose(out[0], p[0] + 1.0) and np.allclose(out[3], p[3] + 1.0)
    assert np.allclose(out[1], p[1] - [0.03, 0, 0]) and np.allclose(out[2], p[2] - [0.03, 0, 0])


def test_foot_memory_yaw_rotation_keeps_world_point():
    from mpc_controller.body_estimate import FootMemory

    m = FootMemory()
    p = np.array([[1.0, 0.0, 0.0]] * 4)
    m.update(p, np.ones(4, bool), np.zeros(3), 0.0, 0.01)
    out = m.update(p, np.zeros(4, bool), np.zeros(3), math.pi / 2, 1.0)  # 몸체가 +90° 회전 → 점은 −90°
    assert np.allclose(out[0], [0.0, -1.0, 0.0], atol=1e-12)


def test_estimate_with_plane_pts_two_feet_matches_four():
    e4 = estimate(Q12, ZERO12, I3, np.zeros(3), ALL, 0.02, COM)
    two = np.array([True, False, False, True])
    assert estimate(Q12, ZERO12, I3, np.zeros(3), two, 0.02, COM) is None  # 현행: 3발 미만 None
    e2 = estimate(Q12, ZERO12, I3, np.zeros(3), two, 0.02, COM, plane_pts=e4.p)
    assert np.allclose(e2.n, e4.n) and np.allclose(e2.x, e4.x) and np.allclose(e2.R_des, e4.R_des)
    assert np.allclose(e2.p, e4.p) and np.allclose(e2.r, e4.r)


def test_foot_memory_reset():
    from mpc_controller.body_estimate import FootMemory

    m = FootMemory()
    p = np.array([[0.19, 0.14, -0.3], [0.19, -0.14, -0.3], [-0.19, 0.14, -0.3], [-0.19, -0.14, -0.3]])
    m.update(p, np.ones(4, bool), np.zeros(3), 0.0, 0.005)
    m.update(p + 1.0, np.array([True, False, False, True]), np.array([0.3, 0, 0]), 0.0, 0.1)  # 평면이 무너진 상태 흉내
    p2 = np.array([[0.19, 0.14, -0.31], [0.19, -0.14, -0.31], [-0.19, 0.14, -0.31], [-0.19, -0.14, -0.31]])
    m.reset(p2)
    assert np.allclose(m.p, p2)


def test_estimate_velocity_from_single_contact_foot():
    qd = np.zeros(12)
    qd[1] = 0.5  # FL thigh만 회전
    one = np.array([True, False, False, False])
    e4 = estimate(Q12, ZERO12, I3, np.zeros(3), ALL, 0.02, COM)
    e = estimate(Q12, qd, I3, np.zeros(3), one, 0.02, COM, plane_pts=e4.p)
    assert np.allclose(e.v, -jacobian("FL", Q_NOM) @ qd[:3])
    assert estimate(Q12, qd, I3, np.zeros(3), np.zeros(4, bool), 0.02, COM, plane_pts=e4.p) is None


def test_slope_vz_zero_on_flat_and_follows_slope():
    from mpc_controller.body_estimate import slope_vz

    assert slope_vz([0.0, 0.0, 1.0], [0.3, 0.1, 0.0]) == 0.0
    th = math.radians(20)
    n = [-math.sin(th), 0.0, math.cos(th)]  # +x 오르막 법선
    assert slope_vz(n, [0.3, 0.0, 0.0]) == pytest.approx(0.3 * math.tan(th))


def test_transport_with_slope_vz_keeps_world_point_on_20deg():
    """스펙 §5-1: 20° 오르막을 0.3 m/s로 1.3 s(앞발→뒷발 시간) 걸은 뒤 기록점 높이 오차 <1 mm. v_z 0(현행)이면 ≈14 cm."""
    from mpc_controller.body_estimate import slope_vz, transport

    th = math.radians(20)
    n = np.array([-math.sin(th), 0.0, math.cos(th)])
    v_true = np.array([0.3, 0.0, 0.3 * math.tan(th)])  # 몸체가 경사면을 따라 이동
    p0 = np.array([[0.19, 0.14, -0.30]])
    p_new, p_old = p0.copy(), p0.copy()
    for _ in range(260):  # 5 ms × 260 = 1.3 s
        v = np.array([0.3, 0.0, 0.0])
        p_new = transport(p_new, [v[0], v[1], slope_vz(n, v)], 0.0, 0.005)
        p_old = transport(p_old, v, 0.0, 0.005)
    truth = p0 - v_true * 1.3
    assert abs(p_new[0, 2] - truth[0, 2]) < 1e-3
    assert abs(p_old[0, 2] - truth[0, 2]) > 0.1


def test_step_history_nearest_radius_and_expiry():
    from mpc_controller.body_estimate import StepHistory

    h = StepHistory(max_n=32, max_age=2.0)
    h.push([[0.10, 0.10, -0.25], [0.30, 0.10, -0.20]])
    assert np.allclose(h.nearest([0.12, 0.10], 0.06), [0.10, 0.10, -0.25])
    assert np.allclose(h.nearest([0.36, 0.10], 0.06), [0.30, 0.10, -0.20])  # 경계 정확히 r → 채택
    assert h.nearest([0.20, 0.30], 0.06) is None
    h.update([0.0, 0.0, 0.0], 0.0, 1.9)
    assert h.nearest([0.10, 0.10], 0.06) is not None
    h.update([0.0, 0.0, 0.0], 0.0, 0.2)  # 2.1 s → 만료
    assert h.nearest([0.10, 0.10], 0.06) is None and len(h.p) == 0


def test_step_history_caps_and_clears():
    from mpc_controller.body_estimate import StepHistory

    h = StepHistory(max_n=3, max_age=2.0)
    for k in range(5):
        h.push([[k * 0.1, 0.0, 0.0]])
    assert len(h.p) == 3 and np.allclose(h.p[:, 0], [0.2, 0.3, 0.4])  # 오래된 것부터 버림
    h.clear()
    assert len(h.p) == 0 and len(h.age) == 0


def test_step_history_transport_follows_yaw():
    from mpc_controller.body_estimate import StepHistory

    h = StepHistory()
    h.push([[1.0, 0.0, 0.0]])
    h.update([0.3, 0.0, 0.0], 0.0, 1.0)  # 몸체 +x 0.3 → 점은 −0.3
    assert np.allclose(h.p[0], [0.7, 0.0, 0.0])
    h.update([0.0, 0.0, 0.0], math.pi / 2, 1.0)  # 몸체 +90° → 점은 −90°
    assert np.allclose(h.p[0], [0.0, -0.7, 0.0], atol=1e-12)


def test_step_history_empty_is_safe():
    from mpc_controller.body_estimate import StepHistory

    h = StepHistory()
    h.update([0.3, 0.0, 0.1], 0.5, 0.005)
    assert h.p.shape == (0, 3) and h.nearest([0.0, 0.0], 0.06) is None
