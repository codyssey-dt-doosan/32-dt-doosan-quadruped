"""통과 가능 판정(⑤-2) 고정 — 합성 라이다 장면. 실측 기록 검증은 vault DT-작업노트-2026-11-27-C안-경사-인지-구현."""
import math

import numpy as np
import pytest

from elevation_map.traversability import imu_roll_pitch, trav_grid

T12 = math.tan(math.radians(11.8))  # unity_plant 경사로
ARGS = dict(res=0.1, size=4.0, sensor_height=0.56, min_range=0.5, max_step=0.15, max_slope_deg=13.0, step_tol=0.03)


def _quat(roll, pitch, yaw=0.0):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def _scan(f, h=0.56, pitch=0.0, roll=0.0):
    """go2 라이다(16×360, 수직 −0.5~+0.26 rad)를 높이장 f(x, y)에 쏜 센서 프레임 포인트. 라이다 바로 아래 지면이 원점."""
    el, az = np.meshgrid(np.linspace(-0.5, 0.26, 16), np.linspace(-math.pi, math.pi, 360), indexing="ij")
    d = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], -1).reshape(-1, 3)
    cp, sp, cr, sr = math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
    R = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])  # 몸체 → 수평
    dl = d @ R.T
    o = np.array([0.0, 0.0, h + f(np.zeros(1), np.zeros(1))[0]])
    hit = np.full(len(d), np.nan)
    for t in np.arange(0.1, 6.0, 0.01):
        p = o + dl * t
        hit[np.isnan(hit) & (p[:, 2] <= f(p[:, 0], p[:, 1]))] = t
    ok = ~np.isnan(hit)
    return ((dl[ok] * hit[ok, None]) @ R).astype(np.float32)


def _box(x0, x1, y0, y1, top):
    return lambda x, y: np.where((x >= x0) & (x <= x1) & (y >= y0) & (y <= y1), top, 0.0)


def _add(*fs):
    return lambda x, y: sum(g(x, y) for g in fs)


def _ramp(x0, slope=T12, top=0.88):
    return lambda x, y: np.clip((x - x0) * slope, 0, top)


def _down(x0, slope=T12):
    return lambda x, y: -np.clip((x - x0) * slope, 0, 0.88)


_cc = (np.arange(40) + 0.5) * 0.1 - 2.0
CX, CY = np.meshgrid(_cc, _cc)  # [행 = y, 열 = x]


def _straight_free(g):
    """mpc_controller heading_scan 직진 후보와 같은 직사각형(전방 0~1.5 m, 좌우 ±0.35 m)에 '> 0.15' 셀이 없나."""
    return not (g[(CX >= 0) & (CX <= 1.5) & (np.abs(CY) <= 0.35)] > 0.15).any()


def _blocked(g, x0, x1, y0, y1):
    return bool((g[(CX >= x0) & (CX <= x1) & (CY >= y0) & (CY <= y1)] == 1.0).any())


FLAT = lambda x, y: np.zeros_like(x)  # noqa: E731
FREE = [
    ("평지", FLAT, 0.56, 0.0),
    ("경사로 들머리 0.4 m 앞", _ramp(0.4), 0.56, 0.0),
    ("경사로 위(몸체가 경사와 나란히)", _ramp(-3.0), 0.56, -math.atan(T12)),
    ("내리막 들머리 0.5 m 앞(앞 1.5 m 공백)", _down(0.5), 0.56, 0.0),
    ("내리막 위", _down(-3.0), 0.56, math.atan(T12)),
    ("내리막 끝 전이(앞 숙임 9.5°)", lambda x, y: np.where(x < 0, -x * T12, 0.0), 0.50, math.radians(9.5)),
    ("오르막 끝 전이(앞 들림)", lambda x, y: np.where(x < 0.3, (x - 0.3) * T12, 0.0), 0.56, -math.atan(T12)),
    ("legged 높이 0.60·pitch 3°", FLAT, 0.60, math.radians(3)),
]
BLOCK = [
    ("박스 0.36 m 0.8 m 앞", _box(0.8, 1.2, -0.3, 0.3, 0.36), 0.56, 0.0),
    ("박스 0.65 m 1.0 m 앞", _box(1.0, 1.4, -0.3, 0.3, 0.65), 0.56, 0.0),
    ("턱 0.20 m 1.2 m 앞", lambda x, y: np.where(x >= 1.2, 0.20, 0.0), 0.56, 0.0),
    ("낭떠러지 0.88 m 1.0 m 앞", lambda x, y: np.where(x >= 1.0, -0.88, 0.0), 0.56, 0.0),
    ("낙차 0.18 m 1.0 m 앞", lambda x, y: np.where(x >= 1.0, -0.18, 0.0), 0.56, 0.0),
    ("하강 계단 0.8 m 앞", lambda x, y: np.where(x >= 0.8, -0.18 * (1 + np.floor((x - 0.8) / 0.3)), 0.0), 0.56, 0.0),
    ("25° 경사", _ramp(0.4, math.tan(math.radians(25))), 0.56, 0.0),
    ("경사로 위 박스 0.36 m", _add(_ramp(-3.0), _box(1.2, 1.6, -0.3, 0.3, 0.36)), 0.56, -math.atan(T12)),
]


@pytest.mark.parametrize("name,f,h,pitch", FREE, ids=[c[0] for c in FREE])
def test_slopes_and_transitions_stay_free(name, f, h, pitch):
    assert _straight_free(trav_grid(_scan(f, h, pitch), 0.0, pitch, **ARGS))


@pytest.mark.parametrize("name,f,h,pitch", BLOCK, ids=[c[0] for c in BLOCK])
def test_steps_drops_and_steep_slopes_block(name, f, h, pitch):
    assert not _straight_free(trav_grid(_scan(f, h, pitch), 0.0, pitch, **ARGS))


def test_side_stairs_edge_blocked_but_path_free():
    stairs = lambda x, y: np.where(y <= -0.8, -0.18 * (1 + np.floor((-0.8 - y) / 0.3)), 0.0)  # noqa: E731
    g = trav_grid(_scan(_add(_down(0.5), stairs)), 0.0, 0.0, **ARGS)  # 내리막 들머리 + 오른쪽 하강 계단
    assert _straight_free(g) and _blocked(g, 0.0, 1.5, -1.1, -0.6)


def test_narrow_gap_between_obstacles_stays_free():
    f = _add(_box(0.8, 1.6, 0.45, 1.0, 0.65), _box(0.8, 1.6, -1.0, -0.45, 0.65))  # corridor 0.9 m 통로
    g = trav_grid(_scan(f), 0.0, 0.0, **ARGS)
    assert _straight_free(g) and _blocked(g, 0.8, 1.6, 0.45, 1.0) and _blocked(g, 0.8, 1.6, -1.0, -0.45)


def test_gravity_leveling_uses_pitch_and_roll():
    # 경사로 위(몸체가 경사와 나란히)에서 0.9 m 앞부터 25°로 가팔라짐 — 몸체 기준으로는 13° 차이라 회전 없이 보면 통과
    steep = lambda x, y: np.where(x < 0.9, x * T12, 0.9 * T12 + (x - 0.9) * math.tan(math.radians(25)))  # noqa: E731
    assert not _straight_free(trav_grid(_scan(steep, pitch=-math.atan(T12)), 0.0, -math.atan(T12), **ARGS))
    # 옆으로 12° 기운 경사면 위(몸체도 같이 기움)에서 왼쪽 0.5 m부터 25° — roll을 무시하면 놓침
    side = lambda x, y: np.where(y < 0.5, y * T12, 0.5 * T12 + (y - 0.5) * math.tan(math.radians(25)))  # noqa: E731
    roll = math.atan(T12)  # ROS roll + = 오른쪽 아래(왼쪽이 올라감)
    assert _blocked(trav_grid(_scan(side, roll=roll), roll, 0.0, **ARGS), 0.0, 1.5, 0.6, 1.5)


def test_first_return_rule_blocks_drop_inside_blind_zone():
    # 0.7 m 앞 0.18 낙차: 가장 낮은 빔(평지 1.03 m)보다 가까워 위층 지면 점이 없다 — 첫 점 ↔ 발밑 비교로만 막힌다
    assert not _straight_free(trav_grid(_scan(lambda x, y: np.where(x >= 0.7, -0.18, 0.0)), 0.0, 0.0, **ARGS))


@pytest.mark.parametrize("h", [0.72, 0.40])
def test_body_height_offset_is_absorbed_on_flat(h):
    # 라이다 실제 높이가 파라미터 0.56과 달라도(뜬 몸체·주저앉음) 둘레 지면이 고르게 어긋나면 발밑 기준을 옮겨 평지는 통과
    assert _straight_free(trav_grid(_scan(FLAT, h=h), 0.0, 0.0, **ARGS))


def test_non_finite_points_are_ignored():
    xyz = _scan(_box(1.0, 1.4, -0.3, 0.3, 0.65))
    dirty = np.vstack([xyz, [[np.nan, 0, 0], [1.0, np.inf, 0.0], [1.0, 0.0, -np.inf]]]).astype(np.float32)
    assert np.array_equal(trav_grid(dirty, 0.0, 0.0, **ARGS), trav_grid(xyz, 0.0, 0.0, **ARGS), equal_nan=True)


def test_output_is_block_free_or_unseen():
    g = trav_grid(_scan(_box(1.0, 1.4, -0.3, 0.3, 0.65)), 0.0, 0.0, **ARGS)
    assert g.shape == (40, 40) and g.dtype == np.float32
    assert set(np.unique(g[~np.isnan(g)]).tolist()) <= {0.0, 1.0}
    assert np.isnan(trav_grid(np.empty((0, 3), np.float32), 0.0, 0.0, **ARGS)).all()


def test_imu_roll_pitch_ignores_yaw_and_scale():
    r, p = imu_roll_pitch(*_quat(0.05, -math.radians(12), yaw=2.0))
    assert np.isclose(r, 0.05) and np.isclose(p, -math.radians(12))
    x, y, z, w = _quat(0.0, 0.2)
    r, p = imu_roll_pitch(2 * x, 2 * y, 2 * z, 2 * w)  # 정규화 안 된 입력
    assert np.isclose(r, 0.0) and np.isclose(p, 0.2)


def test_imu_roll_pitch_rejects_bad_quaternion():
    assert imu_roll_pitch(float("nan"), 0.0, 0.0, 1.0) is None
    assert imu_roll_pitch(0.0, 0.0, 0.0, 0.0) is None
    assert imu_roll_pitch(float("inf"), 0.0, 0.0, 1.0) is None
