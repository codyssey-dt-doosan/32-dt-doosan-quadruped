import base64
import importlib.util
from pathlib import Path

import numpy as np

_spec = importlib.util.spec_from_file_location(
    "unity_layout_to_sdf", Path(__file__).resolve().parents[1] / "scripts" / "unity_layout_to_sdf.py")
ul = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(ul)

WALL, VOID = 32767, -32768


def _layout(cells_mm):
    a = np.asarray(cells_mm, dtype="<i2")
    return {"resolution": 0.1, "origin_x": -0.15, "origin_y": -0.15, "nx": a.shape[1], "ny": a.shape[0],
            "wall": WALL, "void": VOID, "data": base64.b64encode(a.tobytes()).decode()}


def test_merge_rects_covers_exactly_the_cells():
    rng = np.random.default_rng(0)
    cls = rng.integers(-1, 3, size=(23, 31))
    for v in (-1, 0, 1, 2):
        cover = np.zeros_like(cls, dtype=int)
        for i, j, w, h in ul.merge_rects(cls, v):
            cover[j:j + h, i:i + w] += 1
        assert (cover == (cls == v)).all()  # 빠짐·겹침 없음


def test_classify_walls_void_flat_and_ramp():
    lay = _layout([[0, 10, 90, WALL], [VOID, 0, 21, 500]])
    h, wall, void = ul.load_grid(lay)
    cls, base = ul.classify(h, wall, void)
    assert base == 0.0
    assert cls.tolist() == [[0, 0, 4, -1], [-1, 0, 0, 25]]  # 2 cm 이하는 바닥판, 21 mm→1 step은 FLAT 이하라 0


def test_world_sdf_spawns_on_origin_floor_and_names_world():
    lay = _layout([[0, 0, 0], [0, 300, 0], [0, 0, 0]])  # 원점(가운데 셀)이 0.3 m 높이
    h, wall, void = ul.load_grid(lay)
    sdf = ul.world_sdf(lay, h, *ul.classify(h, wall, void))
    assert '<world name="unity_plant">' in sdf
    assert "<pose>0 0 0.700 0 0 0</pose>" in sdf
    assert sdf.count('<collision name="step_') == 1
    assert sdf.count("<model ") == 1  # 박스는 모두 정적 모델 plant 하나의 링크 안


def test_negative_floor_lowers_base_instead_of_flattening():
    lay = _layout([[0, -500], [0, 0]])  # 원점보다 0.5 m 낮은 구덩이
    h, wall, void = ul.load_grid(lay)
    cls, base = ul.classify(h, wall, void)
    assert base == -0.5 and cls.tolist() == [[25, 0], [25, 25]]
    assert "<pose>-0.050 -0.050 -0.550 0 0 0</pose>" in ul.world_sdf(lay, h, cls, base)  # 바닥판이 구덩이 높이


def test_fill_pits_bridges_grating_holes_but_not_edges():
    h = np.array([[0.088, 0.0, 0.062, 0.0, 0.0, 0.30]])  # 경사로 들머리 구멍(가운데 0), 계단 모서리(오른쪽 끝)
    out = ul.fill_pits(h, np.ones_like(h, dtype=bool))
    assert np.isclose(out[0, 1], 0.075)            # 양쪽 0.088·0.062 → 평균
    assert out[0, 3] == 0.0 and out[0, 4] == 0.0   # 한쪽(0.30)만 높은 모서리는 그대로


def _ramp_layout(rect=True):
    """바닥 0(x 셀 0~9) → 기울기 0.2 경사로(10~29) → 위층 0.4 m(30~39), 양옆 벽. rect=False면 경사로 한 귀퉁이를 바닥으로."""
    a = np.zeros((12, 40), dtype=int)
    for i in range(10, 30):
        a[2:10, i] = round(((i - 10) + 0.5) * 0.1 * 0.2 * 1000)
    a[2:10, 30:] = 400
    a[[0, 1, 10, 11], :] = WALL
    if not rect:
        a[2:5, 10:14] = 0
    return _layout(a)


def _pipeline(lay):
    h, wall, void = ul.load_grid(lay)
    floor = ~(wall | void)
    ramps = ul.find_ramps(h, floor, lay["resolution"])
    cls, base = ul.classify(h, wall, void, ramps)
    return ramps, ul.world_sdf(lay, h, cls, base, ramps)


def _pose_size(sdf, name):
    import re
    m = re.search(rf'<collision name="{name}"><pose>([^<]+)</pose><geometry><box><size>([^<]+)</size>', sdf)
    return list(map(float, m.group(1).split())), list(map(float, m.group(2).split()))


def test_ramp_becomes_one_tilted_box_instead_of_pillars():
    ramps, sdf = _pipeline(_ramp_layout())
    assert len(ramps) == 1
    assert sdf.count('<collision name="ramp_') == 1
    assert sdf.count('<collision name="step_') == 1  # 위층 0.4 m 한 덩어리만, 경사로 기둥 없음
    (x, y, z, roll, pitch, yaw), (sx, sy, sz) = _pose_size(sdf, "ramp_0")
    assert np.isclose(pitch, -np.arctan(0.2), atol=0.01) and roll == 0 and yaw == 0
    assert np.isclose(sy, 0.8, atol=1e-3)  # 경사로 폭 8셀


def test_ramp_top_surface_meets_low_floor_and_upper_floor():
    _, sdf = _pipeline(_ramp_layout())
    (x, y, z, _, pitch, _), (sx, sy, sz) = _pose_size(sdf, "ramp_0")
    c, s = np.cos(pitch), np.sin(pitch)
    def world(lx, lz):  # Ry(pitch)·(lx, 0, lz) + center
        return x + c * lx + s * lz, z - s * lx + c * lz
    x0, z0 = world(-sx / 2, sz / 2)   # 윗면 낮은 끝
    x1, z1 = world(sx / 2, sz / 2)    # 윗면 높은 끝
    ox = -0.15
    assert np.isclose(x0, ox + 1.0, atol=0.01) and abs(z0) < 0.01         # 바닥(0)에서 시작
    assert np.isclose(x1, ox + 3.0, atol=0.01) and abs(z1 - 0.4) < 0.01   # 위층(0.4)에 닿음
    assert world(-sx / 2, -sz / 2)[1] < 0 and world(sx / 2, -sz / 2)[1] < 0  # 아랫면은 바닥 아래 — 속이 찬 경사로


def test_non_rectangular_slope_stays_pillars():
    ramps, sdf = _pipeline(_ramp_layout(rect=False))
    assert ramps == []
    assert '<collision name="ramp_' not in sdf
    assert sdf.count('<collision name="step_') > 1
