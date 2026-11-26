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
