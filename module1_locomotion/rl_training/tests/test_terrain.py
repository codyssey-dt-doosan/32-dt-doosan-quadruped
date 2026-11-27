"""terrain.py: 타일 배치·origin, 중앙 플랫폼 평평, d = 1에서 경사·계단 상한, 파이프 능선이 꼭짓점에 남는다. Genesis 초기화 없음."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cfgs  # noqa: E402
from terrain import difficulty, make_heightfield, make_tile  # noqa: E402

CFG = cfgs.get_terrain_cfg()
HS = CFG["hs"]
N = round(CFG["tile"] / HS)
C, PC = N // 2, round(CFG["platform"] / 2 / HS)


def tile(kind, d):
    return make_tile(kind, d, CFG, np.random.default_rng(0))


def centers(origins):
    return np.rint(origins[..., :2] / HS).astype(int).reshape(-1, 2)


def test_shapes_and_origins():
    hf, origins = make_heightfield(CFG)
    b = round(CFG["border"] / HS)
    assert hf.shape == (CFG["rows"] * N + 2 * b + 1, len(CFG["cols"]) * N + 2 * b + 1)
    assert origins.shape == (10, 7, 3)
    assert origins[3, 5, :2] == pytest.approx([CFG["border"] + 3.5 * CFG["tile"], CFG["border"] + 5.5 * CFG["tile"]])
    i, j = centers(origins).T
    assert np.array_equal(hf[i, j], origins[..., 2].ravel())  # z = 중앙 플랫폼 높이
    assert not (hf[:b].any() or hf[-b:].any() or hf[:, :b].any() or hf[:, -b:].any())  # 둘레 border 0
    assert tile("flat", 0.0).shape == (N + 1, N + 1)
    assert (difficulty(0, 10), difficulty(9, 10)) == (0.0, 1.0)


def test_platform_flat_and_difficulty_zero_flat():
    hf, origins = make_heightfield(CFG)
    for i, j in centers(origins):
        assert np.ptp(hf[i - PC:i + PC + 1, j - PC:j + PC + 1]) == 0  # std는 같은 값이어도 합산 오차 1e-17
    for kind in CFG["cols"]:
        assert bool(tile(kind, 0.0).any()) == (kind == "rough")


def test_slope_and_stairs_reach_max():
    for kind in ("slope_up", "slope_down"):
        assert np.abs(np.diff(tile(kind, 1.0), axis=0)).max() / HS == pytest.approx(math.tan(math.radians(25.0)))
    assert np.abs(np.diff(tile("slope_up", 0.5), axis=0)).max() / HS == pytest.approx(math.tan(math.radians(12.5)))
    for kind in ("stairs_up", "stairs_down"):
        z = tile(kind, 1.0)
        assert np.abs(np.diff(z, axis=0)).max() == pytest.approx(0.12)
        assert (np.diff(np.flatnonzero(np.diff(z[:C + 1, C]))) == round(CFG["tread"] / HS)).all()  # 디딤 0.30
    for kind in ("slope_up", "stairs_up"):  # 가운데가 낮은 구덩이(나가려면 오른다)
        z = tile(kind, 1.0)
        assert z[C, C] < z[0, 0]
    for kind in ("slope_down", "stairs_down"):
        z = tile(kind, 1.0)
        assert z[C, C] > z[0, 0]


def test_pipes_peak_on_vertices():
    z = tile("pipes", 1.0)
    assert z.max() == pytest.approx(0.10)
    assert (z == z[:, :1]).all()  # 축 y 능선
    assert tile("pipes", 0.4).max() == pytest.approx(0.04)  # 지름 < hs여도 능선이 사라지지 않는다
