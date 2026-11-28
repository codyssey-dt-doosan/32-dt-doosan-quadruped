"""eval_terrain: 시험값 행(rows 2의 1행) 형상이 §5 값인지, latch가 먼저 일어난 쪽만 남기는지. Genesis 초기화 없음."""
import math
import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from eval_terrain import TEST, latch, terrain_cfg  # noqa: E402
from terrain import make_tile  # noqa: E402

T = terrain_cfg()
RNG = np.random.default_rng(0)


def tile(kind):
    return make_tile(kind, 1.0, T, RNG)  # difficulty(1, 2) = 1.0


def test_test_values():
    z = tile("stairs_down")
    assert np.allclose(np.diff(np.unique(np.round(z, 6))), TEST["step_h_max"])
    z = tile("slope_up")
    c = z.shape[0] // 2
    assert math.isclose(math.degrees(math.atan((z[-1, c] - z[-2, c]) / T["hs"])), TEST["slope_deg_max"], abs_tol=1e-6)
    assert math.isclose(tile("pipes").max(), TEST["pipe_d_max"], rel_tol=1e-6)


def test_latch():
    reached = torch.tensor([False, False, True, False])
    fell = torch.tensor([False, False, False, True])
    prog = torch.tensor([1.0, 1.0, 2.5, 0.3])
    r = torch.tensor([1.5, 0.2, 0.0, 9.0])
    done = torch.tensor([False, True, True, False])
    live = latch(reached, fell, prog, r, done)
    assert fell.tolist() == [False, True, False, True]  # 0: 계속, 1: 이번에 전도, 2: 완주 뒤 리셋은 전도 아님, 3: 이미 전도
    assert torch.allclose(prog, torch.tensor([1.5, 1.0, 2.5, 0.3]))  # 리셋된 스텝·latch된 env는 진행을 바꾸지 않는다
    assert live.tolist() == [True, False, False, False]
