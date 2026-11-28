"""② 지형 법선 자세 보상 terrain_orientation = 1 − (n·z_b)²: orientation_error 값, ±normal_baseline 중앙차분 법선
(20° 경사각·계단·border·heightfield 밖까지 유한·단위 길이·plane (0, 0, 1)). CPU, dr=False.
ground_height가 env마다 점을 받으므로 격자 env는 격자 점 수만큼 env를 둔다."""
import math
import sys
from pathlib import Path

import pytest
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import genesis as gs  # noqa: E402

import cfgs  # noqa: E402
from go2_box_env import Go2BoxEnv, orientation_error  # noqa: E402
from terrain import difficulty  # noqa: E402

S2S = cfgs.load_sim2sim()
SLOPE = {**cfgs.get_terrain_cfg(), "rows": 2, "cols": ["slope_up"], "slope_deg_max": 20.0}
SLOPE_DEG = SLOPE["slope_deg_max"] * difficulty(1, SLOPE["rows"])  # 레벨 1 = 최고 난이도 = 20°
B, T = SLOPE["border"], SLOPE["tile"]
# 레벨 1 타일 가운데에서 +x 축선(대각선 아님)으로 플랫폼 끝 ~ 타일 끝의 중간: 기울기가 x 방향뿐
mid_slope_xy = torch.tensor([[B + 1.5 * T + (SLOPE["platform"] / 2 + T / 2) / 2, B + T / 2]])
STAIRS = {**cfgs.get_terrain_cfg(), "rows": 2, "cols": ["stairs_up", "stairs_down"]}
HI = [2 * STAIRS["border"] + STAIRS["rows"] * STAIRS["tile"], 2 * STAIRS["border"] + len(STAIRS["cols"]) * STAIRS["tile"]]
# 계단·border·heightfield 가장자리, 밖 1 m까지(밖은 ground_height가 잘라 조회)
grid_xy = torch.cartesian_prod(torch.linspace(-1.0, HI[0] + 1.0, 10), torch.linspace(-1.0, HI[1] + 1.0, 10))


def make_env(num_envs, terrain=None):
    if not gs._initialized:  # 다른 테스트 파일이 먼저 돌았을 수 있다
        gs.init(backend=gs.cpu, logging_level="warning")
    return Go2BoxEnv(num_envs, *cfgs.get_cfgs(S2S, dr=False, terrain=terrain))


@pytest.fixture(scope="module")
def env_slope():
    return make_env(1, SLOPE)


@pytest.fixture(scope="module")
def env_stairs():
    return make_env(len(grid_xy), STAIRS)


@pytest.fixture(scope="module")
def env_plane():
    return make_env(1)


def q_pitch(deg):  # y축 회전 wxyz: z_b = (sin θ, 0, cos θ)
    return torch.tensor([[math.cos(math.radians(deg / 2)), 0., math.sin(math.radians(deg / 2)), 0.]])


def test_orientation_error_values():
    up = torch.tensor([[0., 0., 1.]]); q0 = torch.tensor([[1., 0., 0., 0.]])
    assert orientation_error(up, q0).item() == pytest.approx(0.0, abs=1e-7)
    q10 = torch.tensor([[math.cos(math.radians(5)), 0., math.sin(math.radians(5)), 0.]])  # pitch 10°
    assert orientation_error(up, q10).item() == pytest.approx(math.sin(math.radians(10)) ** 2, rel=1e-4)


def test_terrain_normal_slope(env_slope):
    n = env_slope.terrain_normal(mid_slope_xy)
    assert math.degrees(math.acos(n[0, 2].item())) == pytest.approx(SLOPE_DEG, abs=0.5)
    # slope_up은 가운데가 낮은 구덩이라 +x 쪽 점은 x로 오른다 → 법선은 −x로 기울고 y 성분은 없다
    assert n[0, 0] < 0 and abs(n[0, 1]) < 1e-6
    # 몸체가 법선과 나란하면(pitch −20°) 0, 반대로 숙이면(+20°) 둘 사이 40° → sin²40°. quat 방향(역 쿼터니언)도 가린다
    assert orientation_error(n, q_pitch(-SLOPE_DEG)).item() == pytest.approx(0.0, abs=1e-6)
    assert orientation_error(n, q_pitch(SLOPE_DEG)).item() == pytest.approx(math.sin(math.radians(2 * SLOPE_DEG)) ** 2, rel=1e-4)


def test_terrain_normal_finite_unit(env_stairs):
    n = env_stairs.terrain_normal(grid_xy)
    assert torch.isfinite(n).all() and torch.allclose(n.norm(dim=1), torch.ones(len(n)), atol=1e-5)


def test_plane_normal(env_plane):
    assert torch.equal(env_plane.terrain_normal(torch.zeros(3, 2)), torch.tensor([[0., 0., 1.]] * 3))
