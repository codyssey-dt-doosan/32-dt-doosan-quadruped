"""지형 모드(B2): 타일 origin 스폰, terrain_levels_vel 승강·최고 레벨 순환, 지형 기준 높이, 지형 이탈 종료.
작은 지형(3행 × 3열, 타일 3 m), CPU, 8 env. 열 0(flat)의 env 0~2만 옮겨 다른 env의 지형 높이에 끼지 않게 한다."""
import sys
from pathlib import Path

import pytest
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import genesis as gs  # noqa: E402

import cfgs  # noqa: E402
from go2_box_env import Go2BoxEnv  # noqa: E402
from terrain import make_heightfield  # noqa: E402

S2S = cfgs.load_sim2sim()
TERRAIN = {**cfgs.get_terrain_cfg(), "rows": 3, "cols": ["flat", "slope_up", "stairs_down"], "tile": 3.0}
ROWS, N_ENVS = TERRAIN["rows"], 8
FLAT = [0, 1, 2]  # types = arange(8)·3 // 8 → 열 0(flat)


def init():
    if not gs._initialized:  # 같은 프로세스의 다른 테스트 파일이 먼저 초기화했을 수 있다
        gs.init(backend=gs.cpu, logging_level="warning")


@pytest.fixture(scope="module")
def env():
    init()
    return Go2BoxEnv(N_ENVS, *cfgs.get_cfgs(S2S, dr=False, terrain=TERRAIN))


def zero(env):
    return torch.zeros((env.num_envs, env.num_actions), dtype=gs.tc_float, device=gs.device)


def reset_at(env, level):
    env.levels[:] = level
    env.reset()
    env.commands[:] = 0.0  # cmd_path가 늘지 않게(내려가기 판정을 테스트가 정한다)


def move(env, idx, dx):
    pos = env.robot.get_pos()[idx]
    pos[:, 0] += dx
    env.robot.set_pos(pos, envs_idx=idx)


def test_plane_mode_unchanged():
    init()
    env = Go2BoxEnv(1, *cfgs.get_cfgs(S2S, dr=False))
    assert env.env_cfg["terrain"] is None and env.ground is env.plane
    xy = torch.tensor([[[0.3, -2.0], [10.0, 4.0]]], dtype=gs.tc_float, device=gs.device)
    assert torch.equal(env.ground_height(xy), torch.zeros((1, 2), dtype=gs.tc_float, device=gs.device))


def test_spawn_on_tile_origin(env):
    assert set(env.types.tolist()) == {0, 1, 2} and 0 <= env.levels.min() and env.levels.max() < ROWS
    reset_at(env, ROWS - 1)  # 최고 난이도: 계단 플랫폼은 높고 경사 구덩이는 낮다
    _, origins = make_heightfield(TERRAIN)
    expect = torch.tensor(origins, dtype=gs.tc_float, device=gs.device)[env.levels, env.types]
    pos = env.robot.get_pos()
    assert torch.allclose(env.origins, expect) and torch.allclose(pos[:, :2], expect[:, :2], atol=1e-5)
    assert expect[:, 2].min() < -0.1 and expect[:, 2].max() > 0.1
    height = pos[:, 2] - env.ground_height(pos[:, None, :2])[:, 0]
    assert torch.allclose(height, torch.full_like(height, S2S["base_init_z"]), atol=1e-3)


def test_curriculum_up_down(env):
    reset_at(env, 1)
    move(env, [0], 0.6 * TERRAIN["tile"])  # tile/2 넘게 걸어감 → 다음 난이도
    env.cmd_path[:] = 0.0
    env.cmd_path[1] = 100.0  # 명령 경로의 절반도 못 감 → 이전 난이도
    env.episode_length_buf[:] = env.max_episode_length  # 다음 step에서 전부 시간 초과 리셋
    _, _, done, extras = env.step(zero(env))
    assert done.all()
    assert env.levels.tolist() == [2, 0, 1, 1, 1, 1, 1, 1]
    assert extras["episode"]["terrain_level"] == pytest.approx(env.levels.float().mean().item())
    assert torch.allclose(env.robot.get_pos()[:, :2], env.origins[:, :2], atol=1e-5)  # 새 타일 origin에 다시 섬


def test_curriculum_wraps_top_level(env):  # 최고 레벨을 넘기면 범위 밖 인덱스가 아니라 무작위 레벨(Isaac 방식)
    torch.manual_seed(0)
    drawn = []
    for _ in range(5):
        reset_at(env, ROWS - 1)
        move(env, FLAT, -0.6 * TERRAIN["tile"])  # 아래 행 쪽으로(지형 안)
        env.episode_length_buf[:] = env.max_episode_length
        env.step(zero(env))
        drawn += env.levels[FLAT].tolist()
    assert all(0 <= lv < ROWS for lv in drawn) and len(set(drawn)) > 1, drawn


def test_off_terrain_terminates(env):
    reset_at(env, 0)
    for _ in range(3):
        env.step(zero(env))
    move(env, [0], -10.0)  # heightfield 밖(높이 조회 NaN 영역)
    move(env, [1], -(env.origins[1, 0].item() - 1.0))  # border 안(x 1.0 < border 2.5), heightfield 안
    obs, rew, done, _ = env.step(zero(env))
    assert done.tolist() == [True, True] + [False] * (N_ENVS - 2)
    assert torch.isfinite(rew).all() and torch.isfinite(obs["policy"]).all() and torch.isfinite(obs["privileged"]).all()
    assert torch.allclose(env.robot.get_pos()[:2, :2], env.origins[:2, :2], atol=1e-5)  # 리셋돼 타일로 돌아옴
