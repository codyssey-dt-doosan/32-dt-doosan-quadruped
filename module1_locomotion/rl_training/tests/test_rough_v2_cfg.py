"""rough_v2 회전 편향 대책 설정: tracking_ang_vel 0.5·wz = 0 명령 30%·rsl-rl Symmetry(mirror.data_augmentation).
wz = 0 비율은 CPU 평지 64 env × 64회 재표본 = 경로마다 4096 표본으로 0.3 ± 0.03을 본다."""
import pickle
import sys
from pathlib import Path

import pytest
import torch
from rsl_rl.runners import OnPolicyRunner

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import genesis as gs  # noqa: E402

import cfgs  # noqa: E402
import mirror  # noqa: E402
from go2_box_env import Go2BoxEnv  # noqa: E402

S2S = cfgs.load_sim2sim()
N_ENVS, N_DRAWS = 64, 64


@pytest.fixture(scope="module")
def env():
    if not gs._initialized:  # 다른 테스트 파일이 먼저 돌았을 수 있다
        gs.init(backend=gs.cpu, logging_level="warning")
    return Go2BoxEnv(N_ENVS, *cfgs.get_cfgs(S2S, dr=False))


def test_reward_and_command_values():
    _, _, rew, cmd = cfgs.get_cfgs(S2S)
    assert rew["reward_scales"]["tracking_ang_vel"] == 0.5 and cmd["zero_ang_vel_prob"] == 0.3


def test_symmetry_cfg():
    s = cfgs.get_train_cfg("x", 1)["algorithm"]["symmetry_cfg"]
    assert s == {"data_augmentation_func": mirror.data_augmentation, "use_data_augmentation": True,
                 "use_mirror_loss": True, "mirror_loss_coeff": 10.0}
    assert s["data_augmentation_func"] is mirror.data_augmentation


def test_train_cfg_picklable(env):  # train.py가 runner.cfg를 cfgs.pkl에 남긴다 — rsl-rl은 env를 cfg 복사본에만 넣어야 한다
    runner = OnPolicyRunner(env, cfgs.get_train_cfg("x", 1), device=gs.device)
    assert runner.alg.symmetry is not None
    pickle.loads(pickle.dumps(runner.cfg))


def check_zero_wz(cmds):  # (n, 3) 표본: wz = 0이 30%, 그 행의 vx는 0으로 접히지 않고 U(−0.2, 0.5)에 퍼져 있다
    zero = cmds[:, 2] == 0
    assert abs(zero.float().mean() - 0.3) < 0.03
    vx = cmds[zero, 0]
    assert -0.2 <= vx.min() < 0 < vx.max() <= 0.5


def test_zero_ang_vel_prob_both_paths(env):  # reset(envs_idx=None)·주기 재표본(bool 마스크) 두 경로
    torch.manual_seed(0)
    cmds = []
    for _ in range(N_DRAWS):
        env._resample_commands(None)
        cmds.append(env.commands.clone())
    check_zero_wz(torch.cat(cmds))

    mask = torch.arange(N_ENVS, device=gs.device) % 2 == 0  # 마스크 밖 env는 명령을 그대로 둔다
    cmds = []
    for _ in range(2 * N_DRAWS):
        before = env.commands.clone()
        env._resample_commands(mask)
        assert torch.equal(env.commands[~mask], before[~mask])
        cmds.append(env.commands[mask])
    check_zero_wz(torch.cat(cmds))
