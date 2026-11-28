"""좌우 거울(시상면 반사, y → −y). rsl-rl Symmetry의 data_augmentation_func(rough_v2 대칭 학습).
인덱스는 sim2sim.json joint_names·obs.order에서 모듈 로드 때 한 번 만든다."""
import torch
from tensordict import TensorDict

import cfgs

SWAP = {"FL": "FR", "FR": "FL", "RL": "RR", "RR": "RL"}
# 3차원 블록 부호: 각속도는 축벡터라 반사 때 x·z가 바뀌고, 중력·명령(vx, vy, wz)은 y·wz가 바뀐다
VEC_SIGN = {"ang_vel": (-1., 1., -1.), "projected_gravity": (1., -1., 1.), "commands": (1., -1., -1.)}
JOINT_BLOCKS = ("dof_pos", "dof_vel", "last_action")


def joint_perm_sign(joint_names: list[str]) -> tuple[torch.Tensor, torch.Tensor]:
    """거울[i] = sign[i] · 원본[perm[i]]. FL↔FR·RL↔RR, hip(x축 롤)만 −1, thigh·calf(y축)는 그대로."""
    perm = [joint_names.index(SWAP[n[:2]] + n[2:]) for n in joint_names]
    sign = [-1. if "_hip_" in n else 1. for n in joint_names]
    return torch.tensor(perm), torch.tensor(sign)


def _obs_perm_sign(s2s: dict) -> tuple[torch.Tensor, torch.Tensor]:
    jp, js = joint_perm_sign(s2s["joint_names"])
    blocks = {k: (torch.arange(3), torch.tensor(s)) for k, s in VEC_SIGN.items()} | {k: (jp, js) for k in JOINT_BLOCKS}
    perm, sign, o = [], [], 0
    for k in s2s["obs"]["order"]:
        p, s = blocks[k]
        perm.append(p + o)
        sign.append(s)
        o += len(p)
    return torch.cat(perm), torch.cat(sign)


S2S = cfgs.load_sim2sim()
JOINT_PERM, JOINT_SIGN = joint_perm_sign(S2S["joint_names"])
OBS_PERM, OBS_SIGN = _obs_perm_sign(S2S)
GROUPS = {"policy": (OBS_PERM, OBS_SIGN), "policy_clean": (OBS_PERM, OBS_SIGN),
          "privileged": (torch.arange(5), torch.tensor([1., -1., 1., 1., 1.]))}  # 선속도 (vx, vy, vz)·마찰·추가 질량


def _apply(x: torch.Tensor, perm: torch.Tensor, sign: torch.Tensor) -> torch.Tensor:
    return x[..., perm.to(x.device)] * sign.to(x.device, x.dtype)


def mirror_obs(group: str, x: torch.Tensor) -> torch.Tensor:
    return _apply(x, *GROUPS[group])


def mirror_actions(a: torch.Tensor) -> torch.Tensor:
    return _apply(a, JOINT_PERM, JOINT_SIGN)


def data_augmentation(env, obs: TensorDict | None, actions: torch.Tensor | None) -> tuple[TensorDict | None, torch.Tensor | None]:
    """원본 뒤에 거울을 배치 축으로 붙인다(크기 2배, 앞 절반 = 원본). None은 None 그대로(Symmetry는 한쪽만 넘기기도 한다)."""
    if obs is not None:
        m = TensorDict({k: mirror_obs(k, v) for k, v in obs.items()}, batch_size=obs.batch_size, device=obs.device)
        obs = torch.cat([obs, m])
    if actions is not None:
        actions = torch.cat([actions, mirror_actions(actions)])
    return obs, actions
