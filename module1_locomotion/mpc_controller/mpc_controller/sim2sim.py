"""Genesis 학습과 Gazebo rl_node가 함께 읽는 sim2sim.json 로더. ROS 의존은 load() 안에서만."""
import json
from pathlib import Path

_OBS_SIZE = {"ang_vel": 3, "projected_gravity": 3, "commands": 3}

# 배포 계약: rl_node가 관측을 만들고 행동을 서보 명령으로 바꿀 때 읽는 키. 다르면 정책이 다른 입출력을 기대한다.
# 나머지(armature·sim_dt·dr·nominal 등)는 학습 쪽 물리·랜덤화 조건이다.
CONTRACT = ("joint_names", "q_nom", "kp", "kd", "action_scale", "clip_actions", "policy_hz", "imu_hz",
            "tilt_max_deg", "obs", "commands")


def load(path=None) -> dict:
    if path is None:
        from ament_index_python.packages import get_package_share_directory

        path = Path(get_package_share_directory("mpc_controller")) / "config" / "sim2sim.json"
    return json.loads(Path(path).read_text())


def obs_dim(cfg: dict) -> int:
    n = len(cfg["joint_names"])
    return sum(_OBS_SIZE.get(k, n) for k in cfg["obs"]["order"])


def diff(a, b, prefix="") -> list:
    """다른 키를 점 표기로 반환. float 허용오차 1e-9, 리스트는 원소별 비교."""
    if isinstance(a, dict) and isinstance(b, dict):
        out = []
        for k in sorted(a.keys() | b.keys()):
            p = f"{prefix}.{k}" if prefix else k
            out += [p] if k not in a or k not in b else diff(a[k], b[k], p)
        return out
    if isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            return [prefix]
        return [prefix] if any(diff(x, y, "x") for x, y in zip(a, b)) else []
    if isinstance(a, (int, float)) and isinstance(b, (int, float)) and not isinstance(a, bool):
        return [] if abs(a - b) <= 1e-9 else [prefix]
    return [] if a == b else [prefix]


def split_diff(a, b) -> tuple[list, list]:
    """diff를 (배포 계약 키, 학습 전용 키)로 나눈다."""
    keys = diff(a, b)
    contract = [k for k in keys if k.split(".")[0] in CONTRACT]
    return contract, [k for k in keys if k not in contract]
