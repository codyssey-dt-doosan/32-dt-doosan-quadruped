"""가스 농도장 모델 (ROS 비의존). plume_sim 노드와 source_seeking 오프라인 시뮬레이터가 같이 쓴다.

시간 평균 2D 가우시안 플룸 + 가스가 고여 국소 최대(local optimum)를 만드는 정체 구역(decoy).
"""

from __future__ import annotations

import math
import random
from dataclasses import MISSING, dataclass, field, fields

import yaml


@dataclass
class GasFieldParams:
    source_x: float = 7.6            # 누출원 (월드 좌표, m)
    source_y: float = -6.0
    release_rate: float = 100.0      # 누출원 농도 (ppm)
    wind_direction: float = 2.85     # rad, 바람이 불어 가는 방향
    wind_speed: float = 0.5          # m/s
    sigma0: float = 0.3              # 누출원에서 플룸 반폭 (m)
    spread: float = 0.25             # 풍하 1 m당 플룸 확산
    near_radius: float = 0.8         # 누출원 주변 확산 (풍상 쪽으로도 조금 퍼짐)
    # 정체 구역 [x, y, 진폭 ppm, sigma m, ...] -> 국소 최대
    decoys: list = field(default_factory=lambda: [-4.5, -4.2, 15.0, 1.3])
    background: float = 0.5
    rel_noise: float = 0.10          # 비례 센서 노이즈 (표준편차, 비율)
    abs_noise: float = 0.3           # 고정 센서 노이즈 (표준편차, ppm)
    wind_dir_noise: float = 0.15     # 풍향계 노이즈 (표준편차, rad)
    seed: int = 0


class GasField:
    def __init__(self, p: GasFieldParams) -> None:
        self.p = p
        self.rng = random.Random(p.seed)
        d = list(p.decoys)
        self.decoys = [tuple(d[i:i + 4]) for i in range(0, len(d) - len(d) % 4, 4)]

    def mean(self, x: float, y: float) -> float:
        """노이즈 없는 농도 (ppm)."""
        p = self.p
        ux, uy = math.cos(p.wind_direction), math.sin(p.wind_direction)
        dx, dy = x - p.source_x, y - p.source_y
        down = dx * ux + dy * uy        # 풍하 거리
        cross = -dx * uy + dy * ux      # 플룸 중심선에서의 거리
        plume = 0.0
        if down > 0.0:
            s = p.sigma0 + p.spread * down
            plume = p.release_rate * (p.sigma0 / s) * math.exp(-cross * cross / (2 * s * s))
        near = p.release_rate * math.exp(-(dx * dx + dy * dy) / (2 * p.near_radius ** 2))
        c = p.background + max(plume, near)
        for (px, py, amp, sig) in self.decoys:
            c += amp * math.exp(-((x - px) ** 2 + (y - py) ** 2) / (2 * sig * sig))
        return c

    def sample(self, x: float, y: float) -> float:
        """노이즈 섞인 센서값."""
        m = self.mean(x, y)
        return max(0.0, m * (1.0 + self.rng.gauss(0.0, self.p.rel_noise)) + self.rng.gauss(0.0, self.p.abs_noise))

    def wind(self) -> tuple[float, float]:
        """노이즈 섞인 풍속 벡터 (wx, wy), 월드 좌표."""
        a = self.p.wind_direction + self.rng.gauss(0.0, self.p.wind_dir_noise)
        return self.p.wind_speed * math.cos(a), self.p.wind_speed * math.sin(a)


def load_params(path: str, node_name: str, cls):
    """ROS 2 params YAML의 `node_name: ros__parameters:` 절로 dataclass를 채운다."""
    with open(path, encoding="utf-8") as f:
        doc = yaml.safe_load(f) or {}
    section = (doc.get(node_name) or {}).get("ros__parameters") or {}
    names = {f.name for f in fields(cls)}
    return cls(**{k: v for k, v in section.items() if k in names})


def declare_dataclass_params(node, cls):
    """dataclass 필드를 ROS 파라미터로 선언하고 채운 dataclass를 돌려준다."""
    values = {}
    for f in fields(cls):
        default = f.default_factory() if f.default_factory is not MISSING else f.default
        if isinstance(default, list) and not default:
            # 빈 리스트는 타입을 추론할 수 없다(BYTE_ARRAY로 잡힘): 실수 배열로 선언
            from rclpy.parameter import Parameter
            v = node.declare_parameter(f.name, Parameter.Type.DOUBLE_ARRAY).value
            values[f.name] = list(v) if v is not None else []
        else:
            values[f.name] = node.declare_parameter(f.name, default).value
    return cls(**values)
