"""PPO actor의 numpy 형식. 내보내기 테스트(학습 venv)와 Gazebo rl_node(ROS)가 같은 forward를 쓴다 — numpy만 의존."""
import json

import numpy as np

# torch ELU(alpha=1). 양수 쪽 expm1 오버플로를 막으려고 min(h, 0)
_ACT = {"elu": lambda h: np.where(h > 0, h, np.expm1(np.minimum(h, 0.0)))}


class Policy:
    def __init__(self, layers, activation="elu"):
        self.layers = layers
        self.f = _ACT[activation]
        self.in_dim = layers[0][0].shape[1]

    def act(self, obs: np.ndarray) -> np.ndarray:
        """결정적 평균 행동: 은닉층 ELU, 마지막 층 선형."""
        h = np.asarray(obs)
        if h.shape != (self.in_dim,):
            raise ValueError(f"obs shape {h.shape} != ({self.in_dim},)")
        for w, b in self.layers[:-1]:
            h = self.f(w @ h + b)
        w, b = self.layers[-1]
        return w @ h + b


def save(path, layers, cfg: dict, activation: str = "elu", meta: dict | None = None) -> None:
    """meta = 출처(run·ckpt·iteration·git), 추론엔 안 쓴다."""
    if activation not in _ACT:
        raise ValueError(f"unsupported activation {activation}")
    arrays = {}
    for i, (w, b) in enumerate(layers):
        arrays[f"w{i}"], arrays[f"b{i}"] = w, b
    if meta is not None:
        arrays["meta"] = json.dumps(meta)
    np.savez(path, activation=activation, sim2sim=json.dumps(cfg), **arrays)


def load(path) -> tuple[Policy, dict]:
    with np.load(path) as z:
        n = sum(k.startswith("w") for k in z.files)
        layers = [(z[f"w{i}"], z[f"b{i}"]) for i in range(n)]
        return Policy(layers, str(z["activation"])), json.loads(str(z["sim2sim"]))


def load_meta(path) -> dict:
    """save의 meta, 없던 시절 파일이면 {}."""
    with np.load(path) as z:
        return json.loads(str(z["meta"])) if "meta" in z.files else {}
