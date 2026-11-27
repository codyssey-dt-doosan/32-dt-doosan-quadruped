"""지형 커리큘럼 heightfield(스펙 §4.2). 행 = 난이도(x), 열 = 타입(y), 둘레 border. numpy만 — Genesis 초기화 없음."""
import numpy as np


def difficulty(row: int, rows: int) -> float:
    return row / max(rows - 1, 1)


def make_tile(kind: str, d: float, cfg: dict, rng) -> np.ndarray:
    hs = cfg["hs"]
    n = round(cfg["tile"] / hs)
    c, pc = n // 2, round(cfg["platform"] / 2 / hs)  # 가운데 꼭짓점, 플랫폼 반폭 [셀]
    i = np.arange(n + 1)
    rho = np.maximum.outer(np.abs(i - c), np.abs(i - c))  # 가운데에서 체비셰프 거리 [셀]
    sign = -1.0 if kind.endswith("_up") else 1.0  # _up = 가운데가 낮은 구덩이(나가려면 오른다), _down = 피라미드
    if kind == "flat":
        return np.zeros((n + 1, n + 1))
    if kind == "rough":
        a = 0.01 + (cfg["rough_max"] - 0.01) * d
        z = rng.uniform(-a, a, (n + 1, n + 1))
        z[rho <= pc] = 0.0
        return z
    if kind in ("slope_up", "slope_down"):
        return sign * np.tan(np.radians(cfg["slope_deg_max"] * d)) * hs * (c - np.maximum(rho, pc))
    if kind in ("stairs_up", "stairs_down"):
        t = round(cfg["tread"] / hs)
        return sign * cfg["step_h_max"] * d * np.minimum((c - rho) // t, (c - pc) // t)
    if kind == "pipes":
        r = cfg["pipe_d_max"] * d / 2
        m = int((cfg["tile"] / 2 - cfg["pipe_d_max"] / 2 - cfg["platform"] / 2) // cfg["pipe_gap"])  # 타일 경계 안에 드는 수
        offs = np.rint((cfg["platform"] / 2 + cfg["pipe_gap"] * np.arange(1, m + 1)) / hs)  # 능선 중심을 꼭짓점에 맞춘다
        dx = np.abs(i[:, None] - (c + np.concatenate([-offs, offs]))) * hs
        z = np.where(dx <= r, r + np.sqrt(np.clip(r * r - dx * dx, 0.0, None)), 0.0).max(axis=1, initial=0.0)
        return np.tile(z[:, None], (1, n + 1))  # 축 y
    raise ValueError(f"알 수 없는 지형 타입: {kind}")


def make_heightfield(cfg: dict, seed: int = 0) -> tuple[np.ndarray, np.ndarray]:
    hs, rows, cols = cfg["hs"], cfg["rows"], cfg["cols"]
    n, b = round(cfg["tile"] / hs), round(cfg["border"] / hs)
    rng = np.random.default_rng(seed)
    hf = np.zeros((rows * n + 2 * b + 1, len(cols) * n + 2 * b + 1))
    origins = np.zeros((rows, len(cols), 3))
    for r in range(rows):
        for k, kind in enumerate(cols):
            x0, y0 = b + r * n, b + k * n
            tile = make_tile(kind, difficulty(r, rows), cfg, rng)
            hf[x0:x0 + n + 1, y0:y0 + n + 1] = tile  # 경계 꼭짓점 한 줄은 이웃 타일과 공유
            origins[r, k] = ((x0 + n // 2) * hs, (y0 + n // 2) * hs, tile[n // 2, n // 2])
    return hf, origins
