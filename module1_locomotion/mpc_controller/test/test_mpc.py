import math

import numpy as np

from mpc_controller.mpc_controller_node import goal_to_cmd, pick_heading, wrap_angle

RES = 0.1
SIZE = 4.0
N = int(round(SIZE / RES))
COMMON = dict(
    resolution=RES,
    size=SIZE,
    lookahead=1.5,
    half_width=0.35,
    obstacle_h=0.15,
    scan_max=math.radians(60.0),
    scan_step=math.radians(10.0),
)


def _cell_centers():
    """grid[row, col]에 대응하는 (cx, cy) 셀 중심 좌표 배열(로봇 프레임)."""
    idx = np.arange(N)
    centers = (idx + 0.5) * RES - SIZE / 2
    cx, cy = np.meshgrid(centers, centers)  # cx: col 방향, cy: row 방향
    return cx, cy


def test_pick_heading_all_nan_returns_goal():
    grid = np.full((N, N), np.nan, dtype=np.float32)
    assert pick_heading(grid, 0.3, **COMMON) == 0.3


def test_pick_heading_all_ground_returns_goal():
    grid = np.zeros((N, N), dtype=np.float32)
    assert pick_heading(grid, 0.3, **COMMON) == 0.3


def test_pick_heading_frontal_obstacle_detours():
    cx, cy = _cell_centers()
    grid = np.full((N, N), np.nan, dtype=np.float32)
    mask = (cx >= 0.8) & (cx <= 1.2) & (cy >= -0.2) & (cy <= 0.2)
    grid[mask] = 0.65
    goal_rel = 0.3

    h = pick_heading(grid, goal_rel, **COMMON)
    assert h is not None
    assert h != goal_rel
    assert abs(h) <= math.radians(60.0) + 1e-9

    # 찾은 헤딩 자체는 막혀있지 않아야 한다 (scan_max=0 → 그 헤딩만 재검사)
    recheck_kwargs = dict(COMMON, scan_max=0.0)
    assert pick_heading(grid, h, **recheck_kwargs) == h


def test_pick_heading_fully_blocked_returns_none():
    grid = np.full((N, N), 0.65, dtype=np.float32)
    assert pick_heading(grid, 0.3, **COMMON) is None


def test_goal_to_cmd():
    assert goal_to_cmd(0.2, 0.0, 0.5, 1.0, 1.5, 0.3) == (0.0, 0.0)
    assert goal_to_cmd(5.0, 0.0, 0.5, 1.0, 1.5, 0.3) == (0.5, 0.0)

    v, w = goal_to_cmd(5.0, 0.5, 0.5, 1.0, 1.5, 0.3)
    assert w == 0.75
    assert v == 0.5 * math.cos(0.5)

    v, w = goal_to_cmd(5.0, math.pi, 0.5, 1.0, 1.5, 0.3)
    assert v == 0.0
    assert w == 1.0


def test_wrap_angle():
    assert math.isclose(wrap_angle(3 * math.pi), math.pi)
    assert math.isclose(wrap_angle(-3 * math.pi), math.pi)
    assert wrap_angle(0.5) == 0.5


if __name__ == "__main__":
    test_pick_heading_all_nan_returns_goal()
    test_pick_heading_all_ground_returns_goal()
    test_pick_heading_frontal_obstacle_detours()
    test_pick_heading_fully_blocked_returns_none()
    test_goal_to_cmd()
    test_wrap_angle()
    print("ok")
