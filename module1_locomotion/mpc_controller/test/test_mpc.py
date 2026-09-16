import math

import numpy as np

from mpc_controller.mpc_controller_node import blocked_cmd, goal_to_cmd, halt_for_fall, pick_heading, wrap_angle

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


def test_pick_heading_scan_step_zero_no_hang():
    # scan_step<=0이면 후보를 goal_rel 하나로만 좁혀야 한다(무한루프 금지)
    grid = np.full((N, N), np.nan, dtype=np.float32)
    kwargs = dict(COMMON, scan_step=0.0)
    assert pick_heading(grid, 0.3, **kwargs) == 0.3


def test_blocked_cmd():
    # 막힘 시 제자리 선회: goal_rel 부호 방향으로 w_max, goal_rel==0이면 양의 방향
    assert blocked_cmd(0.0, 1.0) == (0.0, 1.0)
    assert blocked_cmd(-0.2, 1.0) == (0.0, -1.0)


def test_pick_heading_corridor_regression():
    # 전방 1.3m, 우측(y<0)에 걸친 장애물 → 왼쪽(+)으로 10도 회피
    cx, cy = _cell_centers()
    grid_right = np.full((N, N), np.nan, dtype=np.float32)
    grid_right[(cx >= 1.2) & (cx <= 1.4) & (cy >= -0.7) & (cy <= -0.3)] = 0.65
    h_right = pick_heading(grid_right, 0.0, **COMMON)
    assert h_right is not None
    assert h_right > 0.0
    assert math.isclose(h_right, math.radians(10.0))

    # 전방 1.3m, 좌측(y>0)에 걸치되 half_width(0.35) 밖 → 직진 그대로 통과
    grid_left = np.full((N, N), np.nan, dtype=np.float32)
    grid_left[(cx >= 1.2) & (cx <= 1.4) & (cy >= 0.4) & (cy <= 0.8)] = 0.65
    h_left = pick_heading(grid_left, 0.0, **COMMON)
    assert h_left == 0.0


# elevation_map_cb의 형식 오류 가드(dim 길이 부족/shape 불일치)는 rclpy.init/Node 생성이 필요해
# 이 순수 함수 테스트 파일에서는 생략한다. task-1-report.md Final fix wave 섹션에 명시.


if __name__ == "__main__":
    test_pick_heading_all_nan_returns_goal()
    test_pick_heading_all_ground_returns_goal()
    test_pick_heading_frontal_obstacle_detours()
    test_pick_heading_fully_blocked_returns_none()
    test_goal_to_cmd()
    test_wrap_angle()
    test_pick_heading_scan_step_zero_no_hang()
    test_blocked_cmd()
    test_pick_heading_corridor_regression()
    print("ok")


def test_halt_for_fall():
    assert halt_for_fall("fallen") is True
    assert halt_for_fall("recovering") is True
    assert halt_for_fall("failed") is True
    assert halt_for_fall("idle") is False
    assert halt_for_fall("recovered") is False
    assert halt_for_fall(None) is False  # fall_recovery 없이 단독 실행
