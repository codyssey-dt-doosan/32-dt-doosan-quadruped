import math

import numpy as np

from mpc_controller.mpc_controller_node import (
    blocked_cmd,
    goal_to_cmd,
    grid_matches,
    halt_for_fall,
    inflate,
    pick_goal_index,
    pick_heading,
    plan_mpc,
    reference_goal,
    rollout,
    scan_offsets,
    wrap_angle,
)

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
MPC = dict(
    resolution=RES,
    size=SIZE,
    v_max=0.5,
    w_max=1.0,
    half_width=0.35,
    obstacle_h=0.15,
    horizon=10,
    dt=0.2,
    n_w=9,
    w_turn=0.1,
    w_head=0.5,
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


def test_scan_offsets_float_boundary():
    # 35/7 = 4.999… → int()면 4. +1e-9로 5 → 0 + ±1..±5 = 11개
    assert len(scan_offsets(math.radians(35.0), math.radians(7.0))) == 11
    assert len(scan_offsets(math.radians(60.0), math.radians(10.0))) == 13
    assert scan_offsets(math.radians(60.0), 0.0) == [0.0]
    offs = scan_offsets(math.radians(20.0), math.radians(10.0))
    assert offs[0] == 0.0 and offs[1] > 0 and offs[2] < 0  # goal 가까운 순, +먼저


def test_inflate_square_window():
    occ = np.zeros((20, 20), dtype=bool)
    occ[10, 10] = True
    out = inflate(occ, 2)
    assert out.sum() == 25
    assert out[8:13, 8:13].all()
    assert not out[7, 10] and not out[13, 10]


def test_inflate_edge_and_zero():
    occ = np.zeros((5, 5), dtype=bool)
    occ[0, 0] = True
    assert inflate(occ, 1).sum() == 4
    same = inflate(occ, 0)
    assert same.sum() == 1 and same is not occ


def test_rollout_straight_and_turn():
    dt = 0.2
    v = np.array([[0.5] * 5, [0.0] * 5])
    w = np.array([[0.0] * 5, [1.0] * 5])
    x, y, th = rollout(v, w, dt)
    assert x.shape == (2, 5)
    assert np.allclose(x[0], [0.1, 0.2, 0.3, 0.4, 0.5])
    assert np.allclose(y[0], 0.0)
    assert np.allclose(th[1], [0.2, 0.4, 0.6, 0.8, 1.0])
    assert np.allclose(x[1], 0.0) and np.allclose(y[1], 0.0)
    # 회전하며 전진: 첫 스텝은 θ=0으로 이동, 둘째 스텝은 θ=0.2로 이동
    v2 = np.array([[0.5, 0.5]])
    w2 = np.array([[1.0, 1.0]])
    x2, y2, _ = rollout(v2, w2, dt)
    assert np.isclose(x2[0, 0], 0.1) and np.isclose(y2[0, 0], 0.0)
    assert np.isclose(x2[0, 1], 0.1 + 0.1 * math.cos(0.2))
    assert np.isclose(y2[0, 1], 0.1 * math.sin(0.2))


def test_plan_mpc_empty_grid_goal_ahead():
    grid = np.full((N, N), np.nan, dtype=np.float32)
    v, w = plan_mpc(grid, (2.0, 0.0), **MPC)
    assert v == 0.5
    assert abs(w) < 0.05


def test_plan_mpc_goal_left_turns_left():
    grid = np.full((N, N), np.nan, dtype=np.float32)
    v, w = plan_mpc(grid, (0.0, 2.0), **MPC)
    assert w > 0.0


def test_plan_mpc_frontal_obstacle_detours():
    cx, cy = _cell_centers()
    grid = np.full((N, N), np.nan, dtype=np.float32)
    grid[(cx >= 0.8) & (cx <= 1.2) & (cy >= -0.2) & (cy <= 0.2)] = 0.65
    res = plan_mpc(grid, (2.0, 0.0), **MPC)
    assert res is not None
    v, w = res
    assert not (v == 0.5 and w == 0.0)  # 전속 직진은 팽창된 장애물(x≥0.4)과 충돌하므로 선택 불가


def test_plan_mpc_enclosed_returns_none():
    cx, cy = _cell_centers()
    grid = np.full((N, N), np.nan, dtype=np.float32)
    r = np.hypot(cx, cy)
    grid[(r >= 0.5) & (r <= 0.7)] = 0.65
    assert plan_mpc(grid, (2.0, 0.0), **MPC) is None


def test_plan_mpc_outside_grid_is_free():
    # 1 m 그리드(n=10)라 지평 1 m 롤아웃이 그리드를 벗어남 → 인덱스 오류 없이 자유 취급
    small = np.full((10, 10), np.nan, dtype=np.float32)
    v, w = plan_mpc(small, (5.0, 0.0), **dict(MPC, size=1.0))
    assert v == 0.5 and abs(w) < 0.05


def test_plan_mpc_time_budget():
    import time

    cx, cy = _cell_centers()
    grid = np.zeros((N, N), dtype=np.float32)
    grid[(cx >= 0.8) & (cx <= 1.2) & (cy >= -0.2) & (cy <= 0.2)] = 0.65
    plan_mpc(grid, (2.0, 0.3), **MPC)  # warm-up
    t0 = time.perf_counter()
    for _ in range(10):
        plan_mpc(grid, (2.0, 0.3), **MPC)
    per_call = (time.perf_counter() - t0) / 10
    assert per_call < 0.02, f"plan_mpc {per_call*1e3:.1f} ms"


def test_plan_mpc_near_goal_moves_now():
    # goal 0.6 m 앞: '대기 후 전진'과 '전진 후 대기'가 종단 거리 동률이면 정지 명령이 뽑혀 영구 정지(실측 버그).
    # 지평 평균 거리 비용이면 지금 움직이는 쪽이 이겨야 한다.
    grid = np.full((N, N), np.nan, dtype=np.float32)
    v, w = plan_mpc(grid, (0.6, 0.0), **MPC)
    assert v > 0.0
    assert abs(w) < 0.05


def test_plan_mpc_goal_behind_turns():
    # goal 정반대(east_end→home 전환 실측 버그): 2 s 지평 안엔 어떤 전진도 거리를 늘려
    # 회전 페널티가 이기면 (0,0)이 뽑혀 영구 정지. heading 오차 항이 있으면 제자리 회전이 뽑혀야 한다.
    grid = np.full((N, N), np.nan, dtype=np.float32)
    v, w = plan_mpc(grid, (-5.0, 0.0), **MPC)
    assert abs(w) > 0.5
    # 살짝 비켜 뒤(−170°)면 회전 방향이 goal 쪽(+)이어야 한다
    v2, w2 = plan_mpc(grid, (-5.0 * math.cos(math.radians(10)), 5.0 * math.sin(math.radians(10))), **MPC)
    assert w2 > 0.5


# 2026-09-17 corridor 실측 덤프: obstacle_2 동쪽 1.1 m, 벽이 대각선으로 보이는 프레임. 로봇이 팽창 장애물 0.2 m 앞 포켓에 갇혀
# plan_mpc(실제 goal)가 (0, ±1) 회전만 반복하던 상황. (row, col) 점유 셀.
POCKET_OCC = [
    (17, 26), (17, 27), (18, 26), (19, 26), (20, 26), (21, 26), (27, 0), (27, 1), (27, 2), (28, 2), (28, 3),
    (28, 4), (28, 5), (28, 6), (28, 7), (28, 8), (28, 9), (29, 9), (29, 10), (29, 11), (29, 12), (29, 13),
    (29, 14), (29, 15), (30, 14), (30, 15), (30, 16), (30, 17), (30, 18), (30, 19), (30, 20), (30, 21),
    (31, 21), (31, 22), (31, 23), (31, 24), (31, 25), (31, 26), (31, 27), (32, 27), (32, 28), (32, 29),
    (32, 30), (32, 31), (32, 32), (32, 33), (33, 33), (33, 34), (33, 35), (33, 36), (33, 37), (33, 38), (34, 39),
]


def test_reference_goal():
    assert reference_goal(15.0, 0.0, 1.5) == (1.5, 0.0)
    x, y = reference_goal(0.8, math.pi / 2, 1.5)
    assert abs(x) < 1e-9 and math.isclose(y, 0.8)


def test_plan_mpc_pocket_regression_with_reference_heading():
    grid = np.full((N, N), np.nan, dtype=np.float32)
    for r, c in POCKET_OCC:
        grid[r, c] = 0.65
    dist, goal_rel = 15.12, 0.018
    # 실제 goal을 그대로 주면 포켓에서 회전만 나온다(원인 기록)
    v0, w0 = plan_mpc(grid, (dist * math.cos(goal_rel), dist * math.sin(goal_rel)), **MPC)
    assert v0 == 0.0 and abs(w0) == 1.0
    # 헤딩 스캔 참조(−59°)를 가상 goal로 주면 자유 방향으로 일관되게 회전해 탈출
    h = pick_heading(grid, goal_rel, **COMMON)
    assert h is not None and h < math.radians(-40)
    v, w = plan_mpc(grid, reference_goal(dist, h, 1.5), **MPC)
    assert w < -0.5


def test_grid_matches_geometry():
    assert grid_matches(np.zeros((N, N)), RES, SIZE)
    assert not grid_matches(np.zeros((N, N + 1)), RES, SIZE)  # 정사각 아님
    assert not grid_matches(np.zeros((20, 20)), RES, SIZE)  # elevation_map만 size 2.0으로 바꾼 경우
    assert not grid_matches(np.zeros((N, N)), 0.2, SIZE)  # mpc만 resolution 바꾼 경우


def test_pick_goal_index_priority_and_activity():
    # 리스트 순서 = 우선순위. 활성 = frame_id 비어 있지 않고 timeout 안 수신
    assert pick_goal_index(["map", "map", "map"], [0.1, 0.1, 0.1], 1.0) == 0
    assert pick_goal_index(["", "", "map"], [0.1, 0.1, 0.1], 1.0) == 2  # 채현 스텁(빈 PoseStamped)은 비활성
    assert pick_goal_index([None, "map", "map"], [None, 0.1, 0.1], 1.0) == 1  # 미수신 건너뜀
    assert pick_goal_index(["map", "", "map"], [5.0, 0.1, 0.1], 1.0) == 2  # 상위가 stale이면 다음
    assert pick_goal_index(["", "", ""], [0.1, 0.1, 0.1], 1.0) is None
    assert pick_goal_index([None, None], [None, None], 1.0) is None


def _center_block_grid():
    """전방 0.5~0.9 m, 좌우 ±0.3 m 벽. goal 정면이면 0° 막히고 ±10°는 동률 후보."""
    grid = np.full((N, N), np.nan, dtype=np.float32)
    cx, cy = _cell_centers()
    grid[(cx > 0.5) & (cx < 0.9) & (np.abs(cy) < 0.3)] = 0.65
    return grid


def test_pick_heading_hysteresis_keeps_prev():
    grid = _center_block_grid()
    best = pick_heading(grid, 0.0, **COMMON)
    assert best is not None and best > 0  # 스캔 순서상 +10° 먼저
    prev = -best  # 직전 틱은 -10°를 골랐다
    hyst = math.radians(15.0)
    assert pick_heading(grid, 0.0, **COMMON, prev=prev, hysteresis=hyst) == prev  # 동률이면 유지
    assert pick_heading(grid, 0.0, **COMMON, prev=prev, hysteresis=0.0) == best  # 0이면 기존 동작
    # prev가 막히면 무시
    assert pick_heading(grid, 0.0, **COMMON, prev=0.0, hysteresis=hyst) == best
    # goal이 크게 돌면(prev가 hysteresis 이상 나쁨) 새 후보
    assert pick_heading(grid, math.radians(40), **COMMON, prev=prev, hysteresis=hyst) != prev
