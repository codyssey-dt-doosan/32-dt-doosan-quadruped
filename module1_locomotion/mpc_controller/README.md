# mpc_controller

담당: **도훈**

`/patrol/goal`(PoseStamped)을 향해 elevation map 위에서 장애물을 피하는 `/cmd_vel`을 10 Hz로 낸다.

```bash
ros2 launch mpc_controller mpc_controller.launch.py planner:=mpc          # 기본
ros2 launch mpc_controller mpc_controller.launch.py planner:=heading_scan # 폴백·비교용
```

## 플래너

| `planner` | 방식 |
|-----------|------|
| `mpc` (기본) | 샘플링 MPC. 유니사이클 지평 `horizon`×`dt`(10×0.2 s), 입력 (v,w)를 앞·뒤 2구간으로 나눠 (3×`n_w`)² = 729 시퀀스 롤아웃. 발자국 반경(`half_width`)만큼 팽창한 점유 셀에 닿는 시퀀스와 정지 시퀀스 제외. 비용 = 종단 goal 거리 + `w_turn`·Σ\|w\|dt. 최소 비용 시퀀스의 첫 (v,w) 실행, 매 틱 재계획 |
| `heading_scan` | goal 방향 ±`scan_max_deg` 후보를 `scan_step_deg` 간격으로, 전방 `lookahead`×±`half_width` 통로가 비는 첫 헤딩 선택. 지평 1스텝 |

둘 다 전부 막히면 goal 쪽으로 제자리 선회(`blocked_cmd`). `dist < stop_dist`면 정지.

## 안전 정지

goal·odom·elevation_map 중 하나라도 `timeout`(1 s) 미수신 → 0. `/fall_recovery/status`가 fallen·recovering·failed(최근 `fall_timeout` 2 s 안 수신) → 0.

## 파라미터

공통: `v_max` 0.5 · `w_max` 1.0 · `stop_dist` 0.3 · `half_width` 0.35 · `obstacle_h` 0.15 · `resolution` 0.1 · `size` 4.0 · `timeout` 1.0 · `fall_timeout` 2.0
mpc: `horizon` 10 · `dt` 0.2 · `n_w` 9 · `w_turn` 0.1
heading_scan: `lookahead` 1.5 · `scan_max_deg` 60 · `scan_step_deg` 10 · `k_ang` 1.5

## 테스트·실험

```bash
python -m pytest test/ -q -p no:launch_testing -p no:launch_ros
python3 scripts/patrol_metrics.py --world corridor   # full_system 떠 있는 상태에서 한 바퀴 측정
```
