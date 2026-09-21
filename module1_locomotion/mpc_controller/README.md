# mpc_controller

담당: **도훈**

goal(PoseStamped)을 향해 elevation map 위에서 장애물을 피하는 `/cmd_vel`을 10 Hz로 낸다. `/cmd_vel` 발행자는 이 노드뿐이다.

## goal 계약 (순찰·가스·복귀 공통)

`goal_topics` 우선순위 순: `/return_to_home/goal` > `/source_seeking/goal` > `/patrol/goal`. 매 틱 위에서부터 첫 **활성** goal을 따른다.

- 활성 = `header.frame_id`가 비어 있지 않고(`"map"`) 최근 `timeout`(1 s) 안에 수신
- **비활성은 `frame_id=""`인 `PoseStamped()`를 계속 발행**하거나 발행을 멈추면 된다. 발행자가 죽어도 1 s 뒤 자동으로 다음 순위로 내려간다
- 활성 goal이 하나도 없으면 정지. 소스가 바뀔 때 로그 1줄

```bash
ros2 launch mpc_controller mpc_controller.launch.py planner:=mpc          # 기본
ros2 launch mpc_controller mpc_controller.launch.py planner:=heading_scan # 폴백·비교용
```

## 플래너

| `planner` | 방식 |
|-----------|------|
| `mpc` (기본) | 헤딩 스캔이 준 자유 헤딩 방향 `min(dist, lookahead)` 지점을 참조 goal로 삼는 샘플링 MPC. 유니사이클 지평 `horizon`×`dt`(10×0.2 s), 입력 (v,w)를 앞·뒤 2구간으로 나눠 (3×`n_w`)² = 729 시퀀스 롤아웃. 발자국 반경(`half_width`)만큼 팽창한 점유 셀에 닿는 시퀀스와 정지 시퀀스 제외. 비용 = 지평 goal 거리 평균 + `w_head`·\|종단 heading 오차\| + `w_turn`·Σ\|w\|dt. 최소 비용 시퀀스의 첫 (v,w) 실행, 매 틱 재계획 |
| `heading_scan` | goal 방향 ±`scan_max_deg` 후보를 `scan_step_deg` 간격으로, 전방 `lookahead`×±`half_width` 통로가 비는 첫 헤딩 선택. 지평 1스텝 |

둘 다 전부 막히면 goal 쪽으로 제자리 선회(`blocked_cmd`). `dist < stop_dist`면 정지.

## 안전 정지

goal·odom·elevation_map 중 하나라도 `timeout`(1 s) 미수신 → 0. `/fall_recovery/status`가 fallen·recovering·failed(최근 `fall_timeout` 2 s 안 수신) → 0.

## 파라미터

공통: `v_max` 0.5 · `w_max` 1.0 · `stop_dist` 0.3 · `half_width` 0.35 · `obstacle_h` 0.15 · `resolution` 0.1 · `size` 4.0 · `timeout` 1.0 · `fall_timeout` 2.0

`resolution`/`size`는 elevation_map과 같아야 한다. 런치 인자 `map_resolution`/`map_size`(full_system 포함)가 두 노드에 같이 넘기므로 노드 파라미터를 따로 바꾸지 말 것. 셀 수가 안 맞으면 error 로그 후 그리드를 버려 timeout 정지.
mpc: `horizon` 10 · `dt` 0.2 · `n_w` 9 · `w_turn` 0.1 · `w_head` 0.5 · `v_min` 0.0(<0이면 후진 후보 추가, 예 -0.25. 포켓 갇힘 재발 시)
heading_scan(참조 헤딩으로 mpc도 사용): `lookahead` 1.5 · `scan_max_deg` 60 · `scan_step_deg` 10 · `hysteresis_deg` 15(직전 헤딩이 자유이고 goal 오차가 최적 후보보다 이만큼 이상 나쁘지 않으면 유지, 좌/우 채터링 방지) · `k_ang` 1.5(heading_scan 전용)

## 테스트·실험

```bash
python -m pytest test/ -q -p no:launch_testing -p no:launch_ros
python3 scripts/patrol_metrics.py --world corridor   # full_system 떠 있는 상태에서 한 바퀴 측정
```
