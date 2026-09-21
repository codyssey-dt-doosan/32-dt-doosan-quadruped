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
| `mpc` (기본) | 헤딩 스캔이 준 자유 헤딩 방향 `min(dist, lookahead)` 지점을 참조 goal로 삼는 샘플링 MPC. 유니사이클 지평 `horizon`×`dt`(10×0.2 s), 입력 (v,w)를 앞·뒤 2구간으로 나눠 (3×`n_w`)² = 729 시퀀스 롤아웃. 원본 장애물 셀에 닿거나 발자국 반경(`half_width`) **원판** 팽창 셀에 **새로 진입**하는 시퀀스와 정지 시퀀스 제외(시작이 이미 팽창 안이면 나가는 건 허용 — 정사각 팽창·전면 금지였을 때 0.9 m 통로가 닫혀 제자리 회전에 갇혔음, 2026-09-21). 비용 = 지평 goal 거리 평균 + `w_head`·\|종단 heading 오차\| + `w_turn`·Σ\|w\|dt. 최소 비용 시퀀스의 첫 (v,w) 실행, 매 틱 재계획 |
| `heading_scan` | goal 방향 ±`scan_max_deg` 후보를 `scan_step_deg` 간격으로, 전방 `lookahead`×±`half_width` 통로가 비는 첫 헤딩 선택. 지평 1스텝 |

둘 다 전부 막히면 goal 쪽으로 제자리 선회(`blocked_cmd`). `dist < stop_dist`면 정지.

## 안전 정지

goal·odom·elevation_map 중 하나라도 `timeout`(1 s) 미수신 → 0. `/fall_recovery/status`가 fallen·recovering·failed(최근 `fall_timeout` 2 s 안 수신) → 0.

## 파라미터

공통: `v_max` 0.5 · `w_max` 1.0 · `stop_dist` 0.3 · `half_width` 0.35 · `obstacle_h` 0.15 · `resolution` 0.1 · `size` 4.0 · `timeout` 1.0 · `fall_timeout` 2.0

`resolution`/`size`는 elevation_map과 같아야 한다. 런치 인자 `map_resolution`/`map_size`(full_system 포함)가 두 노드에 같이 넘기므로 노드 파라미터를 따로 바꾸지 말 것. 셀 수가 안 맞으면 error 로그 후 그리드를 버려 timeout 정지.
mpc: `horizon` 10 · `dt` 0.2 · `n_w` 9 · `w_turn` 0.1 · `w_head` 0.5 · `v_min` 0.0(<0이면 후진 후보 추가, 예 -0.25. 포켓 갇힘 재발 시)
heading_scan(참조 헤딩으로 mpc도 사용): `lookahead` 1.5 · `scan_max_deg` 60 · `scan_step_deg` 10 · `hysteresis_deg` 15(직전 헤딩이 자유이고 goal 오차가 최적 후보보다 이만큼 이상 나쁘지 않으면 유지, 좌/우 채터링 방지) · `k_ang` 1.5(heading_scan 전용)

## 다리 애니메이션 (`leg_animation_node`)

시각 효과 전용. 몸체 이동은 VelocityControl이 하고 다리는 추진에 기여하지 않는다. `/cmd_vel` 속도(`|v| + turn_weight·|w|`)에 비례한 트롯 사인파를 50 Hz로 발행 → `model.sdf`의 `JointPositionController` 4개(대각 쌍 A=FL·RR, B=FR·RL × thigh/calf)가 추종. hip 관절은 스프링에 둠. 토픽 `/leg_animation/{thigh,calf}_{a,b}`(Float64). 끄기: 런치 인자 `leg_animation:=false`.

파라미터: `amp_thigh` 0.3 · `amp_calf` 0.5 · `stride_hz_per_mps` 4.0 · `stride_hz_max` 2.5 · `speed_full` 0.15 · `turn_weight` 0.3 · `cmd_timeout` 0.5

## 다리 구동 go2 — trot 보행 (`legged.launch.py`, 옵트인)

CoM·접지력 MPC(C안)로 가는 첫 단계. **기본 구동과 별개**: `full_system.launch.py`는 그대로 VelocityControl + mpc이고, 이 런치는 다리로 서는 go2만 띄우고 `/cmd_vel` (v, w)를 다리로 따라간다. `full_system` 통합은 아직 — 순찰 불가.

```bash
ros2 launch mpc_controller legged.launch.py                        # 기립, /cmd_vel (v, w)를 trot으로 추종
ros2 launch mpc_controller legged.launch.py gui:=false trot:=true  # 헤드리스, /cmd_vel 없이도 제자리 trot
```

- 모델은 복사본이 아니라 **런치 시 `simulation/models/go2/model.sdf`를 변환**(`legged_model.py`): VelocityControl 제거, 관절별 `JointPositionController` 12개, 다리 마찰 `mu`, 관절 스프링 제거. 원본 구조가 바뀌면 `legged 변환 실패[…]`로 런치가 멈춘다 → 메시지의 단계를 보고 `legged_model.py`를 맞출 것.
- 다중 `<joint_name>` 컨트롤러는 첫 관절만 피드백한다(묶으면 전도) → 관절마다 하나.
- `gait_node`: 곧은 다리 → 명목 자세(`d_nominal` 0.25) 2 s 램프 후 대각 쌍(FL·RR / FR·RL) trot. `/cmd_vel` (v, w)를 0.3 s로 평활해, 지지 발을 뒤로 쓸어 전진하고 좌우 보폭 차이로 회전한다. **개루프**(균형·속도 피드백 없음) — 남는 추종 오차는 위층이 odom으로 닫는다. 토픽 `/legged/<joint>`(Float64) × 12.
- 보정 손잡이: `turn_arm` 0.38(회전 — 기하 0.14로는 앞뒤 발 횡마찰에 42%만 돈다. 2 Hz에선 0.33, 3 Hz에선 0.38이 맞았다), `v_gain` 1.0(전진), `x_max` 0.10(반보폭 한계). `mu`·`p_gain`·`stride_hz`가 바뀌면 다시 맞출 것.
- 런치 인자: `world` · `gui` · `trot` · `mu` 0.8 · `p_gain` 120 · `d_gain` 2.0. 노드 파라미터: `d_nominal` · `lift` · `stride_hz` 3.0 · `duty` 0.5 · `standup_s` 2.0 · `cmd_timeout` 0.5 · `force_trot` · `turn_arm` · `v_gain` · `x_max`.
- 실측(corridor 헤드리스 10 s, `stride_hz` 3.0·`turn_arm` 0.38): (v 0.3, w 0) → 0.267 m/s · (0, 0.5) → 0.502 rad/s · (0.5, 1.0) → 0.426 m/s·0.866 rad/s · 후진 (−0.2, 0) → −0.199 m/s, 기울기 최대 2.0°. 제자리 trot 기울기 0.8°·발 이격 3.4~3.6 cm. `/cmd_vel` 없으면 기립, 발행이 끊기면 기립으로 복귀.
- **맥 GUI는 일시정지로 시작한다**(로봇이 z 0.4 공중에 멈춰 있고 `trot_metrics.py`가 lift 0으로 FAIL) → 창 좌하단 ▶를 누르거나 `gz service -s /world/<world>/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: false'`. 헤드리스는 해당 없음. `gait_node`는 sim time 기준이라 ▶ 누른 시점부터 기립 램프가 시작된다. corridor는 벽이 로봇을 가리므로 Entity Tree에서 벽 우클릭 → View → Transparent.

## 테스트·실험

```bash
python -m pytest test/ -q -p no:launch_testing -p no:launch_ros
python3 scripts/patrol_metrics.py --world corridor   # full_system 떠 있는 상태에서 한 바퀴 측정
python3 scripts/trot_metrics.py --world corridor --expect-v 0.3 --expect-w 0   # legged.launch.py에 /cmd_vel 걸린 상태에서 추종 측정(인자 없으면 제자리 trot 기준)
```
