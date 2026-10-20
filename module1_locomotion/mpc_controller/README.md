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

CoM·접지력 MPC(C안)로 가는 첫 단계. **기본 구동과 별개**: `full_system.launch.py`는 그대로 VelocityControl + mpc이고, 이 런치는 다리로 서는 go2만 띄우고 `/cmd_vel` (v, w)를 다리로 따라간다. 전체 스택과 같이 띄우려면 `full_system.launch.py locomotion:=legged` — 월드·모델만 변환본으로 바뀌고 elevation_map·mpc·patrol·fall_recovery는 수정 없이 그대로 돈다(corridor 195.0 s·factory 197.6 s 완주·막힘 0, 기본 구동 대비 약 +15%. 강제 전도 회복 2.4 s). 기본값 `velocity`는 현행 그대로.

```bash
ros2 launch mpc_controller legged.launch.py                        # 기립, /cmd_vel (v, w)를 trot으로 추종
ros2 launch mpc_controller legged.launch.py gui:=false trot:=true  # 헤드리스, /cmd_vel 없이도 제자리 trot
ros2 launch simulation full_system.launch.py locomotion:=legged    # 전체 스택을 다리 구동으로(순찰)
```

- 모델은 복사본이 아니라 **런치 시 `simulation/models/go2/model.sdf`를 변환**(`legged_model.py`): VelocityControl 제거, 관절별 `JointPositionController` 12개, 다리 마찰 `mu`, 관절 스프링 제거. 원본 구조가 바뀌면 `legged 변환 실패[…]`로 런치가 멈춘다 → 메시지의 단계를 보고 `legged_model.py`를 맞출 것.
- 다중 `<joint_name>` 컨트롤러는 첫 관절만 피드백한다(묶으면 전도) → 관절마다 하나.
- `gait_node`: 곧은 다리 → 명목 자세(`d_nominal` 0.25) 2 s 램프 후 대각 쌍(FL·RR / FR·RL) trot. `/cmd_vel` (v, w)를 0.3 s로 평활해, 지지 발을 뒤로 쓸어 전진하고 좌우 보폭 차이로 회전한다. **개루프**(균형·속도 피드백 없음) — 남는 추종 오차는 위층이 odom으로 닫는다. 토픽 `/legged/<joint>`(Float64) × 12.
- 보정 손잡이: `turn_arm` 0.38(회전 — 기하 0.14로는 앞뒤 발 횡마찰에 42%만 돈다. 2 Hz에선 0.33, 3 Hz에선 0.38이 맞았다), `v_gain` 1.0(전진), `x_max` 0.10(반보폭 한계). `mu`·`p_gain`·`stride_hz`가 바뀌면 다시 맞출 것.
- 런치 인자: `world` · `gui` · `trot` · `mu` 0.8 · `p_gain` 120 · `d_gain` 2.0. 노드 파라미터: `d_nominal` · `lift` · `stride_hz` 3.0 · `duty` 0.5 · `standup_s` 2.0 · `cmd_timeout` 0.5 · `force_trot` · `turn_arm` · `v_gain` · `x_max`. · `step_anchor` false(밟은 자리 기록 착지 보정·경사 추종 발 기억 — 옵트인 실험, 이득 근거 없음)
- walk status 진단 필드: `late`(지지 시작 30 ms 후 센서 미접촉 횟수) · `anchor_hit`(기록 높이 사용 비율, step_anchor 켰을 때) · `land_err_mm`(센서 착지 순간 실제−목표 높이 med/p10/p90/n). `walk_metrics --slope` 출력의 `y_drift`는 코스 안 진입 y 대비 최대 횡 이탈(기준 3·5), `y_max`는 정보용(캡처 전체 절대 y).
- 실측(corridor 헤드리스 10 s, `stride_hz` 3.0·`turn_arm` 0.38): (v 0.3, w 0) → 0.267 m/s · (0, 0.5) → 0.502 rad/s · (0.5, 1.0) → 0.426 m/s·0.866 rad/s · 후진 (−0.2, 0) → −0.199 m/s, 기울기 최대 2.0°. 제자리 trot 기울기 0.8°·발 이격 3.4~3.6 cm. `/cmd_vel` 없으면 기립, 발행이 끊기면 기립으로 복귀.
- **맥 GUI는 일시정지로 시작한다**(로봇이 z 0.4 공중에 멈춰 있고 `trot_metrics.py`가 lift 0으로 FAIL) → 창 좌하단 ▶를 누르거나 `gz service -s /world/<world>/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: false'`. 헤드리스는 해당 없음. `gait_node`는 sim time 기준이라 ▶ 누른 시점부터 기립 램프가 시작된다. corridor는 벽이 로봇을 가리므로 Entity Tree에서 벽 우클릭 → View → Transparent.

## 힘 제어 기립 (`balance.launch.py`, 옵트인)

CoM·접지력 QP(C안 ①~③: 상태 입력·다리 기구학·힘 제어 기립)로 서는 것. `legged.launch.py`(위치 PD·개루프 trot)와는 별개 런치이고, 지평 MPC(SRB, trot·경사 대응, ④)는 아래 `walk.launch.py`. `full_system` 통합 안 함.

```bash
ros2 launch mpc_controller balance.launch.py                       # 평지 QP 주도 기립
ros2 launch mpc_controller balance.launch.py gui:=false             # 헤드리스
ros2 launch mpc_controller balance.launch.py ramp_deg:=15           # 15° 경사판 위 기립
ros2 launch mpc_controller balance.launch.py qp:=false               # QP 끄고 관절 PD만(기동 게인 고정, 스파이크 기준선)
ros2 launch mpc_controller balance.launch.py pitch_offset:=0.0873    # 자세 목표 오프셋(rad)
```

- 의존성 `osqp`는 `package.xml`에 선언하지 않음(옵트인) — `conda install -n ros_env -c conda-forge osqp`. 없으면 import 시 설치법을 담은 오류로 종료.
- 프레임 규약(전부 스펙 §3.0과 동일):

| 기호 | 프레임 | 의미 |
|---|---|---|
| q, q̇ | 관절 | `/joint_states` 12개 |
| `foot_pos(leg, q)` | B | 발 **구 중심**(calf 원점 기준), 몸체 원점 기준 |
| J | B | 구 중심 속도 = J·q̇ |
| R = R_GB | B→G | Rz(−ψ)·R_WB(IMU 자세, yaw 제거) |
| n, t₁, t₂ | G | 지형 법선·마찰콘 접선 기저 |
| 접촉점 p_i | G | R·`foot_pos` − r·n (구 중심 − 반지름×법선) |
| f_i | G | 지면이 발에 주는 힘 |
| τ | 관절 | −Jᵀ(Rᵀf) + PD |

- 파라미터(보정 손잡이): `mass` 13.2 · `inertia` diag(0.22, 0.48, 0.58) · `com_offset` (0, 0, −0.04) · `height` **0.295**(법선 방향 목표 CoM 높이 = `Q_NOM` (0, 0.407, −1.2) 자세값) · `com_shift` 1.0 · `kp_pos` (50,50,100)·`kd_pos` (5,5,10) · `kp_rot` (100,100,50)·`kd_rot` (5,5,5) · `mu` 0.5 · `f_min` 2 N·`f_max` 120 N · 가중치 diag(1,1,2, 10,10,5) · `alpha` 1e-3·`beta` 1e-2 · `start_delay` 3 s.
- 기동 순서(첫 `/joint_states` 이후 sim 시간 기준): 0~2 s 관절 PD 램프(Kp 120·Kd 2, q를 곧은 다리→`q_nom`으로 선형 램프) → 2~3 s smoothstep 블렌딩(Kp 120→0·Kd 2→1로, QP는 t≥2부터 계산) → t≥3 QP 주도(Kp 0·Kd 1, 감쇠만 — 관절 배치는 접지 발·몸체 자세가 결정).
- hold(안전) 후퇴: QP 실패가 50 ms 넘게 지속, 기울기 > 45°, 또는 IMU·관절 콜백이 0.1 s 이상 끊기면 `hold` 모드로 전환 — Kp 120 PD로 `q_nom` 고정(래치, 불연속 허용). 추정→QP→τ_ff 경로는 통째로 try/except로 감싸 예외 시 `f=None`으로 처리(경고 로그, throttle 1 s) — nan·솔버 예외가 executor를 죽이지 않고 기존 supervisor 경로로 hold에 들어간다.
- 실측 명령: `log/balance_one.sh <태그> <metrics 인자...> -- [launch 인자...]`(로컬 스크립트, gitignore 대상 — 레포에 없음; `-- ramp_deg:=15` 등 런치 인자 전달). 내부에서 `scripts/balance_metrics.py --world <w> --log log/balance_<태그>.log`를 호출하며, 지원 플래그는 `--duration`(기본 15) · `--push N S`(2 s 시점에 +y N을 S s 인가 후 clear) · `--expect-pitch D`(경사 판정, 법선/접선 힘 합 밴드 포함). 노드가 1 Hz로 남기는 status 로그는 `mode`·`qp_ms`(평균/p99)·`fail`·`sat`·`sum_fz`·`sum_fn`·`sum_ft`·`calf_min`(가장 굽은 calf 관절 각, rad — 하드스톱은 −1.57)·`tilt`를 담는다.

실측(corridor 헤드리스):

| 기준 | 시험 | 결과 | 판정 |
|---|---|---|---|
| 1 | 평지 기립 | z_mean 0.337·z_std 0.00 mm·기울기 0.10°·Σf_z 126.7 N(mg 129.4, −2%)·QP 0.08 ms 평균/0.11 ms p99·fail 0·sat 0·calf_min −1.18(한계 여유 0.39 rad) | PASS |
| 2 | +y 20 N × 0.5 s 밀기 | 최대 기울기 2.22°·xy 이탈 3.4 cm·클리어 후 복귀 <1 s·발 미끄러짐 0.3 cm | PASS |
| 3 | 경사 15° 기립 | pitch −15.48°(오차 0.5°)·미끄러짐 0 cm·Σnᵀf 121.4 N(필요 124.9, −2.8%)·\|Σtᵀf\| 34.5 N(필요 33.5, +3%)·calf_min −1.05 | PASS |
| 4 | 경사 위 `pitch_offset` ±5° | +5°(코 내림) → −10.30°(목표 −10)·sat 0·calf_min −1.18; −5° → −20.53°(목표 −20)·sat 0 | PASS |
| 5 | QP·토크 시간 | 평균 0.08 ms·p99 0.1 ms | PASS |
| 6 | 회귀 | pytest 107 전부 통과, `walk_one.sh 0.3 0` → 0.268 m/s(HEAD와 동률), `legged_model` 기본 출력 HEAD와 문자열 동일 | PASS |

**미해결 이슈(백로그):** 없음(기준 1~6 전부 PASS). 함정만 둘 — ① `gz topic -p`는 부하 중 간헐적으로 미전달(rc 0)이라 `--push` 반응이 0이면 재실행; **persistent 렌치는 발행마다 누적**되니 2회 발행하면 안 됨(40 N이면 전도). ② IMU는 초기 자세 기준이라 스폰이 수평이어야 함.

**해결 이력(2026-09-22):** ① "토크→힘 14% 손실"·무릎 하드스톱의 원인은 SDF 관절 `<damping>1.0` — gz-sim 8.6/DART가 정지 상태에도 calf에 ≈1.05 N·m를 먹어 τ=−Jᵀf가 30% 부족했다(접촉 센서로 발 반력 31.2 N = mg/4 정상 확인) → `balance.launch.py`는 변환본 관절 damping 0(`make_legged_model(damping=0.0)`, 감쇠는 노드 Kd_j). `legged.launch.py`·trot(gz 위치 PD)는 원본 1.0 유지 → [[DT-작업노트-2026-09-22-C안-무릎-하드스톱-스파이크]]. ② 접촉 자코비안에 −r·n 항(`leg_kinematics.contact_jacobian`) — 경사 접선력 명령 40.0 → 34.5 N. ③ 기립 자세 `Q_NOM` calf −1.35 → −1.2(thigh 0.407, CoM 높이 0.295) — pitch +5° 명령의 포화 해소.

**맥 노트:** `gz sim -s -r`가 아니라 일시정지로 시작하고, 노드가 `start_delay`(3 s) 뒤 `gz service`로 재개한다. GUI는 `legged.launch.py`와 같이 서버·창 분리 기동. IMU orientation은 **초기 자세 기준**(월드 기준 아님) — 스폰이 수평(roll=pitch=0)이어야 하며 현재 corridor의 go2 스폰은 이를 만족한다.

## 지평 MPC 보행 (`walk.launch.py`, 옵트인)

C안 ④: `/cmd_vel` (v, w)를 **SRB 지평 MPC**(Cheetah-3 볼록 MPC, 13상태·condensed, 발 힘 12×N, N 10·dt 0.03 s, osqp) + trot으로 실현하는 `walk_node`. `balance.launch.py`와 같은 변환본(힘 제어·구형 발·damping 0)을 쓰고, `balance_node`·`gait_node`·공용 런치는 불변. `full_system` 통합·경사 보행은 ⑤.

```bash
ros2 launch mpc_controller walk.launch.py                    # 기립(지평 MPC) → /cmd_vel 오면 trot
ros2 launch mpc_controller walk.launch.py gui:=false          # 헤드리스
ros2 launch mpc_controller walk.launch.py slope_deg:=15       # 스폰 앞 2 m 오르막·평지·내리막(15° 완주, 20°는 불안정 — 아래 미해결 1)
ros2 launch mpc_controller legged.launch.py slope_deg:=15     # 대조군: 개루프 gait_node 같은 경사
```

- 구조: `srb_mpc.py`(SRB 동역학·이산화·condensing·QP) · `gait.py`(위상·스케줄·Raibert 착지·스윙 궤적) · `body_estimate.FootMemory`(trot 2발 접촉에서도 최근 착지점 4개로 지형 평면) · `walk_node.py`(200 Hz 추정·50 Hz MPC·1 kHz 관절 토크). 지지 발 τ = −Jᵀf(접촉 자코비안), 스윙 발은 IK + 관절 PD.
- 모드: `standup`(0~2 s PD 램프) → `blend` → `stand`(MPC, 4발) → `walk`(명령 있을 때 gait 활성, 끊기면 정지 전이) → `hold`(QP 실패 50 ms·기울기 45°·IMU/관절 0.1 s·IK 실패 20틱, 래치).
- 런치 인자: `world` · `gui` · `mu` 0.8 · `slope_deg`·`slope_len` 3.0 · `ramp_deg` · 튜닝용 `stride_hz` 3.0 · `lift` 0.025 · `k_raibert` 0.03 · `v_gain` 1.12 · `q_weights`·`q_weights_walk`(13개 쉼표) · `alpha` · `height`·`height_walk` · `contact_source` sensor(기본)|kinematic · `contact_sensors`(kinematic 판정에서 센서만 붙여 계측) · `touch_h` 0.008(kinematic 판정 높이) · `swing_knee_min` 0.7(스윙 목표 무릎 최소 굽힘 rad) · `real_time_factor` 1.0(<1이면 시뮬을 느리게 — 실측용, `walk_metrics --rtf`를 같은 값으로).
- 실측 확정 튜닝(스펙 기본값에서 바뀐 것): `height`·`height_walk` 0.30(0.32는 서 있으면 다리 도달 93%라 좌굴하고, 보행 중엔 몸체가 7 mm 더 떠 착지가 17 ms 늦음) · `lift` 0.025 · `step_max` 0.08 · `v_gain` 1.12(MPC 추종이 89%로 일관) · Q 정지 p_xy 20·v_xy 0.5(밀기 반응), 보행 v_xy 8·p_z 50 · 속도 추정 = `/odom` twist(v_z는 정지 시 다리 오도메트리 저역통과, 보행 시 0) · 지형 법선 저역통과 0.3 s · 스윙은 s 0.85에 착지 후 1 cm 밀기 · 조기 착지만 판정(기본은 발 contact 센서 — 운동학 판정은 발 높이 추정이 10 ms에 4 cm 튀는 것에 속아 공중 발에 힘을 줌) · MPC k=0 접촉 = 실제, 접촉 변화 시 즉시 MPC.

실측(corridor 헤드리스, `log/walk_run.sh <태그> walk_metrics.py <인자> -- [런치 인자]`, 로컬 스크립트):

| 기준 | 시험 | 결과 | 판정 |
|---|---|---|---|
| 1 | 평지 (v,w) 15 s | (0.3,0) → 0.314 m/s · (0,0.5) → 0.476 rad/s · (0.5,1.0) → 0.508/0.963 · (−0.2,0) → −0.201(후진은 8 s — 15 s면 서쪽 벽에 닿아 −0.14), 기울기 ≤2.4°, z_std <1 cm. 2026-10-04 기본값(센서·높이 0.30·무릎 여유 0.7): 0.291 · 0.476 · 0.484/0.964 · −0.206, 기울기 ≤0.9°, 포화 ≤31 | PASS(오차 <5%) |
| — | 기립 40 s · 밀기 +y 20 N×0.5 s | z_std 0.00 mm·기울기 0.05° · 밀기 기울기 1.1°·이탈 2.7 cm(balance 2.2°/3.4 cm) | PASS |
| 2 | 위층 `/cmd_vel` 재생 | 60 s 전체는 개루프 재생이 corridor 장애물에 부딪힘 → 20 s 창(`--replay-window 40 20`, 직진→회전→복합, 녹화 v×0.8). 센서 판정·높이 0.30에서 10회 전도 0, rms_v 0.013·rms_w 0.006·종점 오차 4%, 포화 ≤50은 7/10. 무릎 여유 0.7 추가 후 5회 중 4회 PASS(rms_v 0.008·종점 1.7%·포화 ≤32), 1회 전도(미해결 6) | PASS 4/5 |
| 3·4·5 | 경사 15°·20° (RTF 0.5) | 15°: 완주 3/3(유효 3회), 내리막 최대 기울기 17.6·20.2·17.2°, 횡 드리프트 0.30·1.00·0.11 m. 20°: 완주 1/2, 오르막 0.29 m/s, 내리막 26.8°. 입구 전도는 10-04 평지 수정 후 재현 안 됨 | 15° 4·5 대체로 PASS, 20° 3 FAIL |
| 6 | 계산 | MPC 평균 1.6~2.1 ms·p99 ≈2 ms(GUI 부하 7 ms) | PASS |
| 7 | 회귀 | pytest 146, balance 기립 동률(z_std 0.01 mm·drift 0.1 cm), `legged.launch.py` 불변 | PASS |

- **기준 2 지표(2026-09-28 재정의)**: `walk_metrics --replay`는 보폭 창(1/stride_hz = 0.33 s) 평균 속도로 RMS를 잰다. 명령이 바뀐 뒤 0.5 s 안에 걸친 창은 가속 구간이라 제외(`n_win` = 채점 창 수). 틱별 포즈 미분은 trot 흔들림이 그대로 들어가 v 0.3 일정 명령에서도 rms_v 0.071이 나와 0.05 기준에 원리적으로 못 미쳤다.

**미해결(백로그):**
1. **경사 20°·실측 신뢰도**: 입구 전도는 10-04 수정(센서 접지·높이 0.30·무릎 여유·큐 깊이 1)으로 재현 안 됨. 20°는 완주해도 내리막 기울기 25~27°·횡 드리프트 0.4~0.8 m, 전도도 남음. 실측이 저부하에서도 IMU 끊김(0.7~2.8 s)·QP p99 수십~수백 ms로 흔들려 원인 분리 불가 → 토크 루프 분리(⑤-3) 후 재평가. ⑤-1 `step_anchor`(밟은 자리 기록)는 옵트인, 이득 근거 없음 — vault `DT-작업노트-2026-10-12-C안-경사-보행-구현`.
2. **보행 전도율**: 2026-10-01 원인 규명 — 보행 높이 0.32에서 착지가 스케줄보다 17 ms 늦고(발 contact 센서 실측, status `td_ms`), 운동학 조기 착지 판정이 공중 발(0~8 mm)이나 발 높이 추정 튐을 착지로 받아 힘을 배정. 높이 0.30 + 센서 판정으로 재생 10/10 무전도(이전 기본 구성 3/7·1/5 전도). 머신 부하(MPC p99 >5 ms)에선 여전히 전도 가능.
3. **기동 레이스**: 드물게 기동 직후 `qp 0·active 0`으로 hold(재기동하면 정상).
5. (해소, 2026-10-04) **센서 판정의 펴진 다리 고착** — 늦은 발이 '1 cm 누르기' 목표를 도달 한계까지 따라가 다리가 펴진 채 착지, 몸체가 3.6 cm 높게 고착되고 MPC Σf_z ≈ mg/2였음. 스윙 목표 무릎 최소 굽힘 `swing_knee_min` 0.7 rad(정상 높이에선 스윙 목표가 −0.90까지라 무영향)로 몸체 0.378 → 0.360, Σf_z 65 → 96 N, 착지 지연 16 → 3 ms. 아직 목표보다 1.8 cm 높고 Σf_z < mg — 남은 받침 하중은 미해결.
6. **스윙 발 튐**(2026-10-04 원인 규명): 목표 변화 없이 스윙 발이 5 ms에 7~9 mm 오르내림 = 스윙 관절 PD 진동(PD 요구 토크 100+ N·m). ① `/joint_states` 구독 깊이 10이라 MPC 풀이 동안 쌓인 메시지를 하나씩 처리해 낡은 상태(4~6 ms)로 PD → 깊이 1로 해소(관절 지연 p99 3 → 1 ms). ② 부하 시 MPC와 1 kHz 토크 루프가 Python 한 스레드에서 같이 굶어 토크 갱신이 수십 ms 끊김(바쁜 루프 3개: 콜백 간격 p90 17.6 ms·최대 41 ms, 재생 3/3 전도). **실측은 `real_time_factor:=0.5`로**: 같은 부하에서 3/3 무전도·튐 0~5회. 무부하에서도 가끔 노드가 수십 ms 멈춰(콜백 간격 최대 82 ms 벽시계) 비틀거림(재생 1/3). 근본 대응은 토크 루프 분리(별도 스레드·프로세스) — ⑤/실기 이식 때.
4. (해소) "후진 회귀"는 측정 조건 탓 — 15 s 후진은 스폰에서 서쪽 벽(약 2.2 m)에 닿는다. 8 s로 재면 −0.206·−0.208(2026-10-01, 센서 판정·높이 0.30).

**함정:** 맥이 잠들면 IMU가 끊겨 hold → 배치는 깨어 있을 때만. GUI 카메라는 `gz service -s /gui/move_to --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --req 'data: "go2"'` 반복만 로봇을 잡는다(follow는 벽).

## 테스트·실험

```bash
python -m pytest test/ -q -p no:launch_testing -p no:launch_ros
python3 scripts/patrol_metrics.py --world corridor   # full_system 떠 있는 상태에서 한 바퀴 측정
python3 scripts/trot_metrics.py --world corridor --expect-v 0.3 --expect-w 0   # legged.launch.py에 /cmd_vel 걸린 상태에서 추종 측정(인자 없으면 제자리 trot 기준)
python3 scripts/walk_metrics.py --world corridor --log log/walk_<태그>.log --cmd 0.3 0 15   # walk.launch.py 떠 있는 상태에서 (v,w) 추종(--replay F.csv [--replay-window S D], --slope DEG LEN)
```
