# module2 점검(게이지·열화상) 인터페이스 설계

- 작성: 운학 · 2026-10-15 · 상태: **초안 (팀 검토 전)**
- 범위: `gauge_ocr`, `thermal_fusion`, 공용 패키지 `inspection_common`
- 방침: 재사용 가능한 핵심 기능과 토픽만 만든다. 우선순위·알람 기준·정차 방향 같은 **통합 조건은 팀 회의에서 정한다** (7절).

## 1. 목표와 전체 흐름

과제 흐름(REPORT.md 2절, Project_Definition_Canvas): 순찰 정차 → 게이지 판독·열화상 감시 → 가스 이상 시 누출원 탐색 → 복귀.

운학 파트가 내보내는 것:

| 구분 | 내용 | 누가 쓰나 |
|------|------|------|
| 게이지 점검 위치 | 게이지를 화면에 담을 수 있는 정차 위치·방향 | 순찰(태우)이 웨이포인트로 채택 |
| 게이지 값 | `/inspection/gauge` (기존) | 관제(수현) |
| 과열 지점 goal | 처리할 과열 지점이 있을 때 다가갈 위치 | 이동(도훈)이 goal 우선순위에 추가 |
| 과열 알람 | `/inspection/thermal_alert` (기존) | 관제(수현) |
| 상태 문자열 | 노드별 상태 토픽 | 관제·통합 |

KPI: 게이지 MAE·성공률, 열화상 Recall/F1.

## 2. 현재 확인된 제약 (설계 근거)

| # | 사실 | 영향 |
|---|------|------|
| 1 | 게이지 높이 1.5 m(factory)/1.4 m(corridor), 카메라는 base_link 기준 (0.28, 0, 0.05), 세로 반화각 32.2°, fx≈381.5 | 가까이 서면 게이지가 화면 위로 벗어남 |
| 2 | 게이지 모델에 바늘이 없음. 해가 거의 위에서 비춰 원판이 어두움 | 실제 판독은 시뮬 수정 전까지 불가 |
| 3 | 열화상 센서는 `type="camera"` L8 흑백 영상(밝기). Thermal 플러그인 없음 | 햇빛 받은 바닥·벽이 "뜨겁게" 나옴 → 오탐 |
| 4 | `mpc_controller`는 goal의 yaw를 무시함 | 정차 시 보는 방향이 51~90° 어긋남 |
| 5 | goal 비활성 규칙: `frame_id` 빈 `PoseStamped` 또는 timeout | 새 goal 토픽도 이 규칙을 따른다 |
| 6 | 순찰 상태 문자열 `goto:` / `dwell:` / `waiting:odom` | 게이지 이름 추정에 읽기 전용으로 사용 |

## 3. 패키지 구조

맵 읽기·카메라 계산·점검 위치 계산·장애물 검사는 게이지와 열화상이 같이 쓰므로 공용 패키지로 분리한다.

```
module2_inspection/
  inspection_common/          # 새 ament_python 패키지. 모듈은 ROS 없이 import 가능
    inspection_common/
      world_map.py            # SDF 읽기: 게이지 목록, 장애물, 카메라·열화상 사양
      geometry.py             # 카메라 모델, 픽셀→방향, 2D raycast, 로봇 몸체 충돌 검사
      viewpoint.py            # 게이지 점검 위치, 과열 지점 점검 위치
      state_msg.py            # 상태 문자열 만들기·읽기
    test/
  gauge_ocr/                  # 기존 reader/overlay/node + 점검 위치 도구·토픽, 상태 토픽
    gauge_ocr/make_viewpoints.py
  thermal_fusion/
    thermal_fusion/
      thermal.py              # 픽셀 → 온도 (thermal_mode)
      hotspot.py              # 과열 영역(블롭) 검출
      locate.py               # 픽셀 → 맵 위치 (raycast, 바닥 제외)
      tracker.py              # 같은 지점 묶기, 상태 전이
      align.py, overlay.py    # RGB 정렬·겹쳐 그리기
      thermal_fusion_node.py
  docs/inspection_interface_design.md   # 이 문서
```

### 3-1. 맵 파일 찾기 (순환 의존 회피)

`simulation/package.xml`이 이미 `gauge_ocr`, `thermal_fusion`을 `exec_depend`로 갖고 있다. 반대 방향 의존을 package.xml에 넣으면 순환이 되므로 **넣지 않는다.**

- 런타임에 `get_package_share_directory("simulation")`으로 `worlds/<world>.sdf`, `models/`를 찾는다 (ament 인덱스 조회는 선언 없이 가능).
- 파라미터 `world_file`, `models_path`가 주어지면 그것을 우선한다 (단독 실행·테스트용).
- CLI는 `--sim-share <경로>`로 저장소 `simulation/`을 직접 가리킬 수 있다 (ROS 없이 Mac에서 실행).

## 4. 게이지 설계

### 4-1. 입력 (SDF에서 파싱)

- 월드의 `<include><uri>model://gauge</uri>` → 이름, pose(x, y, z, yaw)
- `models/gauge/model.sdf` → 원판 반지름 0.12 m, 앞면 오프셋 0.04 m
- `models/go2/model.sdf`의 camera 센서 → 장착 위치, pitch, hfov, 해상도
- 벽·장애물 충돌 박스, include 모델(가스탱크 등)의 충돌 모양

파라미터 (YAML, 기본값):

| 이름 | 기본 | 뜻 |
|------|------|------|
| `body_height` | 보행 모드로 자동 (velocity 약 0.4 m, legged 0.295 m) | base_link 높이 |
| `margin_deg` | 5.0 | 화면 위 가장자리 여유각 |
| `arrive_tol` | 0.4 | 순찰 도착 오차 (이만큼 더 가까이 서도 성립해야 함) |
| `robot_radius` | 0.35 | 장애물 검사용 몸체 반경 |
| `min_dial_px` | 20 | 멀리 섰을 때 원판 **지름**(px)이 이보다 작으면 "판독 불가" 사유 (합성 영상 실측: 16 px 이상 오차 0.2 bar 이내, 12 px 실패) |
| `dwell_s` | 5.0 | 순찰 YAML에 적을 정차 시간 |

### 4-2. 계산

1. 게이지 정면 방향 직선 위에서, 원판 윗끝이 `세로 반화각 − margin_deg` 안에 들어오는 **가장 가까운 거리**를 구한다. 카메라가 base_link보다 0.28 m 앞에 있는 것, `arrive_tol`만큼 더 가까워져도 성립해야 하는 것을 반영한다.
2. 그 위치에서 로봇 몸체(`robot_radius`)가 장애물과 겹치지 않는지, 카메라→원판 사이가 막히지 않는지 검사한다.
3. 불가능하면 위치 대신 사유(`blocked`, `too_small`, `no_space`)를 낸다.
4. yaw는 게이지를 정면으로 보는 방향.

현재 맵 계산값 (구현 후 테스트로 다시 확인):

거리 = 원판 앞면에서 base_link까지 수평 거리. 앞 숫자는 velocity, 뒤 숫자는 legged.

| 맵/조건 | 거리 | 원판 지름 px (정위치 / 0.4 m 멀리) | 비고 |
|------|------|------|------|
| factory 1.5 m | 2.96 / 3.16 m | 32→28 / 30→26 | |
| corridor 1.4 m (velocity) | 2.76 m | 34→30 | 가용 공간 2.91 m로 빠듯 |
| corridor 1.4 m (legged) | 2.97 m | — | 공간 부족 → `no_space` |
| 안 A (게이지 0.7 m) | 1.40 / 1.61 m | 80→59 / 67→52 | 시뮬 수정 필요 |

> 정정: 이전 대화에서 "16/15 px", "40/33 px"라고 한 값은 **반지름**이었다. 지름으로는 위 표와 같다. 합성 영상 실측으로 지름 16 px 이상은 오차 0.2 bar 이내로 읽혔으므로 크기 자체는 판독 가능 범위다. 남은 문제는 바늘이 없고 원판이 어둡다는 것(2절 2번).

### 4-3. 출력

| 형태 | 이름 | 내용 |
|------|------|------|
| CLI | `ros2 run gauge_ocr make_viewpoints --world factory` (또는 `python -m gauge_ocr.make_viewpoints --sim-share ...`) | 순찰 YAML 형식(`name, x, y, yaw, dwell_s`) + 주석으로 원판 px·사유 |
| 토픽 | `/inspection/gauge_viewpoints` | `PoseArray`, frame `map`, latched |
| 함수 | `inspection_common.viewpoint.gauge_viewpoint(...)` | 위 계산 |
| 토픽 (기존 유지) | `/inspection/gauge` | `Float32`, 0.5 s 중앙값, 판독값 없으면 발행 안 함 |
| 토픽 | `/inspection/gauge_state` | 상태 문자열 (6절) |

판독 중인 게이지 이름: `/mission/status`의 `dwell:<이름>`을 먼저 쓰고, 없으면 현재 위치에서 1 m 이내 가장 가까운 점검 위치의 게이지, 그것도 없으면 `unknown`.

## 5. 열화상 설계

### 5-1. 문제

밝기 센서라 오탐이 나고, 과열 지점의 거리를 모르며, 같은 지점을 매 프레임 새로 감지하고, "점검 완료" 기준이 없다. RGB 정렬은 depth를 고정값으로 쓰고 있다.

### 5-2. 처리 단계

1. **온도 변환** `thermal_mode`
   - `l16`: 16비트 값 × 0.01 K → °C (진짜 열화상 센서로 바꾼 경우)
   - `l8_thermal`: 8비트를 `t_min`~`t_max` 선형 변환
   - `brightness` (현재 시뮬 기본): 밝기를 그대로 점수로
2. **검출**: 기존 STUDY 방식(임계값 + 최소 면적). 깜빡임 방지는 프레임 단위 히스테리시스 대신 **지점 단위**로 한다: 같은 지점이 `confirm_frames`(3)번 관측돼야 후보 → 대기로 올리고, `forget_s`(3 s) 동안 안 보이는 후보는 지운다
3. **위치 추정**: 영역 중심 픽셀 → 수평 각도 → `/odom` 위치·yaw + 카메라 오프셋 → 맵 장애물에 2D raycast로 맞는 첫 표면이 과열 지점. 픽셀이 수평선 아래이고 바닥에 먼저 닿으면 **바닥으로 보고 버린다** (햇빛 받은 바닥 오탐 제거, 화면 아래를 통째로 가리는 마스크 대신). 위치 함수는 하나로 분리해 LiDAR(`/points`) 방식으로 교체할 수 있게 한다
4. **묶기**: 기존 지점과 `merge_radius`(1.0 m) 이내면 같은 지점으로 보고 최고 온도·관측 횟수만 갱신
5. **상태 전이** (지점마다)

```
CANDIDATE --(confirm_frames 충족)--> PENDING --(goal 0.4 m 안 도착)--> INSPECTING
INSPECTING --(5 s 정차 후 최고 온도 ≥ 임계)--> CONFIRMED
INSPECTING --(5 s 정차 후 미달 / 안 보임)-----> FALSE_ALARM
PENDING --(goal 후 inspect_timeout 초과, 2번째)---> UNREACHABLE
CONFIRMED, FALSE_ALARM, UNREACHABLE --(recheck_s 경과, 기본 0 = 안 함)--> 다시 감지 가능
```

6. **점검 위치**: 과열 지점에서 처음 관측한 쪽으로 `approach_dist`(1.5 m) 떨어진 곳. 도착 오차·몸체 장애물 검사는 게이지와 같은 `geometry` 함수를 쓴다. yaw는 과열 지점 정면. 위치를 못 구하면 `PENDING` 대신 사유를 남기고 goal을 내지 않는다.
7. **정렬·겹쳐 그리기**: 고정 depth 대신 raycast로 구한 실제 거리를 쓴다.

### 5-3. 출력

| 토픽 | 타입 | 규칙 |
|------|------|------|
| `/inspection/thermal_alert` (기존) | `Bool` | 0.5 s 주기 그대로. **CONFIRMED 지점이 있으면 True** (점검 위치에서 확정된 과열만) |
| `/inspection/thermal_goal` | `PoseStamped` | `PENDING`/`INSPECTING` 지점이 있는 동안 2 Hz, 없으면 빈 `frame_id` |
| `/inspection/thermal_hotspots` | `PoseArray` | 모든 지점 위치, latched |
| `/inspection/thermal_overlay` | `Image` | 구독자 있을 때만 |
| `/inspection/thermal_state` | `String` | 상태 문자열 (6절) |

여러 지점이 `PENDING`이면 현재 위치에서 가장 가까운 것부터 처리한다.

## 6. 공통 규칙

### 6-1. 상태 문자열

채현 님 형식(`"상태 | 이유 | 값"`)에 맞춰 `" | "`로 칸을 나누고 첫 칸은 대문자 상태.

| 토픽 | 예시 | 상태 값 |
|------|------|------|
| `/inspection/gauge_state` | `READING \| gauge_line1 \| 4.23 bar \| conf 0.62` | `IDLE`, `READING`, `NO_GAUGE` |
| `/inspection/thermal_state` | `PENDING \| hotspot_1 (7.65, -6.00) \| 85.2 C \| goal (6.1, -6.0)` | `IDLE`, `CANDIDATE`, `PENDING`, `INSPECTING`, `CONFIRMED`, `FALSE_ALARM`, `UNREACHABLE` |

`state_msg.py`의 `format_state(state, *fields)` / `parse_state(text)`를 관제·통합 쪽도 쓸 수 있다. **`/mission/status`에는 발행하지 않는다** (복귀 트리거와 섞이지 않게).

### 6-2. 토픽 규칙

| 항목 | 규칙 | 근거 |
|------|------|------|
| 좌표계 | `frame_id = "map"` (= Gazebo 월드 좌표) | 순찰과 같음. `/odom`도 월드 좌표 |
| goal 비활성 | 빈 `frame_id` | `mpc_controller` 규칙 |
| goal 주기 | 활성 동안 2 Hz | `mpc_controller` timeout |
| 목록형 토픽 | `TRANSIENT_LOCAL`, depth 1 | 늦게 켠 노드도 받음 |
| 영상 토픽 | `qos_profile_sensor_data` | 기존 방침 |
| 기존 토픽 | `/inspection/gauge`, `/inspection/thermal_alert` 이름·타입·주기 유지 | 대시보드가 구독 중 |

### 6-3. 설정

- 맵별 파라미터: `config/<패키지>_<world>.yaml` (현행 유지).
- `full_system`이 각 모듈 런치에 넘기는 `locomotion` 인자를 두 런치가 받아서 `body_height`를 정한다.
- 카메라·열화상 사양, 게이지·장애물 위치는 SDF에서 읽고 YAML에 중복으로 적지 않는다.

### 6-4. 오류 처리

| 상황 | 동작 |
|------|------|
| SDF 못 찾음 / 파싱 실패 | 경고 로그. 게이지는 판독을 계속하고 상태 마지막 칸에 `no_map`. 열화상은 위치를 못 구하므로 지점·goal·알람 없이 `IDLE \| no_map` |
| `/odom` 끊김 (1 s) | 위치 추정·goal·도착 판정 중단(빈 goal), 상태 `IDLE \| no odom`, 경고 로그 1회. 오래된 후보 정리는 계속 |
| 카메라 영상 크기가 SDF 설정과 다름 | 겹쳐 그리기 전에 SDF 크기로 맞춤 |
| 영상 끊김 | 상태 `IDLE \| no image`, 알람은 마지막 확정 상태 유지 |
| 점검 위치 불가 | goal을 내지 않고 상태에 사유 |
| 대기 지점에 도착 실패 (`inspect_timeout` 60 s) | 1번째는 `PENDING` 유지·사유 `unreachable`로 재시도, 2번째는 `UNREACHABLE`로 끝냄 (알람 아님) |

## 7. 다른 팀원 작업에 미치는 영향

**기본 원칙: 다른 사람의 코드·설정 파일은 고치지 않는다.** 새 토픽은 아무도 구독하지 않으면 동작에 영향이 없다.

| 담당 | 지금 영향 | 채택하면 필요한 일 (회의 안건) |
|------|------|------|
| 수현 (관제) | 없음. `/inspection/gauge`, `/inspection/thermal_alert` 이름·타입·주기 유지. 다만 thermal_alert가 지금 스텁(항상 False)에서 실제 값으로 바뀜 | 상태 토픽 표시 여부, 판독 실패 표시, 알람 기준 |
| 태우 (순찰) | 없음. `/mission/status`는 읽기만 함 | `make_viewpoints` 결과를 웨이포인트 YAML에 반영할지. factory 게이지 정차점이 게이지에서 약 3 m로 바뀜. corridor legged는 불가 |
| 도훈 (이동) | 없음. `goal_topics` 기본값에 thermal_goal이 없으므로 무시됨 | `/inspection/thermal_goal`을 우선순위에 넣을지(제안: 복귀 > 누출원 탐색 > 과열 점검 > 순찰). 정차 시 goal yaw 맞추기 — 없으면 점검 위치를 정해도 화면에 안 잡힘 |
| 채현 (가스) | 없음. `/mission/status`에 쓰지 않으므로 복귀 트리거와 무관 | 가스탱크 정차점 위치((8,−5)은 너무 가까움, (6.0,−6) yaw 0 제안), 과열 시 복귀 여부 |
| 시뮬 모델·월드 | 없음. SDF는 읽기만 함. package.xml 순환 의존도 만들지 않음 | 게이지 높이 0.7 m(안 A)·바늘 추가, 열화상 센서를 L16 thermal로 교체. 월드 구조(include 방식)가 바뀌면 `world_map` 테스트가 깨져서 알려줌 |
| 빌드·Docker | `inspection_common` 패키지가 하나 늘어남. 저장소 Dockerfile은 `colcon build` 전체 빌드라 별도 수정 없음 | Dockerfile에 `cv_bridge` 누락은 기존 문제 (arm64화와 함께 안건) |
| 공용 문서 | `README.md` 트리, `docs/architecture.md` 토픽 표는 손대지 않음 | 확정되면 각 담당과 같이 갱신 |

## 8. 테스트 계획 (ROS 없이 Mac에서 도는 것 위주)

| 대상 | 확인 내용 |
|------|------|
| `world_map` | 저장소 실제 맵에서 factory 게이지 2개, corridor 1개, 가스탱크·벽을 찾는지 |
| `viewpoint` (게이지) | 4-2 표의 값(factory 2.96 m 등)이 ±0.05 m로 나오는지, corridor legged가 `no_space`인지 |
| `geometry` | 탱크 쪽 raycast가 탱크 표면에 닿는지, 벽에 붙은 위치를 충돌로 잡는지 |
| `reader` 추가 | 원판 지름 16~32 px 합성 영상 오차 0.2 bar 이내, 12 px은 판독 실패(NaN)로 한계 고정 |
| `tracker` | 반복 감지가 한 지점으로 묶이는지, 확정 후 다시 생기지 않는지, 도착 실패 재시도 1회 |
| `hotspot`, `align` | STUDY 5.6절 검증(정렬 오차, 블롭 검출)을 정식 테스트로 |
| `state_msg` | 만들고 다시 읽으면 같은 값인지 |

노드 단위 실행 확인은 ROS 환경이 준비된 뒤에 한다.

## 9. 진행 순서

1. `inspection_common` + 테스트
2. 게이지: `make_viewpoints`, 점검 위치 토픽, 상태 토픽, 작은 원판 테스트
3. 열화상: 온도 변환 → 검출 → 위치 추정 → 묶기·상태 → goal·알람 → 정렬·겹쳐 그리기
4. `module2_inspection/README.md`에 인터페이스 표 정리

## 10. 범위 밖

- 다른 모듈 코드 수정 (goal 우선순위, yaw 정렬, 순찰 웨이포인트, 대시보드 표시)
- 시뮬 모델 수정 (게이지 높이·바늘, 열화상 센서 교체)
- 결과 저장 형식 (회의 후 결정)
