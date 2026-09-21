# 실험 보고서

디지털 트윈 순찰·점검·가스 안전 시나리오의 알고리즘, 정량 결과, 관제 화면을 정리한다.

## 1. 개요

| 항목 | 내용 |
|------|------|
| 플랜트 | 복도 `45 × 3.5 × 4 m`, 공장 `25 × 18 × 7 m` |
| 로봇 트윈 | Unitree Go2 (12 DoF) |
| 미들웨어 | ROS 2 Jazzy + `ros_gz_bridge` |
| 시뮬레이터 | Gazebo Harmonic |

## 2. 알고리즘

### 2.1 모듈 1 — 보행 (`module1_locomotion`, 도훈)

- **Elevation map**: LiDAR/깊이 포인트로 지형 그리드를 갱신하고 발 착지 후보를 만든다.
- **MPC**: 샘플링 기반 운동학 MPC. 헤딩 스캔(elevation map 전방 1.5 m 통로를 goal 방향 ±60° 후보로 검사)이 준 자유 헤딩 방향 1.5 m 지점을 참조 goal로 삼고, 유니사이클 모델로 지평 2 s(10×0.2 s)를 예측한다. (v, w) 입력을 앞·뒤 2구간으로 나눠 729개 시퀀스를 롤아웃하고, elevation map 점유 셀을 발자국 반경 0.35 m로 팽창해 닿는 시퀀스를 버린 뒤 지평 평균 goal 거리 + 0.5·|종단 heading 오차| + 0.1·Σ|w|dt 가 최소인 시퀀스의 첫 명령을 10 Hz로 실행한다(receding horizon). 전부 막히면 goal 쪽 제자리 선회. 헤딩 스캔 단독은 `planner:=heading_scan`으로 남겨 비교했다. 관절 동역학·접지력 MPC는 트윈에 관절 구동이 없어 범위 밖.
- **Fall recovery**: IMU roll/pitch가 60°를 0.3 s 넘으면 `fallen`, 1.5 s 후 Gazebo `set_pose`로 제자리 기립(x·y·yaw 유지) → 자세 20° 이내 0.5 s 유지 시 `recovered`. 최대 3회 재시도. 회복 중엔 mpc_controller가 `/cmd_vel` 0.

### 2.2 모듈 2 — 점검 (`module2_inspection`, 운학 · 태우)

- **Gauge OCR** (운학): 아날로그/디지털 게이지 ROI를 잡고 지침·숫자를 읽어 알람 임계와 비교한다.
- **Thermal fusion** (운학): RGB와 열화상을 정렬·융합해 과열 영역을 표시한다.
- **Patrol path** (태우): 복도/공장 웨이포인트와 게이지·탱크 점검 정차점을 순회한다.

### 2.3 모듈 3 — 가스 안전 (`module3_gas_safety`, 채현)

- **Plume sim**: 탱크 누출원의 농도장을 가상 플랜트에 올린다.
- **Source seeking**: 농도 구배로 누출원을 추적한다.
- **Return to home**: 알람/배터리/임무 종료 시 홈 포즈로 복귀한다.

## 3. 결과

> 수치·로그는 실험 후 이 절에 채운다.

| 시나리오 | 지표 | 목표 | 실측 |
|----------|------|------|------|
| 복도 순찰 | 경로 완주율 | 100% | 1/1 완주, 시뮬 162 s |
| 보행 제어 비교 | corridor 한 바퀴: 완주 s / 평균\|w\| rad/s / 최소 장애물 거리 m / 계산 ms | heading_scan 166.8 / 0.046 / 0.26 / 0.2 · **mpc 167.7 / 0.052 / 0.28 / 0.9** | 둘 다 완주 |
| 보행 제어 비교 | factory 한 바퀴: 완주 s / 평균\|w\| rad/s / 최소 장애물 거리 m / 계산 ms | heading_scan 167.3 / 0.071 / 0.71 / 0.2 · **mpc 173.2 / 0.090 / 0.35 / 0.7** | 둘 다 완주, 막힘 0 (2026-09-21 재측정 — 원판 팽창 이후 mpc가 장애물에 더 붙어 지남) |
| 공장 점검 | 게이지 판독 오차 | TBD | — |
| 가스 탐색 | 누출원 도달 시간 | TBD | — |
| 전도 회복 | 회복 성공률 | 5/5 (corridor, roll 100° 강제 전도, 평균 회복 2.8 s) | — |

## 4. 관제 캡처

캡처 이미지는 `docs/captures/`에 두고 아래에 삽입한다.

### 4.1 복도 월드

![복도](docs/captures/corridor.png)

### 4.2 공장 월드

![공장](docs/captures/factory.png)

### 4.3 웹 관제

![관제](docs/captures/monitoring.png)
