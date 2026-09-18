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

- **Plume sim**: 시간 평균 2D 가우시안 플룸(바람 0.5 m/s 방향으로 퍼짐) + 가스가 고이는 정체 구역(가짜 봉우리)을 `/odom` 위치에서 샘플링해 가스 센서(`/gas/concentration`)·풍향계(`/gas/wind`)를 흉내 낸다. 공장은 gas_tank_1 밸브 (7.6, -6), 복도는 남쪽 벽 배관 (15, -1.6). 노이즈 비례 10% + 0.3 ppm.
- **Source seeking**: 상태 머신 SEARCH → TRACK → ESCAPE → SOURCE_FOUND. TRACK 방향 = 바람 반대 + 농도 구배(최근 6 s 평면 최소제곱, ±0.6 rad weave로 측면 구배 측정) + tabu 반발. **Local optimum 감지**: 30 s 동안 1.2 m 안에서 맴돎 / 40 s 개선 없음 / 강한 봉우리 지나쳐 농도 절반 이하. 감지하면 **Archimedean spiral**(`r = r0 + 2·θ/2π`, 최대 5 m)로 주변을 훑어, 기준(갇힌 지점 6 s 평균)보다 8% 높은 상태가 1.5 s 이어지면 탈출하고 멀리 있던 옛 봉우리는 tabu로 기록. 40 ppm 이상 봉우리 주변 2.5 m 나선에서 더 높은 곳이 없으면 누출원으로 판정. 출력은 `/source_seeking/goal`(mpc_controller용), 단독 시험은 `/cmd_vel`.
- **Return to home**: 배터리% ≤ 1.5 × 집까지 경로(m) × 소모율(%/m) + 10% 이면 복귀(예: 20 m → 25%). 경로는 지나온 지점(1 m 간격, 1.8 m 이내 연결) 그래프의 Dijkstra 최단 경로라 지도 없이도 걸어 본 길로만 돌아온다. 소모율은 사전값 0.5 %/m와 실측 중 큰 값. 그 밖에 최대 거리 50 m, 임무 1200 s, 누출원 발견 5 s 후, 탐색 영역 소진. 트윈에 배터리가 없어 `battery_sim`이 거리·시간 비례로 `/battery`를 낸다.
- 상세·그림: [module3_gas_safety/README.md](module3_gas_safety/README.md)

## 3. 결과

> 수치·로그는 실험 후 이 절에 채운다.

| 시나리오 | 지표 | 목표 | 실측 |
|----------|------|------|------|
| 복도 순찰 | 경로 완주율 | 100% | 1/1 완주, 시뮬 162 s |
| 보행 제어 비교 | corridor 한 바퀴: 완주 s / 평균\|w\| rad/s / 최소 장애물 거리 m / 계산 ms | heading_scan 166.8 / 0.046 / 0.26 / 0.2 · **mpc 167.7 / 0.052 / 0.28 / 0.9** | 둘 다 완주 |
| 보행 제어 비교 | factory 한 바퀴: 완주 s / 평균\|w\| rad/s / 최소 장애물 거리 m / 계산 ms | heading_scan 165.2 / 0.066 / 0.76 / 0.1 · **mpc 166.8 / 0.086 / 0.72 / 0.9** | 둘 다 완주 |
| 공장 점검 | 게이지 판독 오차 | TBD | — |
| 가스 탐색 | 누출원 판정 성공률 / 시간 / 위치 오차 (오프라인 2D, seed 20개) | factory hybrid **20/20** / 평균 132 s / 0.30–0.58 m · corridor hybrid **19/20** / 평균 234 s / 0.32–0.83 m · 구배만(비교) factory 2/20, corridor 3/20 (가짜 봉우리에 갇힘) | ROS 노드 + 운동학 로봇: factory 0.41 m, corridor 0.41 m. Gazebo 통합(goal mux) 전 |
| 안전 복귀 | 배터리 소진 전 홈 복귀율 | 80/80 (오프라인, 모든 모드·월드). 배터리 25% 시작 → 18.5%에서 복귀 판단(집까지 9.6 m), 13.3% 남기고 도착 | — |
| 전도 회복 | 회복 성공률 | 5/5 (corridor, roll 100° 강제 전도, 평균 회복 2.8 s) | — |

## 4. 관제 캡처

캡처 이미지는 `docs/captures/`에 두고 아래에 삽입한다.

### 4.1 복도 월드

![복도](docs/captures/corridor.png)

### 4.2 공장 월드

![공장](docs/captures/factory.png)

### 4.3 웹 관제

![관제](docs/captures/monitoring.png)
