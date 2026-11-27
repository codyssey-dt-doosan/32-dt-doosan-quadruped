# elevation_map

담당: **도훈**

`/points`(LiDAR PointCloud2, 센서 프레임)를 로봇 중심 XY 그리드로 빈닝해 셀별 최대 높이를 낸다.

```bash
ros2 launch elevation_map elevation_map.launch.py
```

## 출력 `/elevation_map` (std_msgs/Float32MultiArray)

- `layout.dim[0]`=y(행), `layout.dim[1]`=x(열). 기본 40×40, 셀 0.1 m, 로봇이 (20,20), 전방 = 열 증가 방향
- 값 = 셀 내 최대 높이(m), **0 = 지면**. 빈 셀 = NaN
- 셀 (r, c) 의 로봇 기준 좌표: x = (c − 20)·0.1, y = (r − 20)·0.1
- 라이다 하향 빔 한계로 바닥은 전방 약 1.0 m부터 채워짐. 그 안쪽은 NaN(자기 몸체 제외 `min_range` 0.5 m)

## 파라미터

| 이름 | 기본 | 뜻 |
|---|---|---|
| `resolution` | 0.1 | 셀 크기(m) |
| `size` | 4.0 | 그리드 한 변(m) |
| `sensor_height` | 0.56 | 라이다 지상고(m). 몸체 정착 z 0.36 + `model.sdf` 라이다 z 0.20 |
| `min_range` | 0.5 | 이 반경 안 포인트 무시(자기 몸체) |

몸체 roll/pitch 보정 없음. 다리 마찰 0으로 주행 중 몸체는 수평 유지됨.

## 통과 가능 판정 모드 `traversability:=true` (⑤-2, 옵트인)

```bash
ros2 launch simulation full_system.launch.py world:=unity_plant locomotion:=legged traversability:=true
```

**켜면 `/elevation_map` 값의 의미가 바뀐다 — 높이가 아니라 막힘 표시다.** `1.0` = 막힘, `0.0` = 통과 가능 지면, NaN = 미관측. 구독자는 mpc_controller 하나이고, 그대로 '값 > 0.15 = 막힘'으로 읽으므로 플래너는 고치지 않는다. 기본 `false`는 위 높이 출력 그대로다.

`/imu` 자세(roll·pitch)로 포인트를 중력 수평 프레임에 돌린 뒤 같은 방위 빔의 연속 반사점을 비교한다(`elevation_map/traversability.py`).
- 연속 두 점의 높이차가 `trav_step_tol + 거리·tan(trav_max_slope_deg)`를 넘으면 그 광선 구간을 막는다(턱 앞면, 낙차 그늘).
- 방위 첫 점은 발밑과 비교한다. 지면선을 발밑까지 연장한 높이와 몸체 기준 높이 중 덜 극단인 쪽이 ±`trav_max_step`을 넘으면 막는다. 그래서 경사로 들머리·위·끝 전이는 통과하고, 박스·턱·낙차는 막힌다.
- 가장 낮은 빔이 돌아오지 않은 방위(하강 계단·구멍)는 바닥이 있어야 할 곳부터 막는다.
- `/imu`가 없거나 0.5 s 넘게 낡으면 발행을 보류한다(mpc는 1 s 무수신에 정지). **`factory` 월드에는 IMU 시스템 플러그인이 없어 켜면 로봇이 서 있기만 한다** — corridor·unity_plant에서만 쓴다.

| 이름 | 기본 | 뜻 |
|---|---|---|
| `traversability` | false | 켜면 값 = 막힘 표시 |
| `trav_max_step` | 0.15 | 발밑과 첫 지면 사이 허용 턱·낙차(m) |
| `trav_max_slope_deg` | 13.0 | 연속 반사점 사이 허용 기울기 — 12° 경사로 통과, 25°는 막힘 |
| `trav_step_tol` | 0.03 | 연속 반사점 높이차 여유(m). 연속 구간 턱 한계는 사실상 높이 모드(0.15)보다 엄격 — 10 cm 둔덕도 막힌다(trot 발 들림 4 cm) |

- velocity 모드는 경사에서 몸체 pitch가 굳어(VelocityControl이 매 스텝 회전 속도를 덮어씀) 경사 뒤 평지에서 몸체가 뜬다 — 경사로 판정은 legged 기준으로 맞췄다.
- 실측(2026-11-27): unity_plant legged + mpc 1랩 3/3 완주(헤드리스 515.6·539.1 s, Unity Play 동시 sim 370 s·RTF 0.25·막힘 0), 전도 0, 경사로 이탈 0(헤드리스). 막힘은 웨이포인트 너머 벽 앞 도착 직전 1프레임(플래너 헤딩 스캔 1.5 m) — 기록 재생으로 판정 오판 아님 확인. 기본(끔) corridor·factory 회귀 없음 — REPORT §3.
- 검증: 합성 라이다 장면 23개(`test/test_traversability.py` — 직진 통과/막힘 16개는 기존 높이 규칙이 5개 실패, 그 밖에 옆 계단 경계·0.9 m 통로·중력 보정(pitch·roll)·사각 안 낙차·몸체 높이 어긋남). 회전·roll·첫 점 규칙·부유 보정을 하나씩 지운 변이를 모두 잡는다 + unity_plant 실측 기록 오프라인 재생(vault `DT-작업노트-2026-11-27-C안-경사-인지-구현`).
