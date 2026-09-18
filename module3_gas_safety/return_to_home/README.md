# return_to_home

담당: **채현**

알람/배터리/임무 종료 시 홈 포즈로 복귀한다.

```
복귀 기준(%) = 1.5 × 집까지 경로(m) × 소모율(%/m) + 10     배터리 ≤ 기준 → 복귀
```

집까지 경로는 지나온 지점 그래프(breadcrumb)의 Dijkstra 최단 경로. 그 밖에 최대 거리, 임무 시간, 위험 농도, 누출원 발견, 탐색 영역 소진, `/mission/status`의 `return_home` 요청.

```bash
ros2 launch return_to_home return_to_home.launch.py world:=factory              # battery:=30.0, drive_mode:=cmd_vel
ros2 param set /battery_sim percent 20.0                                        # 주행 중 배터리 강제 설정
```

- `battery_sim_node`: 트윈에 배터리가 없어 이동 거리(0.5 %/m)·시간(0.02 %/s)으로 `/battery` 발행
- `return_to_home_node`: 구독 `/odom`, `/battery`, `/gas/concentration`, `/source_seeking/state`, `/mission/status` → 발행 `/return_to_home/goal`, `/return_to_home/status`, `/mission/status`, (`cmd_vel` 모드) `/cmd_vel`
- 코어: `home_path.py` (`VisitedGraph`, `ReturnManager`), `battery.py`, `drive.py` (막힘 감지·cmd_vel 조향) — ROS 비의존
- 테스트: `python -m pytest test/ -q -p no:launch_testing -p no:launch_ros`

알고리즘·결과는 [../README.md](../README.md).
