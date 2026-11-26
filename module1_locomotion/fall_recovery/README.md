# fall_recovery

담당: **도훈**

`/imu` roll/pitch로 전도를 감지하고 Gazebo `set_pose`로 제자리 기립시킨다. 트윈은 다리 관절 제어가 없어 물리적 기립 대신 포즈 리셋(텔레포트)이 회복 수단이다.

```bash
ros2 launch fall_recovery fall_recovery.launch.py world:=corridor
```

## 토픽

| 토픽 | 방향 | 타입 | 내용 |
|------|------|------|------|
| `/imu` | 구독 | sensor_msgs/Imu | 자세 |
| `/odom` | 구독 | nav_msgs/Odometry | 리셋 위치(x, y, yaw 유지) |
| `/fall_recovery/status` | 발행 | std_msgs/String | 2 Hz + 전이 즉시. `idle` · `fallen` · `recovering` · `recovered` · `failed` |

관제: `fallen`이면 `spot_fall` 재생, `recovered`면 `spot_move` 복귀. `mpc_controller`는 `fallen`·`recovering`·`failed` 동안 `/cmd_vel` 0.

## 파라미터

`fall_deg` 60 · `upright_deg` 20 · `debounce` 0.3 s · `recover_delay` 1.5 s(애니메이션 시간) · `max_retries` 3 · `stand_z` 0.4 · `world`

기립 높이: `/odom`·`/tf`가 2D라 리셋 때 `gz model -m go2 -p`로 누운 몸체 z를 읽는다(약 2.6 s). 몸체가 `stand_z`+0.05보다 높으면 바닥이 올라온 곳(경사로·플랫폼)으로 보고 `몸체 z − 0.1 + stand_z`로, 아니면 기존처럼 `stand_z`로 세운다. `unity_plant` 플랫폼(바닥 0.875 m) 강제 전도에서 recovered, 몸체 z 1.216 m(2026-11-27, 1회). `fall_trial.py`는 z 0.5로 눕혀 평지 월드 전용이다.

## 테스트

```bash
python -m pytest test/ -q -p no:launch_testing -p no:launch_ros
python3 scripts/fall_trial.py --trials 5   # 헤드리스 full_system 떠 있는 상태에서
```
