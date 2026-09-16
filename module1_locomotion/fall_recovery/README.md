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

## 테스트

```bash
python -m pytest test/ -q -p no:launch_testing -p no:launch_ros
python3 scripts/fall_trial.py --trials 5   # 헤드리스 full_system 떠 있는 상태에서
```
