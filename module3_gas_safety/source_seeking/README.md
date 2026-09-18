# source_seeking

담당: **채현**

농도 구배로 누출원을 추적한다. 탐색 → 추적(바람 반대 + 구배) → local optimum 감지 → Archimedean 나선 탈출(가짜 봉우리는 tabu) → 누출원 판정.

```bash
ros2 launch source_seeking source_seeking.launch.py world:=factory                    # goal 모드 (mpc_controller용)
ros2 launch source_seeking source_seeking.launch.py world:=factory drive_mode:=cmd_vel # 단독 주행
ros2 run source_seeking offline_sim --world factory [--mode gradient] [--battery 25]   # Gazebo 없이 2D 검증
```

- 구독: `/odom`, `/gas/concentration`, `/gas/wind`, `/return_to_home/status` (RETURNING이면 멈춤)
- 발행: `/source_seeking/goal`, `/source_seeking/state`, `/source_seeking/source`, `/source_seeking/markers`, (`cmd_vel` 모드) `/cmd_vel`
- 파라미터: `config/source_seeking.yaml`, 월드별 주행 영역·장애물 `config/world_<world>.yaml`
- 코어: `source_seeking/seeker.py` (`SeekerCore`, ROS 비의존)
- 테스트: `python -m pytest test/ -q -p no:launch_testing -p no:launch_ros` (공장·복도 시나리오 포함)

알고리즘·결과는 [../README.md](../README.md).
