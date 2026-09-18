# plume_sim

담당: **채현**

탱크 누출원의 농도장을 가상 플랜트에 올린다. 시간 평균 2D 가우시안 플룸(바람 방향으로 퍼짐) + 가스가 고이는 정체 구역(가짜 봉우리)을 `/odom` 위치에서 샘플링해 가스 센서·풍향계를 흉내 낸다.

```bash
ros2 launch plume_sim plume_sim.launch.py world:=factory      # config/plume_<world>.yaml
```

- 발행: `/gas/concentration` (Float32 ppm), `/gas/wind` (Vector3Stamped, 월드 좌표), `/gas/field` (OccupancyGrid, RViz), `/gas/markers` (실제 누출원·정체 구역)
- 모델: `plume_sim/plume.py` (`GasField`, ROS 비의존 — source_seeking 오프라인 시뮬레이터도 사용)
- 테스트: `python -m pytest test/ -q -p no:launch_testing -p no:launch_ros`

알고리즘·결과는 [../README.md](../README.md).
