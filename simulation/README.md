# Simulation — Virtual Plant + Robot Twin

Gazebo Harmonic 월드, Go2 트윈, `ros_gz` 브리지, 통합 런치.

```bash
ros2 launch simulation full_system.launch.py              # 복도
ros2 launch simulation full_system.launch.py world:=factory
ros2 launch simulation full_system.launch.py gui:=false
ros2 launch simulation full_system.launch.py locomotion:=legged   # 다리 구동 trot(도훈, C안 옵트인). 기본은 velocity(현행)
ros2 launch simulation full_system.launch.py world:=unity_plant locomotion:=legged traversability:=true inspection:=false gas_safety:=false   # Unity 발전소 배치(경사로 통과 판정 켬)
```

## 월드

| 월드 | 내용 |
|---|---|
| `corridor` | 복도 45 × 3.5 × 4 m, 장애물 2·게이지 1 |
| `factory` | 공장 25 × 18 × 7 m |
| `unity_plant` | Unity `Plant_GazeboMirror`의 충돌 배치(복도·경사로 12°·계단·공장, 원점 = Unity Spot 시작점). Unity 미러용 — `unity/ROS2.md` "Gazebo 미러" |

`unity_plant.sdf`는 생성물이라 손으로 고치지 않는다: `python3 simulation/scripts/unity_layout_to_sdf.py unity/config/gazebo_layout.json`(순찰 웨이포인트도 같이). 박스 1천여 개를 정적 모델 하나의 링크에 담는다 — 모델을 박스마다 두면 RTF가 0.3까지 떨어졌다. 경사 영역(5~25°, 직사각형 평면)은 기둥 대신 기울인 판 하나로 만든다(`find_ramps`) — 2 cm 반올림 기둥의 4 cm 턱에서 velocity 몸체가 들려 넘어졌다(2026-11-27). 게이지·가스탱크 모델은 아직 넣지 않았다. 생성기 테스트: `python -m pytest test/ -q -p no:launch_testing -p no:launch_ros`.
