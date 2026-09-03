# 32-dt-doosan-quadruped

두산 사족보행 로봇 디지털 트윈. Gazebo Harmonic 가상 플랜트 위에서 Go2 트윈이 복도·공장 순찰, 게이지/열화상 점검, 가스 누출 탐색을 수행한다.

| 모듈 | 담당 | 내용 |
|------|------|------|
| `module1_locomotion` | 도훈 | Elevation map, MPC, 전도 회복 |
| `module2_inspection` | 운학 · 태우 | 게이지 OCR, 열화상 융합, 순찰 경로 |
| `module3_gas_safety` | 채현 | 플룸 시뮬, 누출원 탐색, 복귀 |
| `monitoring` | 수현 | 관제 웹 대시보드 |

## 환경

- **OS**: Ubuntu 24.04 (Noble)
- **ROS**: ROS 2 Jazzy
- **시뮬레이터**: Gazebo Harmonic (`gz-sim`)
- **브리지**: `ros_gz_bridge`

로컬 패키지 의존성:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  gz-harmonic \
  ros-jazzy-ros-gz \
  ros-jazzy-xacro \
  python3-colcon-common-extensions \
  python3-opencv
```

## 빌드

워크스페이스 루트(이 저장소)에서:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 실행

통합 런치(가제보 + 브리지 + 3모듈):

```bash
# 복도 45 × 3.5 × 4 m (기본)
ros2 launch simulation full_system.launch.py

# 공장 25 × 18 × 7 m
ros2 launch simulation full_system.launch.py world:=factory
```

관제 대시보드:

```bash
cd monitoring/web
python3 -m http.server 8080
# 브라우저: http://localhost:8080
```

## Docker

```bash
docker build -t doosan-quadruped .
docker run --rm -it --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  doosan-quadruped
```

헤드리스:

```bash
docker run --rm -it doosan-quadruped \
  bash -lc "ros2 launch simulation full_system.launch.py gui:=false"
```

## 디렉터리

```
├── docs/                     # 설계 메모, 발표 자료
├── simulation/               # Virtual Plant + Robot Twin
│   ├── worlds/               # 복도 45×3.5×4, 공장 25×18×7 SDF
│   ├── models/               # Go2 URDF, 장애물, 게이지, 가스탱크
│   ├── launch/               # 통합 launch (가제보 + 브리지 + 3모듈)
│   └── config/               # ros_gz_bridge.yaml
├── module1_locomotion/       # 도훈
│   ├── elevation_map/
│   ├── mpc_controller/
│   └── fall_recovery/
├── module2_inspection/       # 운학(게이지·열화상) + 태우(순찰)
│   ├── gauge_ocr/
│   ├── thermal_fusion/
│   └── patrol_path/
├── module3_gas_safety/       # 채현
│   ├── plume_sim/
│   ├── source_seeking/
│   └── return_to_home/
└── monitoring/               # 수현 관제
    └── web/
```

알고리즘·실험 결과·관제 캡처는 [REPORT.md](REPORT.md)를 본다.
