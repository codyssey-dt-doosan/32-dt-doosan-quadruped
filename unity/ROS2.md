# ROS 2 ↔ Unity 연결

게이지 판독·합성 열화상·가스 노출 경보를 포함한 통합 순찰은 별도 `Plant_Inspection` 씬과 [INSPECTION.md](INSPECTION.md)를 사용한다. 아래는 기본 이동·센서 연결 씬 설명이다.

Unity 6 데스크톱 Editor/Standalone의 `Plant_ROS2` 씬과 ROS 2 Jazzy를 rosbridge WebSocket으로 연결한다. Unity 내부 순찰을 끈 ROS 모드에서 ROS의 `/cmd_vel`만 수평 이동·회전을 결정한다. Unity는 CharacterController 충돌과 중력, 제공 보행 애니메이션을 처리한다.

## 실행

저장소 루트에서 ROS 통신 서버를 시작한다.

```bash
docker compose -f unity/compose.yaml up -d --build
```

원본 과제 에셋이 있는 Unity 프로젝트에 이 저장소 `unity/`의 `Scripts`, `Editor`, `Scenes`, `DoosanReference`, `object`를 `.meta`와 함께 `Assets` 아래로 복사한다. 현재 작업한 Unity 프로젝트에는 이미 적용했다. `Assets/Scenes/Plant_ROS2.unity`를 열고 **Play**를 누른다. 연결 주소는 `ws://127.0.0.1:9090`이다.

다른 터미널에서 ROS 순찰 노드를 실행하면 로봇이 움직인다.

```bash
docker compose -f unity/compose.yaml exec rosbridge bash -lc \
  'source /opt/ros/jazzy/setup.bash && python3 /workspace/unity-tools/patrol_ros.py'
```

이 노드는 `config/ros_route.json`의 경유점을 `/odom`으로 추적해 `/cmd_vel`을 보낸다. 기본 속도는 0.5 m/s이며 `--speed 0.3`처럼 바꿀 수 있다. Unity의 씬을 다시 Play해 시뮬레이션 시간이 처음으로 돌아가면 순찰도 첫 경유점부터 시작한다. 노드 종료는 `Ctrl+C`, Unity 쪽 일시정지는 `Space`다. 서버 종료는 다음과 같다.

```bash
docker compose -f unity/compose.yaml down
```

수동 명령을 시험하려면 순찰 노드를 먼저 종료한다. 동시에 두 노드가 `/cmd_vel`을 발행하면 명령이 섞인다.

```bash
docker compose -f unity/compose.yaml exec rosbridge bash -lc \
  'source /opt/ros/jazzy/setup.bash && ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"'
```

명령 수신이 0.5초 끊기거나 연결이 끊기면 수평 이동·회전을 멈춘다. 재접속 시 이전 명령은 폐기하고 새 명령을 기다린다. 서버 포트는 Mac/PC의 localhost에만 공개하며, Gazebo는 이 구성에서 실행하지 않는다.

## 데이터

| 토픽 | 형식 | 기본 발행 주기 / 방향 |
|---|---|---|
| `/cmd_vel` | geometry_msgs/msg/Twist | ROS → Unity, linear.x(m/s), angular.z(rad/s) |
| `/odom` | nav_msgs/msg/Odometry | Unity → ROS, 최대 20 Hz |
| `/clock` | rosgraph_msgs/msg/Clock | Unity 시뮬레이션 시간, 최대 20 Hz |
| `/tf` | tf2_msgs/msg/TFMessage | 최대 20 Hz |
| `/imu` | sensor_msgs/msg/Imu | 자세·이동에서 계산한 합성 IMU, 최대 20 Hz |
| `/camera/image` | sensor_msgs/msg/Image | RGB8, 320×240, 최대 5 Hz |
| `/camera/camera_info` | sensor_msgs/msg/CameraInfo | 실제 렌즈의 내부 파라미터, 최대 5 Hz |
| `/points` | sensor_msgs/msg/PointCloud2 | 256×16 Raycast, 최대 5 Hz, 범위 20 m |

발행 주기는 Unity 렌더링·프레임 속도와 통신 처리량에 따라 낮아질 수 있다. 카메라는 로봇 앞쪽의 실제 RenderTexture를 읽고, 라이다는 레이어 8의 실제 맵 충돌 형상에 Raycast한다. 로봇은 레이어 9에 있어 자기 몸을 측정하지 않는다. 미검출 포인트는 NaN, 바이트 배열은 rosbridge base64 형식으로 보낸다. Unity API는 메인 스레드에서만 사용하며 소켓 송수신은 별도 작업으로 처리한다. 느린 연결에서는 토픽별 최신 샘플만 남겨 센서 큐가 계속 쌓이지 않게 한다.

좌표 기준은 ROS의 전방 x / 좌측 y / 위쪽 z다. 원점은 저장된 로봇 시작 위치, 전방은 시작 방향이다. 이 씬의 `base_link`는 CharacterController 루트(발끝에 가까운 높이)다. `odom`의 초기 높이는 정착 과정에서 약간 변할 수 있다. Unity의 월드 좌표를 기존 Gazebo 월드 경유점에 그대로 대입하지 말고, 이 씬에서 내보낸 ROS 경로를 사용한다.

TF는 `map → odom → base_link`와 `base_link → lidar / camera_link / imu_link`, `camera_link → camera_optical_frame`이다. 현재 map→odom은 항등 변환이다. 센서·위치는 동일한 Unity 시간으로 찍는다. ROS 노드에서 이 시간을 이용하려면 `use_sim_time:=true`를 설정한다.

## 검증과 재생성

Unity의 **Doosan → ROS 2 → Check Message Contracts**는 좌표·회전·시간·JSON·바이너리 메시지 형식을 검사한다. 실제 이동·센서를 검증할 때는 순찰 노드를 종료한 뒤 Unity를 Play하고 다음을 실행한다.

```bash
docker compose -f unity/compose.yaml exec rosbridge bash -lc \
  'source /opt/ros/jazzy/setup.bash && python3 /workspace/unity-tools/verify_ros_bridge.py'
```

이 검사는 짧게 전진·회전·후진 명령을 보내므로 주변 공간을 확보한 기본 시작 위치에서 실행한다. 결과는 컨테이너의 `/tmp/unity-ros-verification.json`에 저장한다. `observe_patrol.py`는 명령을 보내지 않고 ROS 순찰의 첫 경사로 통과를 관측한다.

씬 재생성은 **Doosan → ROS 2 → Create Connected Scene**을 사용한다. 영상 시연 씬을 복사해 ROS 씬과 경로 JSON을 만든다. 직접 수정한 ROS 씬은 재생성 전에 다른 이름으로 저장한다. 내보낸 `Assets/DoosanReference/ros_route.json`은 저장소 `unity/config/ros_route.json`에도 복사해야 ROS 순찰에 반영된다.

## Gazebo 미러 (`Plant_GazeboMirror`)

물리·제어는 Gazebo + ROS 노드가, 렌더링은 Unity가 맡는 구성이다. Gazebo 월드 `unity_plant`는 이 Unity 발전소의 충돌 배치를 그대로 옮긴 것이라(원점 = Unity Spot 시작점, x = 시작 방향) 두 쪽 맵이 같다. Unity는 `/odom`을 받아 로봇을 같은 위치·자세에 놓고, `/odom`이 2D라 높이는 Unity 바닥(layer 8)에 붙인다. 다리는 실제 관절이 아니라 속도에 맞춘 애니메이션이다.

```bash
bash unity/tools/start_gazebo_mirror.sh                 # 기본 MODE=legged WORLD=unity_plant
TRAV=false bash unity/tools/start_gazebo_mirror.sh      # 경사 판정 끔(높이 모드 — 경사로 입구에서 멈춤)
MODE=walk RTF=0.5 bash unity/tools/start_gazebo_mirror.sh   # SRB MPC + joint_servo (순찰·인지 없이 /cmd_vel만)
```

Gazebo(헤드리스)·`full_system`·네이티브 브리지(`ws://127.0.0.1:9090`, MJPEG `127.0.0.1:8090`)·대시보드(`http://127.0.0.1:8080/?world=unity`)를 함께 띄운다. 모든 포트는 127.0.0.1에만 열린다. Unity에서 `Plant_GazeboMirror`를 열고 Play하면 붙는다. 종료는 스크립트에 Ctrl+C(SIGTERM이면 노드까지 정리).

- 미러는 `WORLD=unity_plant`에서만 맵이 맞는다. corridor·factory는 스폰이 원점이 아니라 Unity 맵 밖에 놓인다(스크립트가 경고).
- 이 구성은 순찰만 돌린다(`inspection:=false gas_safety:=false`). Unity 카메라·열화상은 대시보드 영상으로만 가고 ROS 점검 노드로는 들어가지 않는다.
- **경사로 통과(⑤-2)**: 스크립트는 `WORLD=unity_plant`에서 elevation_map `traversability`를 켠다(`/elevation_map` 값이 높이 대신 막힘 표시, 끄려면 `TRAV=false`). legged + mpc 1랩 3/3 완주(헤드리스 2회 515.6·539.1 s·경사로 이탈 0, Unity Play 동시 1회 sim 370 s·RTF 0.25·막힘 0, 전도 0, 2026-11-27). 웨이포인트 바로 뒤에 벽·기계가 있는 곳(Return_Aisle·Gas_Leak_Approach)에서는 도착 직전 한 프레임 '경로 막힘'이 난다 — 헤딩 스캔 1.5 m가 웨이포인트 너머 벽을 본다. velocity 모드는 경사 뒤 몸체 pitch가 굳어 떠오르므로 경사로 주행에 맞지 않는다.
- 한 머신에서 Unity HDRP Play를 같이 돌리면 Gazebo RTF가 0.70~0.85에서 0.1 안팎으로 떨어진다(MacBook Air, Unity CPU 약 160%). walk 모드는 이 부하에서 RTF 0.5 기동 중 전도, RTF 0.25에서는 서서 걸었다.

### Docker (2대 분리 시연·팀원 재현성)

위 스크립트를 컨테이너 하나에서 그대로 돌린다(Gazebo·ROS 노드·브리지·대시보드). 저장소 루트에서:

```bash
docker compose up --build             # 기본 MODE=legged WORLD=unity_plant, 첫 빌드 MacBook Air M4 약 4분
MODE=walk RTF=0.5 docker compose up   # 인자는 스크립트와 같다(MODE·WORLD·RTF·TRAV·SLOPE_DEG)
docker compose exec sim bash          # 컨테이너 안 셸(ros2·gz 명령)
docker compose down
```

- 이미지는 노트북마다 직접 빌드한다. 베이스가 멀티 아키텍처 `ros:jazzy-ros-base`라 맥(arm64)·Windows(amd64) 모두 네이티브로 돈다.
- 호스트 포트는 `127.0.0.1:9090·8090·8080`에만 열린다. `unity/compose.yaml`(rosbridge)과 포트가 같아 둘 중 하나만 띄운다.
- 컨테이너에는 GPU가 없어 LiDAR·카메라·열화상을 CPU로 렌더링한다. MacBook Air M4(Docker 5 CPU·8 GB) 실측 RTF 0.44(네이티브 헤드리스 0.70~0.85), 같은 맥에서 Unity Play를 같이 돌리면 0.085(네이티브 0.112, 2026-11-28). 그래서 Unity는 다른 노트북에서 돌린다.

**다른 노트북의 Unity를 붙이기** — 포트를 LAN에 열지 않고 SSH 터널로 같은 로컬 포트를 끌어온다. Gazebo 노트북에서 SSH 서버를 켠다(맥: 시스템 설정 → 일반 → 공유 → 원격 로그인, Windows: 설정 → 시스템 → 선택적 기능 → OpenSSH 서버). Unity 노트북에서:

```bash
ssh -N -L 9090:127.0.0.1:9090 -L 8090:127.0.0.1:8090 -L 8080:127.0.0.1:8080 <사용자>@<Gazebo 노트북 IP>
```

터널을 켜 둔 채 `Plant_GazeboMirror`를 Play한다. Unity(`ws://127.0.0.1:9090`)·대시보드(`http://127.0.0.1:8080/?world=unity`)는 주소를 바꾸지 않아도 된다.

**월드 재생성** — Unity 배치를 바꿨을 때: Play를 끈 상태에서 **Doosan → ROS 2 → Export Gazebo Layout**(Unity 프로젝트 `GazeboLayout/unity_plant_layout.json`) → 저장소 `unity/config/gazebo_layout.json`으로 복사 → `python3 simulation/scripts/unity_layout_to_sdf.py unity/config/gazebo_layout.json`. `simulation/worlds/unity_plant.sdf`와 `patrol_path/config/waypoints_unity_plant.yaml`(`config/inspection_route.json`에서)이 다시 생긴다. 0.1 m 격자로 바닥 높이(계단·적재물)는 높이 기둥, 경사로(5~25° 직사각형 평면)는 기울인 판 하나, 바닥 + 0.3~2.3 m를 막는 것은 벽 박스로 만든다. 경사로 그레이팅 구멍은 양쪽 바닥으로 메운다.

## 구현 범위

ROS 양방향 통신, 속도 입력, 위치·TF·시간·카메라·라이다·합성 IMU, 기본 ROS 경유점 순찰을 구현했다. `patrol_ros.py`는 ROS에서 실행되는 단순 경유점 추종 baseline이며 기존 MPC 알고리즘을 사용하지 않는다. 기존 MPC를 이 Unity 맵에서 사용하려면 경로·센서 높이·지도 설정을 별도로 맞춰 검증해야 한다.

다리 관절 토크·접촉 힘에 의한 물리 보행, SLAM, 실제 열화상·가스 농도 계산은 이 연결 구현에 포함하지 않는다. Desktop용 ClientWebSocket을 사용하므로 WebGL 대상은 지원하지 않는다. 원본 영상 시연은 `Plant_VideoReference` 씬으로 계속 실행할 수 있다.
