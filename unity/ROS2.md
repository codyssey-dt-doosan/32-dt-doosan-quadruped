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

## 구현 범위

ROS 양방향 통신, 속도 입력, 위치·TF·시간·카메라·라이다·합성 IMU, 기본 ROS 경유점 순찰을 구현했다. `patrol_ros.py`는 ROS에서 실행되는 단순 경유점 추종 baseline이며 기존 MPC 알고리즘을 사용하지 않는다. 기존 MPC를 이 Unity 맵에서 사용하려면 경로·센서 높이·지도 설정을 별도로 맞춰 검증해야 한다.

다리 관절 토크·접촉 힘에 의한 물리 보행, SLAM, 실제 열화상·가스 농도 계산은 이 연결 구현에 포함하지 않는다. Desktop용 ClientWebSocket을 사용하므로 WebGL 대상은 지원하지 않는다. 원본 영상 시연은 `Plant_VideoReference` 씬으로 계속 실행할 수 있다.
