# 관제 (수현)

보너스 웹 대시보드. 로봇 트윈 상태, 순찰 미션, 게이지·열화상 점검, 가스 농도를 한 화면에 둔다.

```bash
cd monitoring/web
python3 -m http.server 8080
```

브라우저에서 `http://localhost:8080` 을 연다.

ROS 2 토픽을 붙이려면 별도 터미널에서 `rosbridge_websocket` 을 띄운다.

```bash
sudo apt install ros-jazzy-rosbridge-suite
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

## 3D 트윈 (Three.js + roslib.js)

대시보드 상단 패널은 Three.js 로 그린 가상 플랜트(복도/공장)와 Go2 트윈이다. `roslib.js` 로 rosbridge 에 붙어 아래 토픽을 받는다.

| 토픽 | 타입 | 3D 표시 |
|------|------|---------|
| `/odom` | `nav_msgs/msg/Odometry` | 로봇 위치·yaw, 궤적, 속도 비례 보행 애니메이션 |
| `/mission/status` | `std_msgs/msg/String` | 점검 중이면 근처 게이지 강조 |
| `/patrol/goal` | `geometry_msgs/msg/PoseStamped` | 초록 핀 |
| `/return_to_home/goal` | `geometry_msgs/msg/PoseStamped` | 노랑 핀 |
| `/source_seeking/goal` | `geometry_msgs/msg/PoseStamped` | 빨강 핀 |
| `/inspection/thermal_alert` | `std_msgs/msg/Bool` | 로봇 몸체 빨강 발광 |
| `/gas/concentration` | `std_msgs/msg/Float32` | 로봇 주변 링 색·크기 |

- 주소 기본값은 `ws://<페이지 호스트>:9090`. 헤더 입력창이나 `?ws=ws://192.168.0.10:9090` 으로 바꾼다.
- `?world=factory` 로 시작 월드를 지정한다.
- rosbridge 가 없으면 웨이포인트를 도는 목업 데이터로 동작한다.
- Three.js / roslib.js 는 jsDelivr CDN 에서 받으므로 인터넷 연결이 필요하다.

## 로봇 카메라

3D 트윈 옆 패널에 로봇 시점 영상을 띄운다. 영상은 rosbridge 가 아니라 `web_video_server`(MJPEG, 기본 포트 8090)로 받는다.

| 화면 | 토픽 |
|------|------|
| 전방 | `/camera/image` (640×480) |
| 전방 + 판독 오버레이 버튼 | `/inspection/gauge_debug` |
| 열화상 | `/thermal/image` (320×240) |

- 주소는 헤더의 "영상" 입력창이나 `?video=http://192.168.0.10:8090` 으로 바꾼다.
- 스트림이 끊기면 NO SIGNAL 을 띄우고 3초마다 다시 붙는다.

Docker(Windows)에서 시뮬레이션 + rosbridge + 영상 서버를 함께 띄우기:

```bash
docker run --rm -it -p 127.0.0.1:9090:9090 -p 127.0.0.1:8090:8090 doosan-quadruped bash -lc "ros2 launch rosbridge_server rosbridge_websocket_launch.xml & ros2 run web_video_server web_video_server --ros-args -p port:=8090 & ros2 launch simulation full_system.launch.py locomotion:=legged gui:=false world:=factory"
```
