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
