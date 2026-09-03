# 설계 메모

## 역할 분리

```
Virtual Plant (Gazebo worlds)     Robot Twin (Go2 URDF + sensors)
        \                               /
         \                             /
          ---- ros_gz_bridge ---------
                       |
        +--------------+--------------+
        |              |              |
   locomotion     inspection      gas_safety
        |              |              |
        +--------------+--------------+
                       |
                 monitoring/web
```

- **Plant**: 복도·공장 SDF, 장애물, 게이지, 가스탱크, 플룸.
- **Twin**: Go2 기구학, IMU, LiDAR, RGB/열화상, 가스 센서 토픽.
- **Bridge**: `simulation/config/ros_gz_bridge.yaml` — clock, pose, cmd, 센서.
- **관제**: `monitoring/web`이 ROS 브리지(또는 REST)로 상태·알람을 표시.

## 월드

| 월드 | 크기 (L×W×H) | 용도 |
|------|----------------|------|
| `corridor` | 45 × 3.5 × 4 m | 좁은 통로 순찰, 전도 회복 |
| `factory` | 25 × 18 × 7 m | 게이지 점검, 탱크·플룸 |

## 토픽 (초안)

| 토픽 | 방향 | 용도 |
|------|------|------|
| `/clock` | GZ → ROS | 시뮬 시간 |
| `/cmd_vel` | ROS → GZ | 보행 속도 명령 |
| `/odom` | GZ → ROS | 트윈 오도메트리 |
| `/scan` `/points` | GZ → ROS | 지형·장애물 |
| `/camera/image` | GZ → ROS | 게이지 OCR |
| `/thermal/image` | GZ → ROS | 열화상 융합 |
| `/gas/concentration` | GZ → ROS | 소스 시킹 |
| `/inspection/gauge` | 내부 | 판독값·알람 |
| `/mission/status` | 내부 | 순찰·복귀 상태 |

## 통합 실행

`simulation/launch/full_system.launch.py`가 가제보, 브리지, 모듈 1·2·3 노드를 한 번에 올린다.
