# gauge_ocr 학습 내용 정리 (운학)

[STUDY_PLAN.md](STUDY_PLAN.md) 8절 학습 순서의 항목별 학습 내용을 기록한다.
이 패키지의 실제 코드(`gauge_ocr/gauge_ocr_node.py`, `launch/gauge_ocr.launch.py`)를 예제로 삼는다.

---

## 0. 실행 환경 — macOS + Docker + Gazebo 확인 창구

STUDY_PLAN 2.5·2.6절의 배경 정리. 2026-09-14 기준.

### 0.1 왜 Docker인가

| 구분 | 내용 |
|------|------|
| 내 머신 | macOS 26 (Apple Silicon, arm64) |
| 문제 | ROS 2 Jazzy는 macOS를 Tier 3로만 지원 → 바이너리 없음, 소스 빌드. Nav2, `ros_gz`, `cv_bridge`, `gz_ros2_control`, `foxglove_bridge`, `rosbridge`는 Mac 빌드 자체가 없음 |
| Gazebo 단독 | Gazebo Harmonic은 `brew`로 Mac 네이티브 설치가 되지만, 이 프로젝트는 ROS 브리지·Nav2와 함께 써야 하므로 의미 없음 |
| 결론 | Ubuntu 환경이 필요 → Mac에서 가장 가벼운 방법이 **Docker**. 부수 효과로 팀원·평가자가 `docker compose up` 한 번에 같은 환경을 얻는다(재현성) |

대안 비교:
- Ubuntu 네이티브 PC / 듀얼부팅: 성능 최고, 장비 필요.
- UTM·Parallels VM에 Ubuntu 24.04 arm64: GUI 편하지만 Docker보다 무겁고 재현성 낮음.
- 클라우드(EC2): 이전 프로젝트는 보너스 RL 학습에만 사용.

### 0.2 이전 프로젝트(`plant-robot-digital-twin`)에서 검증된 것

- `osrf/ros:humble-desktop`은 amd64 전용 → Apple Silicon에서 에뮬레이션, 빈 월드 RTF ≈ 0.5.
- Humble arm64 저장소에는 Gazebo Classic도 `ros-gz-sim`도 없음.
- **Jazzy + Gazebo Harmonic은 arm64 네이티브로 전체 스택 제공** (`ros-gz-sim`, `gz-ros2-control`, Nav2, Foxglove, rosbridge) → 채택. 이 프로젝트도 같은 조합.
- 베이스는 `ros:jazzy` 공식 이미지에 필요한 apt 패키지를 직접 얹는 방식. `LIBGL_ALWAYS_SOFTWARE=1`, `QT_X11_NO_MITSHM=1` 환경변수 사용.
- 참고 파일: `plant-robot-digital-twin/docker/Dockerfile`, `docker/docker-compose.yml`, `docker/smoke_test.sh`.

### 0.3 현재 저장소 Dockerfile·README에서 고칠 점

| 항목 | 현재 | 문제 | 조치 |
|------|------|------|------|
| 베이스 이미지 | `osrf/ros:jazzy-desktop` | arm64 태그 여부 미확인 (`docker manifest inspect`가 이 네트워크에서 실패) | 빌드 후 `uname -m`이 `aarch64`인지 확인. 아니면 `ros:jazzy`로 교체 |
| 소스 반영 | `COPY . /workspace` | 이미지 빌드 시점 스냅샷. 호스트에서 편집해도 반영 안 됨 | `-v $PWD:/workspace` 마운트 + 컨테이너 안 `colcon build --symlink-install` |
| 실행 방식 | `--network host`, `-v /tmp/.X11-unix` | Linux 전용. Mac에는 X11 소켓이 없고 host 네트워크도 동작이 다름 | `-p 8765:8765 -p 9090:9090`로 포트 명시, GUI는 0.4절 방식 |
| 의존성 | cv_bridge, message_filters, foxglove_bridge, rosbridge 없음 | 이 패키지 실행 불가 | apt 목록에 추가 |
| 빌드 산출물 | `build/ install/ log/` | `.gitignore`에 이미 있음 | 마운트 방식이면 호스트 저장소 안에 생기지만 커밋되지 않음 |

### 0.4 Gazebo 화면을 보는 세 가지 방법

**① Foxglove Studio (평소 개발용, 권장)**

```bash
# 컨테이너 안
ros2 launch simulation full_system.launch.py world:=factory gui:=false   # gz sim -s -r (서버만)
ros2 launch foxglove_bridge foxglove_bridge_launch.xml                   # ws 8765
```

Mac 네이티브 Foxglove Studio 앱 → Open connection → `ws://localhost:8765`.
로봇 3D 모델·TF·`/camera/image`·`/thermal/image`·토픽 플롯을 Mac GPU로 렌더링하므로 빠르고 안정적.
단점: Gazebo 월드(벽·계단 메쉬)는 안 보인다. ROS 토픽으로 나오는 것만 보임.

**② noVNC (진짜 Gazebo GUI가 필요할 때)**

- compose에 `theasp/novnc` 컨테이너를 추가하고 sim 컨테이너에 `DISPLAY=novnc:0.0`을 준다.
- 컨테이너 셸에서 `gz sim -g` 실행 → 브라우저 `http://localhost:8080/vnc.html`.
- 소프트웨어 렌더링(Mesa) + noVNC 이미지가 amd64라 에뮬레이션까지 겹쳐 느리다. 월드 배치 확인·스크린샷 용도로만.
- `rqt_image_view`도 X 창이 필요하므로 이 안에서만 뜬다.
- 루트 README의 대시보드가 같은 8080을 쓰므로 둘 중 하나는 포트를 바꾼다.

**③ 관제 대시보드 (시연·보고서 캡처)**

`rosbridge_server`(9090) → 브라우저에서 `monitoring/web`. 수현 담당.

XQuartz X11 포워딩은 Gazebo OGRE2와 궁합이 나빠 이전 프로젝트에서 noVNC로 대체했다.
Gazebo GUI 클라이언트만 Mac에 `brew`로 깔아 컨테이너 서버에 붙이는 방법은 gz-transport 멀티캐스트 발견이 Docker Desktop 네트워크를 못 넘어 실용적이지 않다.

### 0.5 헤드리스에서 카메라 센서가 도는 이유

`gui:=false`는 `gz sim -s -r`(서버 전용)이지만, 카메라 센서 렌더링은 서버 쪽 sensors 시스템(OGRE2)이 담당하므로 GUI 없이도 `/camera/image`, `/thermal/image`가 나온다.
GPU가 없으니 `LIBGL_ALWAYS_SOFTWARE=1`로 Mesa CPU 렌더링을 쓴다. RGB 640×480@15 Hz + 열화상 320×240@10 Hz를 CPU로 그리면 RTF가 떨어질 수 있으니 `ros2 topic hz`와 Gazebo RTF를 함께 확인하고, 느리면 해상도·주기를 낮추는 것을 시뮬 담당과 상의한다.

### 0.6 정리

- Mac에서 ROS 2 스택을 쓰는 이상 Docker가 사실상 유일한 현실적 경로.
- 평소: headless + Foxglove. 월드 편집·물리 디버깅: noVNC. 시연: 대시보드.
- 다음 액션: 이전 프로젝트 compose 파일을 가져와 이 저장소 구조(`/workspace`, `full_system.launch.py`)에 맞게 수정.

---

## 1. ROS 2 Python 기초 — 노드·토픽·파라미터·런치

### 1.1 핵심 개념

| 용어 | 뜻 | 이 패키지에서 |
|------|-----|--------------|
| **노드(Node)** | 하나의 실행 단위(프로세스). 이름을 갖고 토픽·파라미터를 소유 | `gauge_ocr` 노드 하나 |
| **토픽(Topic)** | 이름 붙은 데이터 통로. 발행자(publisher)가 쓰고 구독자(subscriber)가 읽음. N:M 가능 | 구독 `/camera/image`, 발행 `/inspection/gauge` |
| **메시지(Message)** | 토픽으로 오가는 데이터의 타입. `패키지/msg/이름` 형식 | `sensor_msgs/msg/Image`, `std_msgs/msg/Float32` |
| **파라미터(Parameter)** | 노드가 갖는 설정값. 런치·YAML·CLI로 바꿀 수 있음 | `world`, 앞으로 추가할 임계값들 |
| **런치(Launch)** | 여러 노드를 인자·파라미터와 함께 한 번에 띄우는 파이썬 스크립트 | `launch/gauge_ocr.launch.py` |
| **패키지(Package)** | 빌드·배포 단위. `package.xml` + `setup.py`(파이썬) | `gauge_ocr` |
| **워크스페이스** | 패키지들을 모아 `colcon build` 하는 디렉터리 | 저장소 루트 |
| **서비스/액션** | 요청-응답형 통신. 이 패키지에서는 당장 안 씀 | — |

토픽 이름의 슬래시는 네임스페이스 구분자다. `/inspection/gauge`는 "inspection 그룹의 gauge 토픽"이라는 뜻이며 폴더와 무관하다 (STUDY_PLAN.md 1절 참고).

### 1.2 워크스페이스와 빌드

```bash
source /opt/ros/jazzy/setup.bash          # ROS 환경 (터미널마다)
cd <저장소 루트>
colcon build --symlink-install --packages-select gauge_ocr
source install/setup.bash                 # 빌드 결과 환경 (빌드 후마다)
```

- `--symlink-install`: 파이썬 파일을 복사하지 않고 링크 → 코드 수정 후 재빌드 없이 바로 실행 반영. 단 `setup.py`·`package.xml`·`data_files`(launch, config)를 바꾸면 다시 빌드해야 한다.
- `--packages-select`: 해당 패키지만 빌드. 전체 빌드는 오래 걸린다.
- 빌드 산출물은 `build/`, `install/`, `log/`에 생기고 `.gitignore`에 들어가 있어야 한다.

패키지 구조:

```
gauge_ocr/
  package.xml          # 이름, 버전, 의존성 (apt/rosdep이 읽음)
  setup.py             # 파이썬 패키징 + 설치할 파일(data_files) + 실행 진입점(entry_points)
  setup.cfg            # 스크립트 설치 위치
  resource/gauge_ocr   # ament 인덱스 마커 (빈 파일, 건드리지 않음)
  gauge_ocr/           # 실제 파이썬 모듈
    __init__.py
    gauge_ocr_node.py
  launch/gauge_ocr.launch.py
```

`setup.py`의 `entry_points`에서 `gauge_ocr_node = gauge_ocr.gauge_ocr_node:main`이 `ros2 run gauge_ocr gauge_ocr_node` 명령을 만든다. 파일을 추가하면 모듈이 자동 포함되지만(`find_packages`), 새 실행 파일을 만들려면 여기에 추가해야 한다.

### 1.3 노드 구조 (현재 코드 해설)

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


class GaugeOcrNode(Node):
    def __init__(self) -> None:
        super().__init__("gauge_ocr")                       # 노드 이름
        self.create_subscription(Image, "/camera/image", self.camera_image_cb, 10)
        self.pub_0 = self.create_publisher(Float32, "/inspection/gauge", 10)
        self.create_timer(0.5, self._tick)                  # 0.5 s 주기 콜백
        self.get_logger().info("gauge_ocr started (운학)")

    def camera_image_cb(self, msg: Image) -> None:          # 이미지가 올 때마다 호출
        del msg

    def _tick(self) -> None:                                # 타이머 콜백
        out = Float32(); out.data = 0.0; self.pub_0.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)      # ROS 통신 초기화
    node = GaugeOcrNode()
    rclpy.spin(node)           # 콜백을 돌리며 블로킹. Ctrl+C까지
    node.destroy_node()
    rclpy.shutdown()
```

알아둘 점:
- `create_subscription(타입, 토픽, 콜백, QoS)`: 마지막 인자 `10`은 큐 깊이(depth)만 준 축약형 QoS다.
- `create_publisher(타입, 토픽, QoS)` → `.publish(msg)`.
- `spin`은 **단일 스레드**로 콜백을 순서대로 실행한다. 이미지 콜백에서 무거운 처리를 하면 다음 콜백이 밀린다. 콜백은 가볍게, 처리 결과는 멤버 변수에 저장하고 타이머에서 발행하는 구조가 안전하다.
- `self.get_logger().info/warn/error()`로 로그. `print`보다 이걸 쓴다.
- 이미지 구독과 발행이 분리돼 있어 현재는 "이미지가 안 와도 0.0을 계속 발행"한다. 실제 구현 시 "최근 판독값이 있을 때만 발행" 또는 "판독 즉시 발행"으로 바꿀지 결정.

### 1.4 메시지 타입 들여다보기

```bash
ros2 interface show sensor_msgs/msg/Image
ros2 interface show std_msgs/msg/Float32
```

`sensor_msgs/msg/Image` 주요 필드:

| 필드 | 뜻 |
|------|-----|
| `header.stamp` | 촬영 시각 (시뮬 시간) |
| `header.frame_id` | 카메라 좌표계 이름 (TF 연동 시 사용) |
| `height`, `width` | 480, 640 |
| `encoding` | `rgb8`, `bgr8`, `mono8`, `mono16` 등. Gazebo 카메라는 보통 `rgb8` |
| `step` | 한 행의 바이트 수 (= width × 채널 수) |
| `data` | 픽셀 바이트 배열. cv_bridge가 이걸 numpy로 바꿔준다 |

콜백에서 `msg.encoding`, `msg.width`를 로그로 찍어보는 게 첫 실습이다.

### 1.5 QoS (Quality of Service)

발행자와 구독자의 QoS가 호환되지 않으면 **연결 자체가 안 되고 에러도 안 난다**. 이미지가 안 들어오면 QoS부터 의심한다.

```python
from rclpy.qos import qos_profile_sensor_data

self.create_subscription(Image, "/camera/image", self.camera_image_cb, qos_profile_sensor_data)
```

- `qos_profile_sensor_data`: BEST_EFFORT + 작은 큐. 최신 프레임만 중요한 센서 데이터에 맞다.
- 기본 프로파일(`10`): RELIABLE. 발행자가 RELIABLE이면 구독자가 BEST_EFFORT여도 연결된다(반대는 안 됨). 그래서 센서 구독은 BEST_EFFORT가 안전하다.
- 확인: `ros2 topic info -v /camera/image` → 발행자·구독자 각각의 Reliability, Durability를 보여준다.

### 1.6 파라미터

선언 → 읽기 → (선택) 변경 콜백.

```python
self.declare_parameter("world", "corridor")
self.declare_parameter("alarm_max", 80.0)
world = self.get_parameter("world").get_parameter_value().string_value
alarm_max = self.get_parameter("alarm_max").value   # .value 축약형도 됨
```

- 선언하지 않은 파라미터를 런치에서 넘기면 **무시**된다. 현재 런치가 `world`를 넘기지만 노드가 선언하지 않아 아무 효과가 없는 상태다.
- 타입은 기본값에서 추론된다. `80.0`(float)로 선언했는데 `80`(int)을 넘기면 에러.
- 실행 중 바꾸기:

```bash
ros2 param list /gauge_ocr
ros2 param get /gauge_ocr alarm_max
ros2 param set /gauge_ocr alarm_max 90.0
```

- 실행 중 변경을 반영하려면 `self.add_on_set_parameters_callback(cb)`로 콜백을 달거나, 매 틱마다 `get_parameter`로 다시 읽는다. 임계값 튜닝에는 후자가 간단하다.
- YAML 파일로 묶어 넘기기 (`config/gauge_factory.yaml`):

```yaml
gauge_ocr:                # 노드 이름
  ros__parameters:
    alarm_max: 80.0
    needle_angle_min: 225.0
```

`setup.py`의 `data_files`에 `(os.path.join("share", package_name, "config"), glob("config/*.yaml"))`을 추가해야 설치된다 (patrol_path 패키지가 이미 이 방식을 쓰고 있으니 참고).

### 1.7 런치 파일 (현재 코드 해설)

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="corridor"),   # ros2 launch ... world:=factory
        Node(
            package="gauge_ocr",
            executable="gauge_ocr_node",        # setup.py entry_points 이름
            name="gauge_ocr",                   # 노드 이름 덮어쓰기
            output="screen",                    # 로그를 터미널로
            parameters=[{"world": LaunchConfiguration("world"), "use_sim_time": True}],
        ),
    ])
```

- `LaunchConfiguration("world")`는 런치 인자를 나중에 평가하는 치환자(substitution)다. 파이썬 문자열처럼 바로 쓸 수 없다.
- `use_sim_time: True`: 노드의 시계를 `/clock` 토픽(시뮬 시간)에 맞춘다. 이게 켜져 있으면 시뮬이 안 돌 때 `self.get_clock().now()`가 0에 머문다.
- YAML 파라미터 파일을 추가로 넘기려면 `parameters=[yaml_path, {...}]`처럼 리스트에 경로를 넣는다. 경로는 `get_package_share_directory("gauge_ocr")`로 얻는다.
- 통합 런치 `simulation/launch/full_system.launch.py`가 이 파일을 `IncludeLaunchDescription`으로 포함하고 `world`를 넘긴다. 즉 이 런치를 고치면 통합 실행에도 반영된다.

### 1.8 자주 쓰는 CLI

```bash
ros2 node list                       # 떠 있는 노드
ros2 node info /gauge_ocr            # 구독·발행·파라미터 목록
ros2 topic list                      # 토픽 목록
ros2 topic echo /inspection/gauge    # 값 확인
ros2 topic hz /camera/image          # 주기 (15 Hz 나와야 정상)
ros2 topic info -v /camera/image     # 발행자·구독자·QoS
ros2 param list /gauge_ocr
ros2 interface show sensor_msgs/msg/Image
ros2 run gauge_ocr gauge_ocr_node    # 런치 없이 노드만
ros2 launch gauge_ocr gauge_ocr.launch.py world:=factory
```

### 1.9 실습 체크리스트

- [ ] 패키지 단독 빌드 후 `ros2 run gauge_ocr gauge_ocr_node` 실행, 다른 터미널에서 `ros2 topic echo /inspection/gauge`로 0.0 확인
- [ ] 시뮬 기동 후 `ros2 topic hz /camera/image`로 15 Hz 확인, `ros2 topic info -v`로 QoS 확인
- [ ] `camera_image_cb`에서 `msg.encoding`, `msg.width`, `msg.height`를 한 번만 로그로 출력 (매 프레임 찍으면 터미널이 넘친다 → 플래그로 1회)
- [ ] `world` 파라미터 선언 추가, 시작 로그에 world 값 출력, `world:=factory`로 런치해 바뀌는지 확인
- [ ] `alarm_max` 파라미터 선언 후 `ros2 param set`으로 실행 중 변경, 타이머에서 읽어 로그 확인
- [ ] 구독 QoS를 `qos_profile_sensor_data`로 바꾸고 이미지가 계속 들어오는지 확인
- [ ] `config/gauge_corridor.yaml` 만들고 `setup.py` data_files + 런치 `parameters`에 연결

### 1.10 헷갈리기 쉬운 것

- 새 터미널마다 `source /opt/ros/jazzy/setup.bash`와 `source install/setup.bash`를 둘 다 해야 한다. 노드가 "package not found"면 십중팔구 이것.
- `--symlink-install`이어도 `setup.py`·launch·config 변경은 재빌드 필요.
- 타이머 콜백과 구독 콜백은 같은 스레드에서 번갈아 돈다. 콜백 안에서 `time.sleep`이나 `cv2.waitKey` 같은 블로킹을 하면 전체가 멈춘다.
- `Float32` 같은 메시지 객체에 파이썬 `int`를 넣으면 타입 에러 (`out.data = 0` ✗, `0.0` ✓).
- 노드 이름(`gauge_ocr`)과 패키지 이름(`gauge_ocr`), 실행 파일 이름(`gauge_ocr_node`)은 서로 다른 것이다. CLI에서 각각 어디에 쓰이는지 구분한다.

### 1.11 참고

- ROS 2 Jazzy 튜토리얼 (Beginner: CLI tools → Client libraries): https://docs.ros.org/en/jazzy/Tutorials.html
- rclpy API: https://docs.ros.org/en/jazzy/p/rclpy/
- QoS 설명: https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html
- 런치 튜토리얼: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html

---

## 2. cv_bridge로 이미지 받아 PNG 저장

계획 2번 항목. 목표는 **`/camera/image`가 콜백에 들어오는 순간 numpy 배열로 바꿔 PNG로 떨어뜨리는 것**까지다.
이게 되면 이후 OpenCV 작업(계획 5번)은 시뮬 없이 PNG만으로 진행할 수 있다.

### 2.1 cv_bridge가 하는 일

`sensor_msgs/msg/Image`는 픽셀이 1차원 바이트 배열(`data`)에 `encoding`·`width`·`height`·`step`과 함께 실려 온다.
`cv_bridge`는 이걸 OpenCV가 쓰는 numpy 배열(H×W×C)로 바꿔 주고, 반대 변환도 해 준다.

| 방향 | 함수 | 이 패키지에서 |
|------|------|--------------|
| ROS → OpenCV | `CvBridge().imgmsg_to_cv2(msg, desired_encoding)` | 카메라 프레임 받기 |
| OpenCV → ROS | `CvBridge().cv2_to_imgmsg(img, encoding)` | 디버그 오버레이 발행 (계획 6번) |

`desired_encoding`에 따라 변환이 달라진다:

| 인자 | 결과 | 언제 |
|------|------|------|
| `"passthrough"` | 원본 인코딩 그대로 | 포맷을 그대로 두고 싶을 때. `rgb8`이면 R·B 순서가 OpenCV와 반대 |
| `"bgr8"` | 3채널, B-G-R 순서 (OpenCV 기본) | **RGB 카메라는 이걸로 받는다** |
| `"mono8"` | 1채널 8비트 | 흑백이 필요할 때 (`bgr8`로 받고 `cvtColor`해도 됨) |
| `"mono16"` | 1채널 16비트 (`uint16`) | 진짜 thermal 센서 (thermal_fusion 참고) |

Gazebo 카메라(`simulation/models/go2/model.sdf`의 `camera` 센서)는 포맷을 지정하지 않았으므로 기본값 **`R8G8B8` → ROS `rgb8`**로 온다.
`imgmsg_to_cv2(msg, "bgr8")`이면 cv_bridge가 R·B 채널을 바꿔 주므로 `cv2.imwrite`로 저장했을 때 색이 맞는다. `"passthrough"`로 받아 그대로 저장하면 빨강·파랑이 뒤바뀐 PNG가 나온다.

### 2.2 설치와 의존성

```bash
sudo apt install ros-jazzy-cv-bridge python3-opencv     # Docker면 Dockerfile apt 목록에 추가
python3 -c "import cv_bridge, cv2; print(cv2.__version__)"
```

`package.xml`에 추가:

```xml
<exec_depend>cv_bridge</exec_depend>
<exec_depend>python3-opencv</exec_depend>
<exec_depend>python3-numpy</exec_depend>
```

- `cv_bridge`는 apt로 설치되는 시스템 파이썬 모듈이라 `pip`로 깔지 않는다. Jazzy(Ubuntu 24.04)에서 `pip install opencv-python`을 섞으면 apt의 `python3-opencv`와 충돌하므로 **apt만** 쓴다.
- `import cv_bridge`가 안 되면 `source /opt/ros/jazzy/setup.bash`를 안 한 것이다.

### 2.3 cv_bridge 없이 직접 변환해 보기 (원리 이해용)

cv_bridge가 내부에서 하는 일은 결국 이것이다. 한 번 손으로 해 보면 `step`·`encoding`의 의미가 잡힌다.

```python
import numpy as np

def image_to_bgr(msg) -> np.ndarray:
    # data는 길이 height*step 인 바이트 배열. step은 한 행의 바이트 수(패딩 포함 가능)
    flat = np.frombuffer(msg.data, dtype=np.uint8)
    rows = flat.reshape(msg.height, msg.step)          # (H, step)
    img = rows[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
    if msg.encoding == "rgb8":
        img = img[:, :, ::-1]                          # RGB → BGR
    elif msg.encoding != "bgr8":
        raise ValueError(f"unexpected encoding {msg.encoding}")
    return np.ascontiguousarray(img)                   # OpenCV 함수에 넘기려면 연속 메모리
```

- `np.frombuffer`는 복사 없이 뷰를 만든다. `msg`가 살아 있는 동안만 유효하므로 저장해 둘 거면 `.copy()`.
- `step`이 `width*3`과 같은 게 보통이지만 항상 그렇다고 가정하지 않는다(위 코드처럼 자른다).

### 2.4 노드 콜백에 붙이기 — 주기 저장

현재 `camera_image_cb`는 `del msg`로 프레임을 버린다. 이걸 "N초마다 한 장 PNG 저장"으로 바꾼다.

```python
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data


class GaugeOcrNode(Node):
    def __init__(self) -> None:
        super().__init__("gauge_ocr")
        self.declare_parameter("world", "corridor")
        self.declare_parameter("dump_dir", "")          # 비어 있으면 저장 안 함
        self.declare_parameter("dump_period", 2.0)      # 초
        self._bridge = CvBridge()
        self._last_dump = None
        self._logged_once = False
        self.create_subscription(Image, "/camera/image", self.camera_image_cb, qos_profile_sensor_data)
        ...

    def camera_image_cb(self, msg: Image) -> None:
        if not self._logged_once:                      # 포맷 확인은 첫 프레임 한 번만
            self.get_logger().info(f"camera {msg.width}x{msg.height} {msg.encoding} step={msg.step}")
            self._logged_once = True

        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().warn(f"cv_bridge: {e}")
            return

        self._latest = bgr                             # 이후 판독 로직은 이 프레임을 쓴다
        self._maybe_dump(msg, bgr)

    def _maybe_dump(self, msg: Image, bgr) -> None:
        dump_dir = self.get_parameter("dump_dir").value
        if not dump_dir:
            return
        now = self.get_clock().now()                   # use_sim_time=True 면 시뮬 시계
        period = Duration(seconds=self.get_parameter("dump_period").value)
        if self._last_dump is not None and now - self._last_dump < period:
            return
        os.makedirs(dump_dir, exist_ok=True)
        stamp = msg.header.stamp
        path = os.path.join(dump_dir, f"camera_{stamp.sec}_{stamp.nanosec:09d}.png")
        if cv2.imwrite(path, bgr):
            self.get_logger().info(f"saved {path}")
        else:
            self.get_logger().warn(f"imwrite failed: {path}")
        self._last_dump = now
```

알아둘 점:
- 파일명에 `header.stamp`를 넣으면 나중에 rosbag·`/odom`과 시각을 맞출 수 있다. 시뮬 시간이라 `0_000000000`처럼 작게 시작한다.
- `cv2.imwrite`는 **디렉터리가 없거나 확장자가 이상하면 예외 없이 `False`만 돌려준다**. 반환값을 꼭 확인한다.
- 콜백 안에서 `cv2.imshow`·`cv2.waitKey`는 쓰지 않는다(spin이 막힘). 확인은 저장된 PNG나 Foxglove Image 패널로.
- 저장 주기 조절은 `ros2 param set /gauge_ocr dump_period 0.5`. 끄려면 `dump_dir`을 빈 문자열로.

실행:

```bash
ros2 launch gauge_ocr gauge_ocr.launch.py world:=factory      # 런치에 dump_dir 파라미터를 추가하거나
ros2 run gauge_ocr gauge_ocr_node --ros-args -p dump_dir:=/workspace/docs/captures -p dump_period:=2.0 -p use_sim_time:=true
```

### 2.5 저장 위치 (Docker일 때)

- 컨테이너 안 경로가 호스트에 마운트된 곳(`/workspace` = 저장소)이어야 Mac에서 파일이 보인다. `/tmp`에 저장하면 컨테이너가 죽을 때 사라진다.
- `docs/captures/*.png`는 `.gitignore`에 있으므로 그 아래에 마음껏 저장해도 커밋되지 않는다. 보고서에 쓸 캡처만 골라서 `git add -f`.
- 파일 소유자가 root가 되는 문제(Linux 호스트)는 Docker Desktop for Mac에서는 생기지 않는다.

### 2.6 반대 방향: numpy → Image 발행 (계획 6번 디버그 토픽의 기초)

```python
out = self._bridge.cv2_to_imgmsg(overlay_bgr, encoding="bgr8")
out.header = msg.header          # 원본 프레임의 stamp·frame_id를 그대로. Foxglove 시간축·TF가 맞는다
self.pub_debug.publish(out)
```

`header`를 안 넣으면 stamp가 0이라 Foxglove에서 이미지가 안 뜨거나 시간축이 어긋난다.

### 2.7 코드 없이 저장하는 방법 (비교용)

| 방법 | 명령 | 비고 |
|------|------|------|
| `image_view` 패키지의 `image_saver` | `ros2 run image_view image_saver --ros-args -r image:=/camera/image -p filename_format:=frame%04d.png` | `ros-jazzy-image-view` 설치 필요. 빠른 확인용 |
| Foxglove Image 패널 | 패널 메뉴 → Download image | 한 장씩. 보고서 캡처에 편함 |
| rosbag → 나중에 추출 | 계획 3번 | 반복 실험에는 이쪽 |

노드에 직접 넣는 이유는 "판독 로직이 실제로 받는 프레임"을 그대로 남기기 위해서다. 인코딩 변환까지 같은 코드를 타므로 오프라인 튜닝 결과가 노드에서 그대로 재현된다.

### 2.8 실습 체크리스트

- [ ] `python3 -c "import cv_bridge, cv2"` 성공 (Docker면 이미지 재빌드 후)
- [ ] `package.xml`에 `cv_bridge`, `python3-opencv`, `python3-numpy` 추가 → `colcon build --packages-select gauge_ocr`
- [ ] 첫 프레임 로그에서 `640x480 rgb8 step=1920` 확인
- [ ] 2.3의 수동 변환과 `imgmsg_to_cv2(msg, "bgr8")` 결과가 같은지 `np.array_equal`로 확인
- [ ] `dump_dir:=/workspace/docs/captures`로 실행해 PNG가 Mac 파인더에서 보이는지 확인. 색(빨강/파랑)이 맞는지 눈으로 확인
- [ ] 로봇을 factory `gauge_line1` 앞에 세워 놓고 게이지가 찍힌 프레임 한 장 확보 → 계획 5번의 첫 입력
- [ ] `ros2 param set`으로 `dump_period`를 바꿔 저장 간격이 바뀌는지 확인
- [ ] 저장된 PNG를 `cv2.imread`로 다시 읽어 `shape == (480, 640, 3)` 확인

### 2.9 헷갈리기 쉬운 것

- **색이 뒤집힘**: `passthrough`로 받은 `rgb8`을 `imwrite`하면 R·B가 바뀐다. `bgr8`로 받거나 `cvtColor(img, COLOR_RGB2BGR)`.
- `imwrite`가 조용히 실패: 디렉터리 없음, 경로에 확장자 없음, 배열이 `float`(0~1)인데 그대로 저장 → `uint8`로 변환(`(img*255).astype(np.uint8)`).
- `np.frombuffer` 결과는 읽기 전용이다. 픽셀을 수정하려면 `.copy()`.
- QoS를 `qos_profile_sensor_data`로 바꾸지 않으면 프레임이 밀려서 저장되는 이미지가 몇 초 전 것일 수 있다.
- `use_sim_time`이 켜졌는데 시뮬이 안 돌면 `get_clock().now()`가 멈춰 있어 `dump_period` 판정이 영원히 안 된다. `ros2 topic hz /clock` 확인.
- PNG는 무손실이라 640×480 한 장에 수백 KB. 저장 주기를 짧게 오래 돌리면 금방 커진다.

### 2.10 참고

- cv_bridge 튜토리얼 (ROS 2): https://docs.ros.org/en/jazzy/p/cv_bridge/
- `sensor_msgs/msg/Image` 정의와 인코딩 문자열: https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html , `sensor_msgs/image_encodings.hpp`
- OpenCV `imread`/`imwrite`: https://docs.opencv.org/4.x/d4/da8/group__imgcodecs.html
- Gazebo 카메라 센서 SDF (`<format>` 기본값 R8G8B8): https://sdformat.org/spec?elem=sensor

## 3. rosbag 녹화와 오프라인 데이터셋

계획 3번 항목. 목표는 **시뮬을 한 번만 돌려 녹화해 두고, 이후 판독 알고리즘 개발은 bag과 PNG만으로 반복하는 것**이다.
Mac + Docker 환경에서는 Gazebo가 CPU 렌더링이라 느리므로 이 단계의 가치가 특히 크다.

### 3.1 rosbag2 기본

| 명령 | 하는 일 |
|------|---------|
| `ros2 bag record -o <디렉터리> <토픽...>` | 지정 토픽 녹화. `-o` 디렉터리가 이미 있으면 에러 |
| `ros2 bag record -a` | 모든 토픽. 이미지·포인트클라우드까지 들어가 금방 커지므로 쓰지 않는다 |
| `ros2 bag info <디렉터리>` | 길이, 토픽별 메시지 수, 저장 형식 확인 |
| `ros2 bag play <디렉터리>` | 재생. `--loop`, `--rate 0.5`, `--start-offset 10` |

- Jazzy의 기본 저장 형식은 **MCAP**(`*.mcap`)이다. 예전 자료의 `.db3`(SQLite)와 다르다. Foxglove Studio에서 MCAP 파일을 그대로 열 수 있다.
- bag은 **디렉터리**(`metadata.yaml` + `*.mcap`)다. 옮길 때 디렉터리째 옮긴다.
- `.gitignore`에 `*.mcap`, `*.bag`이 있어 실수로 커밋되지 않는다. `metadata.yaml`은 걸러지지 않으니 bag 디렉터리는 `docs/captures/` 밖(예: `bags/`, 저장소 밖 마운트 경로)에 두거나 통째로 add하지 않는다.

### 3.2 무엇을 녹화하나

```bash
ros2 bag record -o /workspace/bags/factory_gauge_01 \
  /clock /camera/image /camera/camera_info /odom /tf /mission/status
```

| 토픽 | 왜 |
|------|-----|
| `/camera/image` | 판독 입력 |
| `/camera/camera_info` | 계획 5번 ROI 투영에 K 행렬 필요 |
| `/odom`, `/tf` | 프레임마다 로봇 위치·자세 → "어느 거리·각도에서 찍혔나" 기록, 정차 판정 |
| `/mission/status` | `goto:gauge_line1` 같은 정차점 이름 → 데이터셋 폴더 분류 기준 |
| `/clock` | 재생할 때 시뮬 시간을 그대로 되살리기 위해 (3.3절) |

용량 감각: 640×480×3 B × 15 Hz ≈ **13.8 MB/s**, 1분이면 약 830 MB. 순찰 한 바퀴를 통째로 녹화하지 말고 **게이지 정차 구간만** 녹화한다.
- 로봇이 정차점에 도착하면 녹화 시작, dwell(5 s) 끝나면 Ctrl+C. 정차점마다 bag 하나.
- `--max-bag-duration 60`으로 파일을 잘게 나누거나, `--compression-mode file --compression-format zstd`로 압축할 수 있다(재생 시 자동 해제, 대신 CPU 사용).

### 3.3 재생할 때의 시간 문제

노드는 `use_sim_time=True`로 동작한다. 재생 시 `/clock`을 누가 내느냐에 따라 결과가 다르다.

| 방식 | 명령 | `get_clock().now()` | `header.stamp` | 결과 |
|------|------|---------------------|----------------|------|
| **녹화한 `/clock` 재생** (권장) | `ros2 bag play <bag>` | 녹화 당시 시뮬 시간 | 녹화 당시 시뮬 시간 | 둘이 같은 시간축 → 시뮬에서와 똑같이 동작 |
| bag이 `/clock` 생성 | `ros2 bag play <bag> --clock` | 녹화 당시 **수신 시각**(벽시계) | 시뮬 시간 | 시간축이 어긋남. `/clock`을 녹화하지 않았을 때만 |

- 그래서 3.2절에서 `/clock`을 같이 녹화했다. `--clock`과 녹화된 `/clock`을 동시에 쓰면 시계가 두 개가 되어 시간이 앞뒤로 튄다.
- 재생 중에는 시뮬을 꺼 둔다. 시뮬과 bag이 같은 토픽을 동시에 내면 섞인다.
- `--loop` 재생 시 시간이 처음으로 되돌아간다. 2.4절의 `dump_period` 판정처럼 "이전 시각과 비교"하는 코드는 시간이 역행하면(`now < _last_dump`) 리셋하도록 짜 둔다.

### 3.4 bag → PNG 추출 스크립트

노드를 띄워 `dump_dir`로 저장해도 되지만(2.4절), **bag을 직접 읽어 추출**하면 재생 속도와 QoS 손실 없이 모든 프레임을 꺼낼 수 있다.
`rosbag2_py`는 ROS 2 설치에 포함된 파이썬 API다.

```python
#!/usr/bin/env python3
"""bag에서 /camera/image를 PNG로, 프레임별 메타데이터를 CSV로 뽑는다.

사용: python3 bag_to_png.py <bag_dir> <out_dir> [--every N]
"""
import argparse
import csv
import math
import os

import cv2
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("out")
    ap.add_argument("--every", type=int, default=5, help="N장마다 1장 (15 Hz → 3 Hz)")
    args = ap.parse_args()

    reader = SequentialReader()
    reader.open(StorageOptions(uri=args.bag, storage_id="mcap"), ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    bridge = CvBridge()
    os.makedirs(args.out, exist_ok=True)
    pose = (float("nan"), float("nan"), float("nan"))   # 가장 최근 /odom (x, y, yaw)
    status = ""
    n_img = 0

    with open(os.path.join(args.out, "frames.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["file", "stamp", "x", "y", "yaw", "status"])
        while reader.has_next():
            topic, data, _t = reader.read_next()        # 메시지는 기록 순서대로 나온다
            msg = deserialize_message(data, get_message(types[topic]))
            if topic == "/odom":
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                yaw = 2.0 * math.atan2(q.z, q.w)   # 평지 가정 (roll·pitch ≈ 0)
                pose = (p.x, p.y, yaw)
            elif topic == "/mission/status":
                status = msg.data
            elif topic == "/camera/image":
                n_img += 1
                if n_img % args.every:
                    continue
                s = msg.header.stamp
                name = f"camera_{s.sec}_{s.nanosec:09d}.png"
                cv2.imwrite(os.path.join(args.out, name), bridge.imgmsg_to_cv2(msg, "bgr8"))
                w.writerow([name, f"{s.sec}.{s.nanosec:09d}", *(f"{v:.3f}" for v in pose), status])


if __name__ == "__main__":
    main()
```

- `read_next()`는 `(토픽, 직렬화된 바이트, 기록 시각 ns)`를 준다. `deserialize_message` + `get_message("sensor_msgs/msg/Image")`로 메시지 객체가 된다.
- `/odom`은 이미지와 따로 오므로 "이미지 직전의 최신 odom"을 붙인다. 50 Hz 이상이면 오차는 수 cm.
- 쿼터니언 → yaw: 평지에서 roll·pitch가 0이면 `yaw = 2·atan2(z, w)`. 일반적으로는 `atan2(2(wz+xy), 1−2(y²+z²))`.
- 이 스크립트는 노드가 아니므로 `rclpy.init()`이 필요 없다. 다만 `source /opt/ros/jazzy/setup.bash`는 해야 `rosbag2_py`가 import된다.
- 위치: 패키지 안에 넣을 거면 `gauge_ocr/tools/bag_to_png.py`. 설치 대상(`entry_points`)에는 넣지 않는다.

### 3.5 데이터셋 구성

```
docs/captures/gauge/               # *.png는 .gitignore로 커밋 안 됨
  factory_gauge_line1_01/
    camera_123_450000000.png
    ...
    frames.csv                     # file, stamp, x, y, yaw, status
    labels.csv                     # file, needle_deg, value  ← 정답 (4절 모델에서 지정한 값)
  corridor_gauge_corridor_01/
  ...
```

- **정답(label)이 핵심**이다. 시뮬은 바늘 각도를 우리가 정하므로(4절) bag 이름이나 `labels.csv`에 그 값을 반드시 남긴다. 이게 있어야 계획 5번에서 판독 오차를 숫자로 낼 수 있다.
- 다양성: 바늘 각도 여러 개 × 정차 거리·좌우 오프셋 몇 개. 처음엔 각도 5개(0%, 25%, 50%, 75%, 100%) × 정면 1위치로 시작하고, 알고리즘이 돌면 늘린다.
- 학습용/검증용 분리는 필요 없다(학습하는 모델이 아님). 대신 **튜닝에 쓴 세트**와 **최종 측정 세트**는 나눈다. 같은 이미지로 튜닝하고 측정하면 오차가 과소평가된다.
- PNG는 커밋되지 않으므로 팀 공유가 필요하면 bag 디렉터리를 압축해 드라이브로 공유하고, 저장소에는 `frames.csv`·`labels.csv`와 대표 캡처 몇 장만 `git add -f`.

### 3.6 실습 체크리스트

- [ ] 시뮬 실행 → 로봇을 `gauge_line1` 정차점에 세우고 3.2절 명령으로 10초 녹화
- [ ] `ros2 bag info`로 `/camera/image` 메시지 수가 약 150개(15 Hz × 10 s)인지 확인. 크게 적으면 RTF 저하 → 기록
- [ ] 시뮬 끄고 `ros2 bag play` → Foxglove에서 이미지가 재생되는지 확인
- [ ] 재생 중 `ros2 topic echo --once /clock`과 `ros2 topic echo --once --field header.stamp /camera/image`가 같은 시간대인지 확인 (3.3절)
- [ ] 2.4절 노드를 `dump_dir`과 함께 띄워 bag 재생으로 PNG가 저장되는지 확인 (시뮬 없이 노드 테스트가 되는지)
- [ ] 3.4절 스크립트로 PNG + `frames.csv` 추출, 메시지 수와 PNG 수(÷`--every`) 일치 확인
- [ ] Foxglove Studio에서 `.mcap` 파일을 직접 열어 보기
- [ ] 3.5절 구조로 첫 데이터셋 폴더 만들기 (모델 보강 전이라면 원판만 찍힌 상태로라도)

### 3.7 헷갈리기 쉬운 것

- `-o` 경로를 컨테이너 안 `/root/...`로 주면 Mac에서 안 보이고 컨테이너 삭제 시 사라진다. 마운트 경로(`/workspace/...`) 아래로.
- 이미지 토픽 QoS가 BEST_EFFORT여도 recorder가 발행자 QoS에 맞춰 구독하므로 녹화는 된다. 다만 CPU가 바쁘면 프레임이 빠진다 → `ros2 bag info`의 메시지 수로 확인.
- 재생 시 `ros2 bag play`의 기본 QoS는 녹화 당시 발행자 QoS를 따른다. 구독 쪽이 RELIABLE이고 bag 재생이 BEST_EFFORT면 안 맞아 메시지가 안 온다 → 노드 구독을 `qos_profile_sensor_data`로 맞춰 둔 이유.
- `storage_id="mcap"`인데 예전 `.db3` bag을 열면 에러. `ros2 bag info`의 `Storage id`를 보고 맞춘다.
- 3.4 스크립트가 `ModuleNotFoundError: rosbag2_py` → ROS 환경 source 안 함.

### 3.8 참고

- rosbag2 튜토리얼: https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html
- 파이썬으로 bag 읽기: https://docs.ros.org/en/jazzy/Tutorials/Advanced/Reading-From-A-Bag-File-Python.html
- MCAP 형식·Foxglove: https://mcap.dev/ , https://docs.foxglove.dev/docs/connecting-to-data/local-data
- 시뮬 시간과 bag 재생: https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Time.html

## 4. 게이지 모델 보강

계획 4번 항목. 현재 `simulation/models/gauge/model.sdf`는 **어두운 패널 + 밝은 원판**뿐이라 읽을 바늘이 없다.
판독 알고리즘을 만들기 전에 ① 카메라에 게이지가 **보이는지**, ② 바늘·눈금을 **어떻게 넣을지** 정하고 시뮬 담당·태우와 협의한다.

### 4.1 현재 모델 구조 읽기

```xml
<model name="gauge">
  <static>true</static>
  <link name="panel">            <!-- 0.04 × 0.35 × 0.35 m 박스. 모델 x축이 두께 방향 -->
  <link name="dial">
    <pose>0.03 0 0 0 1.5708 0</pose>   <!-- pitch 90° → 원기둥 축(z)이 모델 x축으로 눕는다 -->
    <cylinder> r = 0.12, length = 0.02 </cylinder>
```

- 원판 앞면은 모델 좌표 x = 0.03 + 0.01 = **0.04 m**에 있고, **모델 +x 방향을 바라본다**.
- 월드에서는 `<pose>10 8.7 1.5 0 0 -1.5708</pose>`(yaw −90°)로 놓여 모델 +x가 월드 −y를 향한다. 즉 벽(y = 9) 앞에서 로봇 쪽(−y)을 본다.
- SDF 포즈 순서는 `x y z roll pitch yaw`, 회전은 라디안. 링크 `<pose>`는 모델 기준.

### 4.2 먼저 확인할 것: 정차점에서 게이지가 화면에 들어오는가

카메라 센서는 `base_link` 기준 (0.28, 0, 0.05)에 있고(`model.sdf` 64행), 모델 스폰 높이가 0.4 m이므로 카메라 높이는 약 **0.45 m**, 수평으로 앞을 본다.
수직 화각은 `fy = fx ≈ 381 px`, 반 화각 = `atan(240 / 381)` ≈ **±32°**다.

| 월드 | 게이지 (월드) | 정차점 (patrol_path) | 수평 거리 | 높이 차 | 올려다보는 각 | 판정 |
|------|---------------|----------------------|-----------|---------|---------------|------|
| factory | `gauge_line1` (10, 8.7, **1.5**) | (10, 7.5, yaw 1.57) | 0.87 m | 1.05 m | **50°** | 화면 밖 |
| corridor | `gauge_corridor` (15, 1.55, **1.4**) | (15, 0.4, yaw 1.57) | 0.82 m | 0.95 m | **49°** | 화면 밖 |

계산: 수평 거리 = (원판 앞면 y) − (정차점 y + 0.28), 올려다보는 각 = `atan(높이 차 / 수평 거리)`.
**50° > 32°이므로 지금 배치로는 게이지가 프레임 위로 벗어나 아예 찍히지 않는다.** 이게 가장 먼저 풀어야 할 협의 사항이다.

해결안 비교 (원판 반지름의 화면상 크기 = `fx · 0.12 / 거리`):

| 안 | 바꾸는 것 | 올려다보는 각 | 원판 반지름 | 영향 범위 |
|----|-----------|---------------|-------------|-----------|
| **A. 게이지 높이 낮춤** (권장) | 월드 `<pose>`의 z: 1.5 → **0.7** (corridor 1.4 → 0.7) | 16° | ≈ 50 px | 월드 파일 두 줄. 다른 모듈 영향 없음 |
| B. 정차 거리 늘림 | 정차점을 게이지에서 약 2.25 m 떨어지게 (factory y 7.5 → 6.1) | 25° | ≈ 18 px | 태우 웨이포인트. 원판이 작아져 판독 정밀도 하락 |
| C. 카메라를 위로 기울임 | `camera` 센서 pose의 pitch를 −0.5 rad 정도 | 50° 그대로, 광축이 올라감 | ≈ 34 px | Go2 모델. `/camera/image`를 쓰는 다른 모듈에 영향 |

- A가 원판이 가장 크게, 가장 정면에 가깝게 찍힌다(올려다보는 각 16°면 원판이 세로로 약 4% 눌린 타원 → 거의 원).
- "현실 공장 게이지는 눈높이(1.5 m)에 있다"는 반론에는 **사족 로봇 카메라 높이에 맞춘 점검용 게이지**로 설명하거나, 실제 로봇처럼 C(틸트)를 검토한다.
- SDF에서 pitch 양수는 y축 기준 회전이라 **기수가 아래로** 숙여진다. 위를 보려면 음수다. 바꾸고 나면 반드시 이미지로 확인한다.

### 4.3 바늘 넣기

STUDY_PLAN 6절 권장대로 **바늘은 별도 박스 visual**로 만든다. 각도를 SDF 숫자 하나로 바꿀 수 있어 정답 라벨 만들기가 쉽다.

바늘 각도 규칙을 먼저 정한다 (이 문서 전체에서 이 규칙을 쓴다):
- **φ = 화면에서 12시 방향 기준 시계 방향 각도** (사람이 게이지 보는 방식). 12시 = 0°, 3시 = 90°, 6시 = 180°.
- 일반 아날로그 게이지처럼 **최소값 φ = 225°(7시 반) → 시계 방향 270° 스윕 → 최대값 φ = 135°(4시 반)**.

모델 좌표에서 바늘 방향 구하기:
- 원판 앞(+x)에서 원판을 보면 화면 오른쪽 = 모델 +y, 위 = 모델 +z.
  (시선 방향 −x와 위쪽 +z의 외적 → 오른쪽 = +y)
- 바늘을 모델 x축 기준으로 ψ만큼 roll 회전하면 바늘 끝 방향은 (0, −sin ψ, cos ψ). ψ > 0이면 화면 왼쪽으로 돈다 = 반시계.
- 따라서 **ψ = −φ** (라디안).
- 바늘은 중심에서 한쪽으로만 뻗어야 한다(양쪽으로 뻗으면 판독 시 180° 모호). 길이 L 박스를 중심에서 L/2만큼 밀어 놓는다.

```xml
<!-- dial 링크 다음에 추가. φ = 90°(3시) 예시: ψ = −1.5708 -->
<link name="needle">
  <!-- 위치 = (0.045, −(L/2)·sin ψ, (L/2)·cos ψ), 회전 = (ψ, 0, 0), L = 0.10 -->
  <pose>0.045 0.05 0.0 -1.5708 0 0</pose>
  <visual name="visual">
    <geometry><box><size>0.004 0.008 0.10</size></box></geometry>   <!-- 두께 x, 폭 y, 길이 z -->
    <material>
      <ambient>0.8 0.05 0.05 1</ambient>
      <diffuse>0.9 0.05 0.05 1</diffuse>
      <emissive>0.4 0 0</emissive>          <!-- 그늘에서도 빨강이 유지되게 -->
    </material>
  </visual>
</link>
<link name="hub">                            <!-- 중심 캡: 바늘 뿌리를 가려 중심 검출이 쉬워짐 -->
  <pose>0.05 0 0 0 1.5708 0</pose>
  <visual name="visual">
    <geometry><cylinder><radius>0.012</radius><length>0.006</length></cylinder></geometry>
    <material><diffuse>0.1 0.1 0.1 1</diffuse></material>
  </visual>
</link>
```

- x = 0.045: 원판 앞면(0.04)보다 살짝 앞. 같은 위치에 겹치면 z-fighting(깜빡임)이 생긴다.
- `static` 모델이라 링크에 `<inertial>`·`<collision>`이 없어도 된다. 바늘은 충돌이 필요 없다.
- 바늘 색을 **빨강**으로: 원판(흰색)·패널(남색)과 HSV 색상(Hue)으로 확실히 갈린다 → 계획 5번 이진화가 쉬워진다.

#### 각도별로 SDF를 생성하는 스크립트

정답 각도를 바꿔 가며 데이터를 만들려면 손으로 pose를 계산하기보다 스크립트로 만든다.

```python
import math

def needle_pose(phi_deg: float, length: float = 0.10, x: float = 0.045) -> str:
    """φ(12시 기준 시계 방향, 도) → SDF <pose> 문자열."""
    psi = -math.radians(phi_deg)
    h = length / 2
    return f"{x} {-h * math.sin(psi):.4f} {h * math.cos(psi):.4f} {psi:.4f} 0 0"

def value_to_phi(v, v_min=0.0, v_max=10.0, phi_min=225.0, sweep=270.0) -> float:
    return (phi_min + (v - v_min) / (v_max - v_min) * sweep) % 360.0

print(needle_pose(value_to_phi(5.0)))   # 중간값 → φ = 0°(12시) → "0.045 -0.0000 0.0500 -0.0000 0 0"
```

### 4.4 런타임에 바늘 움직이기 (선택)

모델을 바꿀 때마다 시뮬을 재시작하면 느리다. 바늘을 **별도 모델**(`gauge_needle`)로 분리해 월드에 include하면, Gazebo의 `set_pose` 서비스로 실행 중에 각도를 바꿀 수 있다.

```bash
# 예: 안 A(높이 0.7 m)에서 φ = 0° → 바늘 중심은 원판 중심보다 0.05 m 위
gz service -s /world/factory/set_pose \
  --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 \
  --req 'name: "needle_line1", position: {x: 10, y: 8.655, z: 0.75}, orientation: {x: ..., y: ..., z: ..., w: ...}'
```

- 이때 pose는 **월드 좌표**라 게이지 yaw(−90°)와 바늘 roll ψ를 합성한 쿼터니언을 직접 계산해야 한다. 4.3의 모델 내부 링크 방식보다 계산이 번거롭다.
- `static` 모델에도 set_pose가 먹는지는 Docker 환경에서 직접 확인이 필요하다(안 되면 바늘 모델만 `<static>false</static>` + 중력 끄기).
- 순서: 처음엔 4.3 방식(모델 내부 링크 + 스크립트 생성)으로 각도 5개 데이터를 만들고, 반복이 많아지면 이 방식으로 넘어간다.

### 4.5 눈금 넣기

| 방법 | 내용 | 장단점 |
|------|------|--------|
| (가) 텍스처 | 눈금을 그린 PNG를 원판 앞 얇은 박스(두께 0.001)의 `<pbr><metal><albedo_map>`으로 입힘 | 보기 좋음. 원기둥 윗면 UV 매핑은 늘어지기 쉬워 **박스 앞면에** 붙이는 게 안전 |
| (나) 작은 박스 여러 개 | 큰 눈금 11개를 4.3과 같은 식으로 배치 | 텍스처 파일 관리 불필요. SDF가 길어짐(스크립트로 생성) |
| (다) 눈금 없음 | 판독은 φ만으로 하고, 눈금은 보고서 그림용으로만 | 가장 단순. 판독 알고리즘이 눈금에 의존하지 않으면 충분 |

- 판독(계획 5번)은 **바늘 각도 + 미리 정한 φ_min·스윕·값 범위**로 하므로 눈금이 없어도 동작한다. 눈금은 시각적 완성도용이다.
- 텍스처를 쓸 경우 PNG도 OpenCV로 만들 수 있다(`cv2.ellipse`·`cv2.line`·`cv2.putText`로 0~10 눈금) → 계획 5번 단위 테스트의 합성 이미지 코드와 공유 가능.
- 텍스처 경로는 `model://gauge/materials/textures/face.png` 형태. `GZ_SIM_RESOURCE_PATH`에 `simulation/models`가 들어가 있어야 찾는다.

### 4.6 협의 항목 정리

| 대상 | 내용 | 결정 |
|------|------|------|
| 시뮬 담당 | 4.2 가시성 문제와 해결안 A/B/C 중 선택 | |
| 시뮬 담당 | `gauge/model.sdf`에 needle·hub 링크 추가 (4.3), 눈금 방식 (4.5) | |
| 태우 | 정차점 좌표·yaw·dwell(5 s) 유지 여부. 안 B면 정차점 변경 | |
| 수현 | 게이지 값 단위·범위 (예: 0~10 bar), 대시보드 알람 기준 | |
| 공통 | 게이지 사양(φ_min = 225°, 스윕 270°, 값 범위)을 어디에 둘지 → `gauge_ocr/config/gauge_<world>.yaml` (계획 6번) | |

### 4.7 실습 체크리스트

- [ ] 4.2 계산을 직접 재현: `fx`, 반 화각, 정차점별 올려다보는 각
- [ ] 시뮬에서 정차점에 세운 뒤 `/camera/image`에 게이지가 **안 보이는 것**을 실제로 확인 (캡처 남겨 두면 협의 자료가 됨)
- [ ] 로컬에서 월드 z를 0.7로 바꿔 보고 보이는지 확인 → 협의 근거
- [ ] `needle_pose()`로 φ = 0°, 90°, 225°를 만들어 넣고, 화면상 바늘이 12시·3시·7시 반을 가리키는지 확인 (ψ = −φ 부호 검증)
- [ ] 바늘이 원판보다 앞에 보이고 깜빡이지 않는지 확인 (z-fighting)
- [ ] 4.6 협의 결과를 표에 기입

### 4.8 헷갈리기 쉬운 것

- SDF 회전은 **라디안**. `1.57`과 `90`을 헷갈리면 모델이 엉뚱하게 돈다.
- 링크 `<pose>`는 모델 기준, 월드 `<include><pose>`는 월드 기준. 바늘을 모델 안에 넣으면 게이지 yaw는 신경 쓸 필요가 없다.
- 원기둥은 기본적으로 z축이 길이 방향이다. 원판을 벽에 붙이려고 pitch 90°를 준 이유.
- 화면 각도(φ, 시계 방향)·이미지 각도(OpenCV, x축 기준 시계 방향)·수학 각도(반시계)가 모두 다르다. 이 문서는 φ로 통일하고, 계획 5번 코드에서 변환한다.
- STUDY_PLAN 2.3절 표의 "head 링크 기준"은 실제로는 `base_link` 기준이다(`model.sdf`에 head 링크가 따로 없음).

### 4.9 참고

- SDF `<pose>`·`<link>`·`<visual>`·`<material>`: https://sdformat.org/spec?elem=visual
- Gazebo PBR 머티리얼·텍스처: https://gazebosim.org/api/sim/8/pbr.html
- `set_pose` 등 월드 서비스: `gz service -l | grep factory`로 목록 확인, https://gazebosim.org/docs/harmonic/moving_robot

## 5. OpenCV 원 검출 → 바늘 검출 → 각도·값 매핑

> 계획 5번 항목. 학습 후 기입.

## 6. 파라미터화·디버그 토픽·단위 테스트

> 계획 6번 항목. 학습 후 기입.
