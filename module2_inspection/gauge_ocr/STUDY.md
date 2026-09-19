# gauge_ocr 학습 내용 정리 (운학)

[STUDY_PLAN.md](STUDY_PLAN.md) 8절 학습 순서의 항목별 학습 내용을 기록한다.
이 패키지의 실제 코드(`gauge_ocr/gauge_ocr_node.py`, `launch/gauge_ocr.launch.py`)를 예제로 삼는다.

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

> 계획 2번 항목. 학습 후 기입.

## 3. rosbag 녹화와 오프라인 데이터셋

> 계획 3번 항목. 학습 후 기입.

## 4. 게이지 모델 보강

> 계획 4번 항목. 협의 후 기입.

## 5. OpenCV 원 검출 → 바늘 검출 → 각도·값 매핑

> 계획 5번 항목. 학습 후 기입.

## 6. 파라미터화·디버그 토픽·단위 테스트

> 계획 6번 항목. 학습 후 기입.
