# thermal_fusion 학습 내용 정리 (운학)

[STUDY_PLAN.md](STUDY_PLAN.md) 8절 학습 순서의 항목별 학습 내용을 기록한다.
이 패키지의 실제 코드(`thermal_fusion/thermal_fusion_node.py`, `launch/thermal_fusion.launch.py`)를 예제로 삼는다.
ROS 2 기초는 gauge_ocr과 공통이므로 겹치는 부분은 짧게 쓰고, 이 패키지에 특히 필요한 **두 토픽 동시 구독과 시간 동기화**를 더 다룬다.

---

## 1. ROS 2 Python 기초 — 노드·토픽·파라미터·런치

### 1.1 핵심 개념

| 용어 | 뜻 | 이 패키지에서 |
|------|-----|--------------|
| **노드(Node)** | 하나의 실행 단위(프로세스). 이름을 갖고 토픽·파라미터를 소유 | `thermal_fusion` 노드 하나 |
| **토픽(Topic)** | 이름 붙은 데이터 통로. 발행자가 쓰고 구독자가 읽음. N:M 가능 | 구독 `/camera/image`, `/thermal/image`, 발행 `/inspection/thermal_alert` |
| **메시지(Message)** | 토픽 데이터의 타입. `패키지/msg/이름` | `sensor_msgs/msg/Image`, `std_msgs/msg/Bool` |
| **파라미터(Parameter)** | 노드 설정값. 런치·YAML·CLI로 변경 | `world`, 앞으로 추가할 `hot_threshold`, `min_area_px` 등 |
| **런치(Launch)** | 노드들을 인자·파라미터와 함께 띄우는 파이썬 스크립트 | `launch/thermal_fusion.launch.py` |
| **패키지(Package)** | 빌드·배포 단위. `package.xml` + `setup.py` | `thermal_fusion` |
| **워크스페이스** | 패키지들을 모아 `colcon build` 하는 디렉터리 | 저장소 루트 |

`/camera/image`는 gauge_ocr 노드도 같이 구독한다. 토픽은 N:M이라 두 노드가 같은 토픽을 구독해도 서로 영향이 없다.

### 1.2 워크스페이스와 빌드

```bash
source /opt/ros/jazzy/setup.bash
cd <저장소 루트>
colcon build --symlink-install --packages-select thermal_fusion
source install/setup.bash
```

- `--symlink-install`: 파이썬 파일 수정은 재빌드 없이 반영. `setup.py`·launch·config 변경은 재빌드 필요.
- 패키지 구조는 gauge_ocr과 동일 (`package.xml`, `setup.py`, `resource/`, `thermal_fusion/`, `launch/`).
- `setup.py` `entry_points`의 `thermal_fusion_node = thermal_fusion.thermal_fusion_node:main`이 `ros2 run thermal_fusion thermal_fusion_node`를 만든다.

### 1.3 노드 구조 (현재 코드 해설)

```python
class ThermalFusionNode(Node):
    def __init__(self) -> None:
        super().__init__("thermal_fusion")
        self.create_subscription(Image, "/camera/image", self.camera_image_cb, 10)
        self.create_subscription(Image, "/thermal/image", self.thermal_image_cb, 10)
        self.pub_0 = self.create_publisher(Bool, "/inspection/thermal_alert", 10)
        self.create_timer(0.5, self._tick)
        self.get_logger().info("thermal_fusion started (운학)")

    def camera_image_cb(self, msg: Image) -> None:
        del msg
    def thermal_image_cb(self, msg: Image) -> None:
        del msg

    def _tick(self) -> None:
        out = Bool(); out.data = False; self.pub_0.publish(out)
```

알아둘 점:
- 구독이 두 개지만 **콜백은 따로따로** 온다. RGB는 15 Hz, 열화상은 10 Hz라 같은 순간의 프레임 쌍이 자동으로 묶이지 않는다. → 1.7 동기화 참고.
- `spin`은 단일 스레드. 두 콜백과 타이머가 한 스레드에서 번갈아 실행되므로 콜백 안에서 무거운 처리를 하면 다른 콜백이 밀린다.
- 현재는 이미지가 안 와도 0.5 s마다 False를 발행한다. 실제 구현에서는 "최근 판정 결과"를 멤버 변수에 두고 타이머에서 발행하는 구조가 자연스럽다.

### 1.4 메시지 타입

```bash
ros2 interface show sensor_msgs/msg/Image
ros2 interface show std_msgs/msg/Bool
```

`sensor_msgs/msg/Image`에서 이 패키지가 특히 봐야 할 것:

| 필드 | RGB (`/camera/image`) | 열화상 (`/thermal/image`) |
|------|------|------|
| `width × height` | 640 × 480 | 320 × 240 |
| `encoding` | `rgb8` (예상) | `mono8` (L8). 진짜 thermal 센서로 바꾸면 `mono16` |
| `step` | 640 × 3 = 1920 | 320 × 1 = 320 (mono16이면 640) |
| `header.stamp` | 촬영 시각. 동기화 기준 | 동일 |

두 토픽의 `encoding`을 콜백에서 확인하는 것이 첫 실습이다. 열화상이 `mono8`인지 `mono16`인지에 따라 이후 처리 코드가 달라진다.

### 1.5 QoS

```python
from rclpy.qos import qos_profile_sensor_data

self.create_subscription(Image, "/camera/image", self.camera_image_cb, qos_profile_sensor_data)
self.create_subscription(Image, "/thermal/image", self.thermal_image_cb, qos_profile_sensor_data)
```

- QoS가 안 맞으면 연결이 조용히 실패한다. 이미지가 안 오면 `ros2 topic info -v /thermal/image`로 발행자 QoS를 확인.
- 센서 구독은 BEST_EFFORT(`qos_profile_sensor_data`)가 안전하다. RELIABLE 발행자 ↔ BEST_EFFORT 구독자는 연결된다.
- 이미지 두 개를 RELIABLE + depth 10으로 받으면 큐가 쌓여 지연이 커진다. 정렬·융합은 최신 프레임만 필요하므로 depth를 작게 둔다.

### 1.6 파라미터

```python
self.declare_parameter("world", "corridor")
self.declare_parameter("hot_threshold", 200)      # int: mono8 밝기 기준
self.declare_parameter("min_area_px", 50)
self.declare_parameter("scale", 1.30)             # float: 열화상→RGB 배율
world = self.get_parameter("world").value
```

- 런치가 `world`를 넘기지만 노드가 선언하지 않아 지금은 무시되고 있다.
- 타입은 기본값으로 고정된다. `200`(int)로 선언하면 `ros2 param set ... 200.5`는 에러. 온도(℃)로 바꿀 가능성이 있으면 처음부터 `200.0`(float)로 선언하는 편이 낫다.
- 실행 중 튜닝:

```bash
ros2 param set /thermal_fusion hot_threshold 180.0
```

타이머나 콜백에서 매번 `get_parameter`로 읽으면 즉시 반영된다.
- YAML(`config/thermal.yaml`):

```yaml
thermal_fusion:
  ros__parameters:
    hot_threshold: 200.0
    min_area_px: 50
    scale: 1.30
```

`setup.py` `data_files`에 `config/*.yaml` 항목 추가 필요 (patrol_path 패키지 참고).

### 1.7 두 토픽 동기화 — `message_filters`

이 패키지의 핵심. 두 콜백이 따로 오는 문제를 `ApproximateTimeSynchronizer`로 푼다.

```python
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.qos import qos_profile_sensor_data

rgb_sub = Subscriber(self, Image, "/camera/image", qos_profile=qos_profile_sensor_data)
th_sub = Subscriber(self, Image, "/thermal/image", qos_profile=qos_profile_sensor_data)
self._sync = ApproximateTimeSynchronizer([rgb_sub, th_sub], queue_size=5, slop=0.1)
self._sync.registerCallback(self.pair_cb)

def pair_cb(self, rgb: Image, th: Image) -> None:
    # 스탬프 차이가 slop(0.1 s) 이내인 쌍만 여기로 온다
    ...
```

- `slop`: 허용 시각 차이(초). 15 Hz와 10 Hz면 최대 차이가 약 0.067 s이므로 0.1이면 충분.
- `queue_size`: 짝을 기다리며 보관할 메시지 수.
- `use_sim_time`이 켜져 있고 두 센서가 같은 시뮬 시계로 스탬프를 찍으므로 시뮬에서는 잘 맞는다. 스탬프가 0이거나 안 맞으면 짝이 안 만들어지고 콜백이 한 번도 안 온다 → `ros2 topic echo --field header.stamp /thermal/image`로 확인.
- 대안(단순): RGB 콜백은 최신 프레임만 멤버에 저장하고, 열화상 콜백에서 그 프레임과 융합한다. 정차 중 판정이면 이 방식으로도 충분하다. 처음엔 이 방식으로 시작하고 나중에 `message_filters`로 바꿔도 된다.
- 의존성: `package.xml`에 `<exec_depend>message_filters</exec_depend>`. 설치는 `ros-jazzy-message-filters` (desktop에 보통 포함).

### 1.8 런치 파일

```python
DeclareLaunchArgument("world", default_value="corridor"),
Node(
    package="thermal_fusion",
    executable="thermal_fusion_node",
    name="thermal_fusion",
    output="screen",
    parameters=[{"world": LaunchConfiguration("world"), "use_sim_time": True}],
),
```

- `use_sim_time: True`: `/clock` 기준 시계. 동기화가 스탬프에 의존하므로 반드시 켜져 있어야 한다.
- YAML 추가 시 `parameters=[config_path, {...}]`. `config_path`는 `os.path.join(get_package_share_directory("thermal_fusion"), "config", "thermal.yaml")`.
- 통합 런치 `simulation/launch/full_system.launch.py`가 이 파일을 포함하므로 여기 수정이 통합 실행에 반영된다.

### 1.9 자주 쓰는 CLI

```bash
ros2 node info /thermal_fusion
ros2 topic hz /camera/image            # 15 Hz
ros2 topic hz /thermal/image           # 10 Hz
ros2 topic info -v /thermal/image      # QoS
ros2 topic echo --field header.stamp /thermal/image
ros2 topic echo /inspection/thermal_alert
ros2 param list /thermal_fusion
ros2 run thermal_fusion thermal_fusion_node
ros2 launch thermal_fusion thermal_fusion.launch.py world:=factory
```

### 1.10 실습 체크리스트

- [ ] 패키지 단독 빌드 후 `ros2 run thermal_fusion thermal_fusion_node`, `ros2 topic echo /inspection/thermal_alert`로 False 확인
- [ ] 시뮬 기동 후 두 이미지 토픽의 `hz`와 QoS 확인
- [ ] 두 콜백에서 각각 `encoding`, `width`, `height`, `step`을 1회 로그 출력 → 열화상이 `mono8`인지 확인
- [ ] 두 토픽의 `header.stamp` 차이를 로그로 찍어 동기화 가능 여부 확인
- [ ] `world`, `hot_threshold` 파라미터 선언, `ros2 param set`으로 실행 중 변경 확인
- [ ] 구독 QoS를 `qos_profile_sensor_data`로 변경
- [ ] `message_filters`로 두 토픽 묶어 `pair_cb`가 약 10 Hz로 호출되는지 확인 (열화상 주기에 맞춰짐)
- [ ] `config/thermal.yaml` 만들고 `setup.py` data_files + 런치 `parameters`에 연결

### 1.11 헷갈리기 쉬운 것

- 새 터미널마다 `source /opt/ros/jazzy/setup.bash` + `source install/setup.bash`.
- 동기화 콜백이 한 번도 안 오면: (1) 둘 중 하나가 안 들어옴(QoS), (2) 스탬프가 0 또는 서로 다른 시계, (3) `slop`이 너무 작음 순으로 의심.
- 콜백 안에서 `cv2.imshow`/`cv2.waitKey`를 쓰면 spin이 막힌다. 확인은 `rqt_image_view`나 PNG 저장으로.
- `Bool` 메시지에 numpy bool(`np.bool_`)을 넣으면 타입 에러가 날 수 있다. `bool(...)`로 감싼다.
- 노드 이름(`thermal_fusion`), 패키지 이름(`thermal_fusion`), 실행 파일 이름(`thermal_fusion_node`)은 서로 다른 것.

### 1.12 참고

- ROS 2 Jazzy 튜토리얼: https://docs.ros.org/en/jazzy/Tutorials.html
- rclpy API: https://docs.ros.org/en/jazzy/p/rclpy/
- QoS: https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html
- message_filters: https://docs.ros.org/en/jazzy/p/message_filters/
- 런치: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html

---

## 2. cv_bridge로 두 이미지 토픽 받아 PNG 저장

> 계획 2번 항목. 학습 후 기입.

## 3. 열화상 센서 방향 결정 + 발열체 모델 협의

> 계획 3번 항목. 협의 후 기입.

## 4. rosbag 녹화 + 동일 스탬프 PNG 쌍 확보

> 계획 4번 항목. 학습 후 기입.

## 5. 정렬 → 임계 검출 → 오버레이

> 계획 5번 항목. 학습 후 기입.

## 6. message_filters 동기화 노드화·파라미터화·단위 테스트

> 계획 6번 항목. 학습 후 기입.
