# thermal_fusion 학습 내용 정리 (운학)

[STUDY_PLAN.md](STUDY_PLAN.md) 8절 학습 순서의 항목별 학습 내용을 기록한다.
이 패키지의 실제 코드(`thermal_fusion/thermal_fusion_node.py`, `launch/thermal_fusion.launch.py`)를 예제로 삼는다.
ROS 2 기초는 gauge_ocr과 공통이므로 겹치는 부분은 짧게 쓰고, 이 패키지에 특히 필요한 **두 토픽 동시 구독과 시간 동기화**를 더 다룬다.

---

## 0. 실행 환경 — macOS + Docker + Gazebo 확인 창구

gauge_ocr과 완전히 공통이므로 상세는 [gauge_ocr/STUDY.md 0절](../gauge_ocr/STUDY.md#0-실행-환경--macos--docker--gazebo-확인-창구)을 본다. 요점만:

- Mac(Apple Silicon)은 ROS 2 Jazzy·`ros_gz`·`cv_bridge`·`message_filters` 네이티브 불가 → **Docker(Ubuntu 24.04 arm64)**. Jazzy + Harmonic 조합만 arm64 전체 스택이 있다.
- 화면 확인: **Foxglove Studio**(평소, `ws://localhost:8765`) / **noVNC**(Gazebo GUI·`rqt_image_view` 필요 시, `http://localhost:8080/vnc.html`) / **관제 대시보드**(시연).
- 이 패키지에서 특히: Foxglove Image 패널을 두 개 열어 `/camera/image`와 `/thermal/image`를 나란히 보면 화각 차이(80° vs 57°)와 정렬 배율을 눈으로 확인할 수 있다.
- 헤드리스(`gui:=false`)에서도 카메라 센서는 서버 쪽 OGRE2가 CPU(Mesa)로 렌더링하므로 두 이미지 토픽이 나온다. 두 카메라를 CPU로 그리면 RTF가 떨어질 수 있으니 `ros2 topic hz /thermal/image`가 10 Hz 근처인지 확인.
- 열화상을 진짜 thermal 센서(STUDY_PLAN 3절 선택 2)로 바꾸면 `<render_engine>ogre2</render_engine>`가 필요한데, 소프트웨어 렌더링에서도 동작하는지는 Docker 안에서 별도 확인이 필요하다.

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

계획 2번 항목. cv_bridge 기본(변환 함수, 인코딩, 설치, `imwrite` 주의점)은 [gauge_ocr/STUDY.md 2절](../gauge_ocr/STUDY.md#2-cv_bridge로-이미지-받아-png-저장)과 같으므로 여기서는 **이 패키지에서 달라지는 것**만 적는다:
① 열화상은 1채널 `mono8`(나중엔 `mono16`일 수 있음), ② 두 토픽을 **같은 순간의 쌍**으로 저장해야 한다.

### 2.1 두 토픽의 인코딩

| 토픽 | SDF `<format>` | ROS `encoding` | `imgmsg_to_cv2` 인자 | numpy |
|------|----------------|----------------|----------------------|-------|
| `/camera/image` | (기본) R8G8B8 | `rgb8` | `"bgr8"` | `(480, 640, 3) uint8` |
| `/thermal/image` | `L8` | `mono8` | `"mono8"` 또는 `"passthrough"` | `(240, 320) uint8` |
| (thermal 센서로 교체 시) | `L16` | `mono16` | `"passthrough"` (또는 `"mono16"`) | `(240, 320) uint16` |

- 열화상은 `"passthrough"`로 받아도 채널 순서 문제가 없다(1채널). 다만 나중에 `mono16`으로 바뀔 수 있으니 **`msg.encoding`을 보고 분기**하는 습관을 들인다.
- `mono16`을 `"mono8"`로 요청하면 cv_bridge가 상위 비트를 잘라 버려 온도 정보가 뭉개진다. 16비트는 `uint16`으로 받아서 처리하고, 보기 좋게 만들 때만 8비트로 정규화한다.
- 수동 변환(원리 확인)은 1채널이라 더 단순하다: `np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width]`. `mono16`이면 `dtype=np.uint16`, `step == width*2`, `msg.is_bigendian`이 0인지 확인.

### 2.2 PNG로 저장할 때 열화상의 함정

- `cv2.imwrite`는 `uint8` 1채널 → 8비트 그레이 PNG, **`uint16` 1채널 → 16비트 PNG**를 그대로 지원한다. 온도 데이터를 잃지 않고 저장하려면 정규화하지 말고 `uint16` 그대로 쓴다.
- 다시 읽을 때는 `cv2.imread(path, cv2.IMREAD_UNCHANGED)`. 기본 `imread`는 8비트 3채널로 바꿔 버린다.
- 사람이 보기 위한 컬러맵(`cv2.applyColorMap(img8, cv2.COLORMAP_INFERNO)`)은 **보기용 파일을 따로** 만든다. 원본 PNG는 처리용, 컬러맵 PNG는 보고서용으로 이름을 구분한다(`thermal_raw_*.png`, `thermal_view_*.png`).

### 2.3 쌍으로 저장하기 — 최신 RGB 캐시 방식

`message_filters`(1.7절)를 붙이기 전 단계로, 가장 단순한 방법: RGB 콜백은 최신 프레임만 저장해 두고, 열화상 콜백이 올 때 그 프레임과 함께 저장한다.
로봇이 정차한 상태면 두 프레임의 시각 차이(최대 약 0.07 s)는 무시해도 된다.

```python
import os
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data


class ThermalFusionNode(Node):
    def __init__(self) -> None:
        super().__init__("thermal_fusion")
        self.declare_parameter("world", "corridor")
        self.declare_parameter("dump_dir", "")
        self.declare_parameter("dump_period", 2.0)
        self._bridge = CvBridge()
        self._latest_rgb = None          # (stamp, bgr ndarray)
        self._last_dump = None
        self._logged = set()
        self.create_subscription(Image, "/camera/image", self.camera_image_cb, qos_profile_sensor_data)
        self.create_subscription(Image, "/thermal/image", self.thermal_image_cb, qos_profile_sensor_data)
        ...

    def _log_once(self, name: str, msg: Image) -> None:
        if name not in self._logged:
            self.get_logger().info(f"{name} {msg.width}x{msg.height} {msg.encoding} step={msg.step}")
            self._logged.add(name)

    def camera_image_cb(self, msg: Image) -> None:
        self._log_once("camera", msg)
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().warn(f"cv_bridge(rgb): {e}"); return
        self._latest_rgb = (msg.header.stamp, bgr)

    def thermal_image_cb(self, msg: Image) -> None:
        self._log_once("thermal", msg)
        try:
            th = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")   # mono8 → uint8, mono16 → uint16
        except CvBridgeError as e:
            self.get_logger().warn(f"cv_bridge(thermal): {e}"); return
        if self._latest_rgb is None:
            return                                          # 아직 RGB가 안 들어옴
        rgb_stamp, bgr = self._latest_rgb
        self._maybe_dump_pair(msg, bgr, th, rgb_stamp)

    def _maybe_dump_pair(self, th_msg: Image, bgr, th, rgb_stamp) -> None:
        dump_dir = self.get_parameter("dump_dir").value
        if not dump_dir:
            return
        now = self.get_clock().now()
        period = Duration(seconds=self.get_parameter("dump_period").value)
        if self._last_dump is not None and now - self._last_dump < period:
            return
        os.makedirs(dump_dir, exist_ok=True)
        s = th_msg.header.stamp
        tag = f"{s.sec}_{s.nanosec:09d}"                    # 열화상 스탬프를 쌍의 이름으로
        dt_ms = ((s.sec - rgb_stamp.sec) * 1e9 + (s.nanosec - rgb_stamp.nanosec)) / 1e6
        ok_rgb = cv2.imwrite(os.path.join(dump_dir, f"rgb_{tag}.png"), bgr)
        ok_th = cv2.imwrite(os.path.join(dump_dir, f"thermal_raw_{tag}.png"), th)   # uint8/uint16 그대로
        view = th if th.dtype == np.uint8 else cv2.normalize(th, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        cv2.imwrite(os.path.join(dump_dir, f"thermal_view_{tag}.png"), cv2.applyColorMap(view, cv2.COLORMAP_INFERNO))
        self.get_logger().info(f"pair {tag} (rgb-thermal dt={dt_ms:+.1f} ms) ok={ok_rgb and ok_th}")
        self._last_dump = now
```

알아둘 점:
- 파일명은 **열화상 스탬프** 기준으로 통일한다. 열화상이 10 Hz로 더 느리므로 이쪽이 쌍의 기준이 된다.
- 로그의 `dt`가 ±70 ms를 넘게 계속 나오면 QoS로 프레임이 밀리고 있거나 시계가 다른 것이다. 이 값은 1.7절 `slop` 결정의 근거가 된다.
- 계획 6번에서 `message_filters`로 바꾸면 `pair_cb(rgb, th)` 안에서 같은 `_maybe_dump_pair`를 부르면 된다. 저장 함수는 그대로 재사용된다.

### 2.4 두 장을 한눈에 비교하는 미리보기 (선택)

정렬 작업(계획 5번) 전에 화각 차이(80° vs 57°)를 눈으로 확인해 두면 좋다. 열화상을 RGB 높이에 맞춰 키운 뒤 옆으로 붙인다.

```python
view_bgr = cv2.cvtColor(view, cv2.COLOR_GRAY2BGR)              # 1채널 → 3채널 (hconcat은 채널 수가 같아야 함)
view_bgr = cv2.resize(view_bgr, (int(320 * 480 / 240), 480))    # 높이를 480에 맞춤 → 640×480
side = cv2.hconcat([bgr, view_bgr])                             # 1280×480
cv2.imwrite(os.path.join(dump_dir, f"pair_{tag}.png"), side)
```

이 그림에서 열화상 중앙의 물체가 RGB에서는 더 작게(약 1/1.3배) 보이는 것이 정렬 배율의 출발점이다(STUDY_PLAN 4.2절).

### 2.5 실습 체크리스트

- [ ] `package.xml`에 `cv_bridge`, `python3-opencv`, `python3-numpy` 추가 → 빌드
- [ ] 첫 프레임 로그: `camera 640x480 rgb8 step=1920`, `thermal 320x240 mono8 step=320` 확인
- [ ] 열화상을 `"passthrough"`로 받은 배열의 `dtype`·`shape`이 `uint8 (240, 320)`인지 확인
- [ ] `dump_dir:=/workspace/docs/captures`로 실행해 `rgb_*`, `thermal_raw_*`, `thermal_view_*` 세 파일이 같은 태그로 생기는지 확인
- [ ] `thermal_raw_*.png`를 `cv2.imread(path, cv2.IMREAD_UNCHANGED)`로 읽어 원본과 `np.array_equal`인지 확인
- [ ] 로그의 `dt`가 대략 −70 ~ +70 ms 안에 들어오는지 확인 → 1.7절 `slop=0.1` 근거
- [ ] 로봇을 factory `gas_tank` 정차점 (8, −5)에 두고 가스탱크가 두 이미지 모두에 들어오는 쌍 확보 → 계획 5번 첫 입력. 열화상 화각이 좁아 안 들어오면 STUDY_PLAN 6절 협의 항목
- [ ] (선택) `pair_*.png` 미리보기로 화각 차이 확인

### 2.6 헷갈리기 쉬운 것

- 열화상 `mono8` 배열을 `bgr` 이미지와 `hconcat`·`addWeighted`하면 채널 수가 달라 에러. `cvtColor(..., COLOR_GRAY2BGR)` 먼저.
- `mono16`을 `imwrite`할 때 `astype(np.uint8)`로 캐스팅하면 상위 비트가 잘려 엉뚱한 그림이 된다. 보기용은 `cv2.normalize` 또는 `(img / 256).astype(np.uint8)`.
- `_latest_rgb`를 세팅하기 전에 열화상 콜백이 먼저 오면 `None` → 위 코드처럼 걸러야 한다.
- 두 구독 중 하나만 `qos_profile_sensor_data`로 바꾸면 한쪽만 밀려 `dt`가 계속 커진다. 둘 다 바꾼다.
- 저장 경로·시뮬 시계·`imwrite` 반환값 관련 주의점은 gauge_ocr 2.5·2.9절과 동일.

### 2.7 참고

- gauge_ocr/STUDY.md 2절 (cv_bridge 기본, 설치, 수동 변환, Docker 저장 경로)
- cv_bridge: https://docs.ros.org/en/jazzy/p/cv_bridge/
- OpenCV 16비트 PNG·`IMREAD_UNCHANGED`: https://docs.opencv.org/4.x/d4/da8/group__imgcodecs.html
- 컬러맵 종류: https://docs.opencv.org/4.x/d3/d50/group__imgproc__colormap.html

## 3. 열화상 센서 방향 결정 + 발열체 모델 협의

> 계획 3번 항목. 협의 후 기입.

## 4. rosbag 녹화 + 동일 스탬프 PNG 쌍 확보

> 계획 4번 항목. 학습 후 기입.

## 5. 정렬 → 임계 검출 → 오버레이

> 계획 5번 항목. 학습 후 기입.

## 6. message_filters 동기화 노드화·파라미터화·단위 테스트

> 계획 6번 항목. 학습 후 기입.
