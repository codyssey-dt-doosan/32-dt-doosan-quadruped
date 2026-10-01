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

계획 3번 항목. STUDY_PLAN 3절의 선택지 (1) 흑백 카메라 유지 / (2) Gazebo 열화상 센서 중 하나를 고르고, **무엇이 뜨거운지**(발열체)와 **어디서 보는지**(정차점)를 시뮬 담당·태우·채현과 정한다.
이 결정에 따라 이후 코드의 입력 형식(`mono8` 밝기 vs `mono16` 켈빈)이 달라지므로 5번 구현 전에 끝내야 한다.

### 3.1 지금 `/thermal/image`의 실체

`simulation/models/go2/model.sdf` 81행:

```xml
<sensor name="thermal" type="camera">        <!-- type이 thermal이 아니라 camera -->
  <pose>0.28 0 0.02 0 0 0</pose>
  <update_rate>10</update_rate>
  <topic>thermal</topic>
  <camera>
    <horizontal_fov>1.0</horizontal_fov>
    <image><width>320</width><height>240</height><format>L8</format></image>
```

- 일반 카메라가 장면을 렌더링한 뒤 **흑백으로 바꾼 것**이다. 픽셀값 = 밝기(조명 × 재질 색), 온도와 무관하다.
- 예: 가스탱크 재질은 빨강(diffuse 0.75, 0.18, 0.14) → 밝기 ≈ 0.3·0.75 + 0.59·0.18 + 0.11·0.14 ≈ 0.34 → 픽셀 약 90. 반대로 **흰 게이지 원판이 가장 "뜨겁게"** 나온다.
- 즉 지금 상태로 임계값 검출을 하면 "흰 물체 검출기"가 된다.

### 3.2 두 선택지 비교

| 항목 | (1) 흑백 카메라 유지 | (2) Gazebo 열화상 센서 |
|------|----------------------|------------------------|
| 센서 SDF | 그대로 | `type="thermal"`로 변경 |
| 발열체 표현 | 재질을 흰색·`<emissive>`로 밝게 | visual에 `Thermal` 시스템 플러그인 + `<temperature>` (켈빈) |
| 픽셀 의미 | 밝기 0~255 | 온도 (L16: 켈빈 × 100, L8: 지정 범위로 양자화) |
| 오탐 | 흰 벽·원판·조명 반사도 "뜨거움" | 온도를 지정한 물체만 뜨거움 |
| 보고서 표기 | "밝기 200 이상" | **"최고 87 ℃, 임계 70 ℃"** |
| 렌더링 | 일반 카메라 | ogre2 필요 (factory·corridor 월드 모두 이미 `<render_engine>ogre2</render_engine>`) |
| 위험 | 없음 | Docker CPU 렌더링(Mesa)에서 thermal 센서가 정상 출력되는지 **미확인** |

**결정 방침: (2)를 시도하고, Docker에서 안 되면 (1)로 대체한다.**
코드는 두 경우 모두 돌도록 "픽셀값 → 온도(또는 밝기) 배열"을 바꾸는 함수 하나로 입력을 추상화한다(3.5절). 그러면 이후 정렬·검출 코드는 그대로다.

### 3.3 (2) Gazebo 열화상 센서 설정

센서 쪽 (`go2/model.sdf`의 `thermal` 센서 교체):

```xml
<sensor name="thermal" type="thermal">
  <pose>0.28 0 0.02 0 0 0</pose>
  <always_on>true</always_on>
  <update_rate>10</update_rate>
  <topic>thermal</topic>                       <!-- 그대로 두면 브리지 설정 변경 불필요 -->
  <camera>
    <horizontal_fov>1.0</horizontal_fov>
    <image><width>320</width><height>240</height><format>L16</format></image>
    <clip><near>0.05</near><far>15</far></clip>
  </camera>
</sensor>
```

발열체 쪽 (뜨겁게 할 visual 안에):

```xml
<visual name="visual">
  <geometry>...</geometry>
  <material>...</material>
  <plugin filename="gz-sim-thermal-system" name="gz::sim::systems::Thermal">
    <temperature>358.15</temperature>          <!-- 켈빈. 85 ℃ -->
  </plugin>
</visual>
```

픽셀 → 온도:

| `<format>` | ROS `encoding` | 변환 | 비고 |
|------------|----------------|------|------|
| `L16` | `mono16` | `T[K] = 픽셀 × 0.01` (기본 해상도 0.01 K) | 0.01 K 단위, 범위 넉넉. **권장** |
| `L8` | `mono8` | `T[K] = min_temp + 픽셀 × resolution` | 센서에 `gz-sim-thermal-sensor-system` 플러그인으로 `min_temp`·`max_temp`·`resolution` 지정 |

- 온도를 지정하지 않은 물체는 **주변 온도**로 렌더링된다. 월드의 `<atmosphere>` 설정에 따른다.
- 위 플러그인 이름·변환식은 Gazebo Harmonic 예제 월드(`gz-sim` 저장소 `examples/worlds/thermal_camera.sdf`)를 기준으로 정리했다. 실제 버전에서 그대로 되는지는 **Docker 안에서 예제 월드를 먼저 띄워 확인**한다: `gz sim -s -r thermal_camera.sdf` → `gz topic -e -t /thermal_camera --json-output | head`로 픽셀값 확인.
- 브리지(`ros_gz_bridge.yaml`)는 타입이 같은 `gz.msgs.Image`라 바꿀 필요가 없다. 인코딩만 `mono8` → `mono16`으로 바뀐다(2.1절 표).

### 3.4 어디서 보나: 가스탱크 정차점 기하

열화상 화각: 수평 반각 = 0.5 rad ≈ **28.6°**, 수직 반각 = `atan(120 / 293)` ≈ **22.3°** (`fx = 160 / tan(0.5) ≈ 293 px`).
카메라 높이는 스폰 높이 0.4 + 0.02 = 약 0.42 m. 가스탱크 `gas_tank_1`은 (8, −6), 반지름 0.35 m, 높이 0 ~ 1.5 m.

| 정차점 y (태우 웨이포인트) | 탱크 표면까지 거리 | 열화상에 보이는 폭 | 보이는 높이 범위 | 탱크 폭(열화상 px) | 판정 |
|---------------------------|-------------------|--------------------|------------------|--------------------|------|
| **−5.0 (현재)** | 0.37 m | 0.40 m | 0.27 ~ 0.57 m | 554 px (화면 320 px) | 탱크가 화면을 **꽉 채움** |
| −4.0 | 1.37 m | 1.50 m | −0.14 ~ 0.98 m | 150 px | 탱크 전체 폭 + 바닥이 보임 |
| −3.5 | 1.87 m | 2.04 m | −0.35 ~ 1.19 m | 110 px | 여유 있음, 발열체는 작아짐 |

계산: 거리 = (정차점 y − 0.28) − (−6 + 0.35), 보이는 폭 = 2 · 거리 · tan 28.6°.

- 현재 정차점에서는 화면 전체가 탱크 표면 한 조각이다. 탱크 전체를 뜨겁게 하면 **화면 전체가 과열**, 아니면 아무것도 없음 → "과열 영역을 찾아 표시"가 성립하지 않는다.
- 정차점을 **y = −4.0** 정도로 물리면 탱크와 주변이 함께 보여 "어디가 뜨거운지"를 보여 줄 수 있다.
- 단, `gas_tank` 정차점은 **가스 측정(채현)도 쓰는 지점**(dwell 8 s)이다. 거리를 늘리면 가스 센서 값이 달라질 수 있으니 채현과 같이 정한다. 합의가 안 되면 열화상 전용 정차점을 하나 추가하는 안도 있다.

### 3.5 발열체 정하기

| 안 | 내용 | 장단점 |
|----|------|--------|
| 가. 탱크 전체 가열 | `gas_tank/model.sdf` visual에 온도 지정 | 간단. 그러나 모델을 공유하는 `gas_tank_2`도 같이 뜨거워짐 |
| **나. 과열 부품 별도 모델** (권장) | `hot_valve` 같은 작은 모델(예: 0.15 × 0.08 × 0.15 m 박스)을 `gas_tank_1` 앞면 (8, −5.64, 0.5)에 include, 85 ℃ | 과열 "영역"이 생김. 탱크 본체는 30 ℃ 정도로 두면 "주변보다 뜨거운 곳" 검출이 자연스럽다. `gas_tank_2`는 정상 대조군 |
| 다. 배관·모터 신규 모델 | 공장 라인 옆에 과열 모터 추가 | 시나리오는 좋지만 정차점·순찰 경로 추가 필요 |

나안의 높이 0.5 m는 3.4절 표에서 y = −4.0일 때 보이는 범위(−0.14 ~ 0.98 m) 안이다.

온도 설정 예 (보고서·임계값의 근거가 되므로 표로 남긴다):

| 물체 | 온도 | 의도 |
|------|------|------|
| 주변(지정 안 함) | 약 20 ℃ | 배경 |
| 탱크 본체 | 30 ℃ (303.15 K) | 약간 따뜻한 정상 설비 |
| 과열 밸브 | 85 ℃ (358.15 K) | 과열 |
| 알람 임계 | 60 ℃ 켜짐 / 55 ℃ 꺼짐 | 5번 히스테리시스 |

(1)로 대체할 경우: 과열 밸브만 흰색 + `<emissive>1 1 1 1</emissive>`로 두고, **게이지 원판처럼 흰 물체는 정차점 화면에 들어오지 않는지** 확인한다.

입력 추상화 함수 (5·6번 코드는 이 함수 출력만 쓴다):

```python
import numpy as np

def to_celsius(img: np.ndarray, encoding: str, l8_min_k: float = 253.15, l8_res: float = 3.0) -> np.ndarray:
    """열화상 픽셀 → ℃ (float32). (1) 흑백 카메라면 밝기를 그대로 돌려준다(단위 없음)."""
    if encoding == "mono16":                       # (2) L16: 0.01 K 단위
        return img.astype(np.float32) * 0.01 - 273.15
    if encoding == "mono8_thermal":                # (2) L8 + ThermalSensor 플러그인
        return l8_min_k + img.astype(np.float32) * l8_res - 273.15
    return img.astype(np.float32)                  # (1) mono8 밝기
```

- 센서가 L8 열화상인지 일반 흑백인지는 ROS 메시지 인코딩(`mono8`)만으로 구별이 안 된다. 그래서 `"mono8_thermal"`은 노드 파라미터(`thermal_mode`)로 정해 넘긴다.

### 3.6 협의 항목 정리

| 대상 | 내용 | 결정 |
|------|------|------|
| 시뮬 담당 | 선택 (1)/(2), `thermal` 센서 `type="thermal"` + L16 교체 | |
| 시뮬 담당 | 발열체 안(가/나/다), `hot_valve` 모델 추가, 온도 표 | |
| 시뮬 담당 | corridor 월드에도 발열체를 둘지 (현재 corridor에는 열화상 대상이 없음 → 항상 정상) | |
| 태우 + 채현 | `gas_tank` 정차점 y −5.0 → −4.0 변경 또는 열화상 전용 정차점 추가 | |
| 수현 | 대시보드에 최고 온도(℃) 표시 추가 여부 | |

### 3.7 실습 체크리스트

- [ ] 현재 `/thermal/image`에서 흰 물체(게이지 원판)가 가장 밝게 나오는 것을 확인 → (1)의 한계를 눈으로 확인
- [ ] Docker 안에서 Gazebo 예제 `thermal_camera.sdf`를 헤드리스로 띄워 열화상 토픽이 나오는지 확인 → (2) 가능 여부 결정
- [ ] (2) 가능하면 로컬에서 `thermal` 센서를 L16으로 바꾸고 `ros2 topic echo --once --field encoding /thermal/image`가 `mono16`인지 확인
- [ ] 발열체에 온도를 주고, 그 픽셀값 × 0.01 − 273.15가 지정 온도와 맞는지 확인
- [ ] 3.4절 계산 재현, 현재 정차점에서 열화상이 탱크로 꽉 차는 것을 캡처로 확인 → 협의 자료
- [ ] 3.6절 협의 결과 기입

### 3.8 헷갈리기 쉬운 것

- `<format>L8</format>`이라고 열화상이 아니다. `type="thermal"`이어야 온도를 렌더링한다.
- 온도 단위는 **켈빈**. 85 ℃ = 358.15 K. SDF에 85를 넣으면 −188 ℃짜리 물체가 된다.
- L16 원본을 `"mono8"`로 받거나 `uint8`로 캐스팅하면 온도가 뭉개진다(2.1·2.6절). 처리 전에 `to_celsius`로 float 배열로 바꾼다.
- 모델 파일을 고치면 그 모델을 include한 **모든 인스턴스**가 바뀐다(`gas_tank_1`, `gas_tank_2`). 하나만 바꾸려면 별도 모델.
- 정차점 변경은 다른 사람 모듈(patrol_path·gas)에 영향을 준다. 혼자 고치지 않는다.

### 3.9 참고

- Gazebo Thermal Camera 튜토리얼: https://gazebosim.org/api/sim/8/thermalcameraigngazebo.html
- 예제 월드: https://github.com/gazebosim/gz-sim/tree/gz-sim8/examples/worlds (`thermal_camera.sdf`)
- SDF `<sensor>` 타입 목록: https://sdformat.org/spec?elem=sensor
- 실제 열화상 카메라의 radiometric 출력(참고): FLIR "radiometric" 개념

## 4. rosbag 녹화 + 동일 스탬프 PNG 쌍 확보

계획 4번 항목. rosbag 기본(명령, MCAP, `/clock` 재생 문제, `rosbag2_py` 읽기)은 [gauge_ocr/STUDY.md 3절](../gauge_ocr/STUDY.md#3-rosbag-녹화와-오프라인-데이터셋)과 같다.
여기서는 이 패키지에서 달라지는 것만 다룬다: ① 녹화할 토픽과 용량, ② **두 카메라 프레임을 시각 기준으로 짝짓는 법**, ③ 열화상 정답 라벨.

### 4.1 녹화

```bash
ros2 bag record -o /workspace/bags/factory_thermal_01 \
  /clock /camera/image /camera/camera_info /thermal/image /odom /tf /mission/status
```

| 토픽 | 크기 | 초당 |
|------|------|------|
| `/camera/image` 640×480×3 B, 15 Hz | 0.92 MB | 13.8 MB/s |
| `/thermal/image` 320×240×1 B (`mono8`), 10 Hz | 0.077 MB | 0.77 MB/s |
| `/thermal/image` 320×240×2 B (`mono16`), 10 Hz | 0.15 MB | 1.5 MB/s |

- RGB가 용량 대부분이다. gauge와 마찬가지로 **`gas_tank` 정차 구간(dwell 8 s)만** 녹화한다.
- 3절에서 센서를 `mono16`으로 바꾸면 그 전에 녹화한 bag은 `mono8`이다. bag 이름에 센서 모드를 넣는다(`factory_thermal_L16_01`).
- `/thermal/camera_info`는 브리지에 없다(STUDY_PLAN 6절). 열화상 내부 파라미터는 SDF에서 계산한 값(`fx ≈ 293`, `cx = 160`, `cy = 120`)을 쓴다.

### 4.2 "같은 순간"이란 — 두 센서의 시각 관계

두 센서는 같은 Gazebo 시뮬 시계로 스탬프를 찍는다. 15 Hz와 10 Hz이므로:
- 0.2초마다(15와 10의 공배수) 두 센서가 **정확히 같은 시각**에 찍힌다.
- 그 사이의 열화상 프레임은 가장 가까운 RGB와 **1/30 ≈ 0.033 s** 차이가 난다.
- 따라서 짝짓기 허용 오차 `max_dt = 0.05 s`면 모든 열화상 프레임이 짝을 찾는다. 0.033보다 작게 잡으면 0.2초마다만 짝이 생긴다(5 Hz).

로봇이 정차해 있으면 0.033 s 차이는 영상에 아무 차이도 없다. 이동 중 데이터라면 정확히 같은 시각 쌍만 쓰는 편이 정렬 검증에 깨끗하다.

실제 Gazebo는 렌더링이 밀리면 센서 업데이트를 건너뛰기도 하므로, 위 관계를 가정하지 말고 **추출 로그의 dt 분포를 보고** `max_dt`를 정한다. 이 값은 6번의 `ApproximateTimeSynchronizer(slop=...)` 근거가 된다(2.3절 노드 로그의 `dt`와 같은 값).

### 4.3 bag에서 쌍 추출 — 가장 가까운 시각 짝짓기

bag 메시지는 **기록 순서**(수신 시각)로 나온다. 열화상 프레임을 받았을 때 그 직전 RGB가 가장 가깝다는 보장은 없다(바로 다음 RGB가 더 가까울 수 있음).
그래서 "열화상 시각 이후의 RGB가 한 장 들어올 때까지 기다렸다가" 최근 RGB 몇 장 중 가장 가까운 것을 고른다.

```python
from collections import deque


class NearestPairer:
    """기록 순서대로 들어오는 RGB·열화상을 header 시각이 가장 가까운 쌍으로 묶는다.

    열화상 하나당 RGB 하나. 열화상 시각 이후의 RGB가 한 장 들어와야 "가장 가까운지" 확정된다.
    """

    def __init__(self, max_dt: float = 0.05, keep: int = 4):
        self.max_dt = max_dt
        self.rgb = deque(maxlen=keep)       # (t, rgb_msg)
        self.pending = []                   # [(t, thermal_msg)]

    def add_rgb(self, t, msg):
        self.rgb.append((t, msg))
        return self._resolve(final=False)

    def add_thermal(self, t, msg):
        self.pending.append((t, msg))
        return self._resolve(final=False)

    def flush(self):
        return self._resolve(final=True)

    def _resolve(self, final):
        out, keep = [], []
        latest = self.rgb[-1][0] if self.rgb else None
        for t, th in self.pending:
            if not final and (latest is None or latest < t):
                keep.append((t, th))        # 아직 이후 RGB가 안 옴 → 대기
                continue
            if self.rgb:
                tr, rgb = min(self.rgb, key=lambda x: abs(x[0] - t))
                if abs(tr - t) <= self.max_dt:
                    out.append((t, th, tr, rgb))
        self.pending = keep
        return out
```

추출 스크립트 본체 (gauge 3.4절 스크립트에 짝짓기만 추가):

```python
#!/usr/bin/env python3
"""bag → rgb_*.png, thermal_raw_*.png, pairs.csv

사용: python3 bag_to_pairs.py <bag_dir> <out_dir> [--max-dt 0.05] [--every 2]
"""
import argparse
import csv
import os

import cv2
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

from pairer import NearestPairer          # 위 클래스 (같은 폴더)


def stamp_sec(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag"); ap.add_argument("out")
    ap.add_argument("--max-dt", type=float, default=0.05)
    ap.add_argument("--every", type=int, default=2, help="N쌍마다 1쌍 저장")
    args = ap.parse_args()

    reader = SequentialReader()
    reader.open(StorageOptions(uri=args.bag, storage_id="mcap"), ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    bridge, pairer = CvBridge(), NearestPairer(max_dt=args.max_dt)
    os.makedirs(args.out, exist_ok=True)
    n = 0

    with open(os.path.join(args.out, "pairs.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["tag", "rgb_file", "thermal_file", "dt_ms", "encoding"])

        def save(pairs):
            nonlocal n
            for t_th, th, t_rgb, rgb in pairs:
                n += 1
                if n % args.every:
                    continue
                s = th.header.stamp
                tag = f"{s.sec}_{s.nanosec:09d}"                  # 열화상 스탬프 기준 (2.3절과 같은 규칙)
                cv2.imwrite(os.path.join(args.out, f"rgb_{tag}.png"), bridge.imgmsg_to_cv2(rgb, "bgr8"))
                cv2.imwrite(os.path.join(args.out, f"thermal_raw_{tag}.png"),
                            bridge.imgmsg_to_cv2(th, "passthrough"))   # mono8/mono16 그대로
                w.writerow([tag, f"rgb_{tag}.png", f"thermal_raw_{tag}.png",
                            f"{(t_rgb - t_th) * 1000:+.1f}", th.encoding])

        while reader.has_next():
            topic, data, _ = reader.read_next()
            if topic not in ("/camera/image", "/thermal/image"):
                continue
            msg = deserialize_message(data, get_message(types[topic]))
            if topic == "/camera/image":
                save(pairer.add_rgb(stamp_sec(msg), msg))
            else:
                save(pairer.add_thermal(stamp_sec(msg), msg))
        save(pairer.flush())
    print(f"pairs: {n}")


if __name__ == "__main__":
    main()
```

- RGB 메시지를 `deque(maxlen=4)`로만 들고 있어 메모리가 일정하다. bag 전체 이미지를 리스트에 담으면 1분짜리도 1 GB 가까이 된다.
- `max_dt`를 넘는 열화상은 버려진다. 추출 결과 쌍 수가 열화상 메시지 수(`ros2 bag info`)보다 크게 적으면 RGB가 빠진 구간이 있는 것이다.
- 파일 이름 규칙(`rgb_<tag>`, `thermal_raw_<tag>`)은 2.3절 노드 저장과 같게 맞췄다. 노드로 저장한 쌍과 bag에서 뽑은 쌍을 같은 코드로 처리할 수 있다.
- 짝짓기 클래스는 가짜 시각열(15 Hz·10 Hz, 기록 순서 지터 포함)로 검증해 두었다: 모든 열화상이 짝을 찾고, 고른 RGB가 항상 가장 가까운 것이었다. 6번에서 단위 테스트로 옮긴다.

### 4.4 데이터셋 구성과 정답

```
docs/captures/thermal/                 # *.png는 .gitignore로 커밋 안 됨
  factory_gas_tank_L16_hot_01/          # 과열 밸브 있음 (3.5절 나안)
    rgb_*.png, thermal_raw_*.png
    pairs.csv                           # tag, rgb_file, thermal_file, dt_ms, encoding
    labels.csv                          # tag, hot(0/1), hot_x, hot_y, hot_w, hot_h (RGB 픽셀 박스)
  factory_gas_tank_L16_normal_01/       # 과열 밸브 온도를 30 ℃로 → 오탐 측정용
```

정답 라벨이 두 종류 필요하다:

| 라벨 | 쓰임 | 만드는 법 |
|------|------|-----------|
| 과열 있음/없음 (`hot`) | 알람 정확도(오탐·미탐) | 장면 설정으로 정해짐. 폴더 단위로 같다 |
| 과열 위치 박스 (RGB 좌표) | **정렬 정확도** 측정 | 발열체 3D 위치(8, −5.64, 0.5)를 RGB 카메라로 투영하거나, 몇 장은 RGB에서 손으로 박스를 친다 |

- 정렬 검증의 핵심: 열화상에서 찾은 과열 블롭을 5번 정렬식으로 RGB에 옮겼을 때, RGB에서 본 밸브 위치와 몇 px 어긋나는지. 그래서 RGB 쪽 정답 박스가 필요하다.
- **정상 장면**도 꼭 녹화한다. 과열 장면만 있으면 "항상 과열"을 내는 코드도 100%가 나온다.
- 거리별 비교(STUDY_PLAN 4.6절, 1·2·5 m)를 하려면 정차 위치만 바꿔 같은 장면을 여러 번 녹화한다. bag 이름에 거리를 넣는다.

### 4.5 16비트 열화상 PNG 확인

```python
import cv2, numpy as np
th = cv2.imread("thermal_raw_123_400000000.png", cv2.IMREAD_UNCHANGED)
print(th.dtype, th.shape)                     # uint16 (240, 320) 이어야 함. uint8이면 IMREAD_UNCHANGED 빠짐
c = th.astype(np.float32) * 0.01 - 273.15     # 3.5절 to_celsius
print(f"min {c.min():.1f} ℃, max {c.max():.1f} ℃")   # 과열 장면이면 max ≈ 85
```

### 4.6 실습 체크리스트

- [ ] `gas_tank` 정차 상태에서 4.1절 명령으로 10초 녹화 → `ros2 bag info`로 `/camera/image` ≈ 150개, `/thermal/image` ≈ 100개 확인
- [ ] `bag_to_pairs.py`로 쌍 추출, `pairs.csv`의 `dt_ms` 분포 확인 (0과 ±33 근처에 몰리는지) → `max_dt`·6번 `slop` 결정
- [ ] 쌍 수가 열화상 메시지 수(÷`--every`)와 거의 같은지 확인
- [ ] `thermal_raw_*.png`를 `IMREAD_UNCHANGED`로 읽어 `dtype` 확인 (L16이면 `uint16`)
- [ ] 과열 장면·정상 장면 bag을 각각 확보, `labels.csv` 작성
- [ ] RGB 몇 장에서 과열 밸브 위치 박스를 손으로 기록 (5번 정렬 검증용)

### 4.7 헷갈리기 쉬운 것

- 짝짓기는 **`header.stamp`(센서 시각)** 기준이다. `read_next()`가 주는 세 번째 값(기록 시각)은 수신 시각이라 두 토픽 사이 지연이 섞인다.
- 열화상 `passthrough`로 받은 `uint16`을 `imwrite`하면 16비트 PNG가 된다. 일반 이미지 뷰어에서는 대비 없는 밋밋한 회색으로 보이는 게 정상이다(20~85 ℃가 픽셀 29300~35800이라 0~65535 범위에서 차이가 아주 작음). 보기용은 컬러맵 파일을 따로 만든다(2.2절).
- `mono8` 시절 bag과 `mono16` bag을 섞어 튜닝하면 임계값 단위가 달라 엉망이 된다. `pairs.csv`의 `encoding` 열로 걸러 낸다.
- `--every`로 솎아 낼 때 쌍 단위로 솎아야 한다. 토픽별로 따로 솎으면 짝이 깨진다.

### 4.8 참고

- gauge_ocr/STUDY.md 3절 (rosbag 기본, 시간 문제, `rosbag2_py`)
- `message_filters` ApproximateTime 알고리즘 설명 (짝짓기 개념 비교): https://wiki.ros.org/message_filters/ApproximateTime
- OpenCV 16비트 PNG: https://docs.opencv.org/4.x/d4/da8/group__imgcodecs.html

## 5. 정렬 → 임계 검출 → 오버레이

계획 5번 항목. 4절의 PNG 쌍을 입력으로 **열화상 한 장 + RGB 한 장 → (과열 블롭 목록, 오버레이 그림)** 함수를 오프라인에서 완성한다.
모두 ROS 없는 순수 함수로 `align.py`, `hotspot.py`, `overlay.py`에 나눈다(STUDY_PLAN 7절 파일 구조).

### 5.1 파이프라인 한눈에

```
thermal_raw (uint8/uint16)          rgb (bgr8)
   │ to_celsius (3.5절)                │
   ▼                                  │
 ℃ 배열 240×320 ──► find_hot_blobs ──► 블롭 목록 (열화상 좌표) ──► Hysteresis ──► 알람 Bool
   │                                  │                          │
   │ warp_to_rgb (H)                  │                          │ map_points (H)
   ▼                                  ▼                          ▼
 ℃ 배열 480×640 + 유효 마스크 ──► fuse ──► draw_blobs ──► 오버레이 이미지
```

- **검출은 열화상 원본 해상도에서** 한다. 정렬로 늘린 이미지에서 검출하면 보간된 값이 섞이고 계산량도 늘어난다.
- 정렬은 **그림 그릴 때만** 쓴다(열지도 합성 + 박스 좌표 변환).

### 5.2 정렬 이론 — 왜 K만으로 되는가

두 카메라는 같은 `base_link`에 같은 방향(회전 0)으로 붙어 있고 위치만 z로 3 cm 다르다(RGB 0.05, 열화상 0.02).

1. **회전도 이동도 없다면**: 같은 방향의 광선은 두 이미지에서 `p_rgb ~ K_rgb · K_th⁻¹ · p_th`로 대응한다(동차좌표). 이 3×3 행렬이 호모그래피 H다. 거리와 무관하다.
2. **3 cm 이동(시차)**: 열화상이 아래에 있으므로 같은 물체가 열화상에서는 상대적으로 **위**에 맺힌다. RGB로 옮길 때 아래(+v)로 `Δv = fy_rgb · 0.03 / Z` 만큼 밀어야 한다. Z(물체 거리)에 따라 달라서 **한 거리만 정확**하다.

| 물체 거리 Z | 시차 Δv (RGB px) |
|-------------|------------------|
| 0.37 m (현재 `gas_tank` 정차점) | 31 px |
| 1.37 m (정차점 y −4.0안) | 8 px |
| 1.87 m | 6 px |
| 5 m | 2 px |

- 그래서 설계 거리(정차점에서 발열체까지 거리)를 파라미터 `depth`로 두고 그 거리에 맞춘다. 3.4절에서 정차점을 물리자고 한 또 하나의 이유: 가까울수록 거리 오차에 따른 정렬 오차가 커진다.
- 내부 파라미터: `fx = (W/2) / tan(HFOV/2)`, `cx = W/2`, `cy = H/2`, `fy = fx`. RGB `fx ≈ 381`, 열화상 `fx ≈ 293`, 배율 ≈ **1.30**. 열화상 320×240은 RGB 위에서 약 **417×313** 영역이 된다.
- RGB는 `/camera/camera_info`의 `K`로 검증할 수 있다(위 계산과 같아야 함). 열화상 camera_info는 브리지에 없어 SDF 값으로 계산한다.
- (심화) 실제 로봇이라면 두 카메라 사이 회전·이동을 캘리브레이션해야 한다. 열화상에서 보이는 체커보드(가열판)로 스테레오 캘리브레이션을 한다. 시뮬은 SDF에 정확한 값이 있으므로 생략.

### 5.3 `align.py`

```python
"""열화상 → RGB 정렬. 두 카메라의 내부 파라미터(K)만 쓴다."""

import math

import cv2
import numpy as np


def intrinsics(width: int, height: int, hfov: float) -> np.ndarray:
    """SDF의 해상도·수평 화각 → K. 정사각 픽셀, 주점 = 이미지 중앙."""
    fx = (width / 2) / math.tan(hfov / 2)
    return np.array([[fx, 0, width / 2], [0, fx, height / 2], [0, 0, 1]], np.float64)


K_RGB = intrinsics(640, 480, 1.396)
K_TH = intrinsics(320, 240, 1.0)


def thermal_to_rgb_h(k_rgb=K_RGB, k_th=K_TH, baseline_z: float = 0.03, depth: float = 1.5) -> np.ndarray:
    """열화상 픽셀 → RGB 픽셀 호모그래피.

    두 카메라는 방향이 같고 열화상이 baseline_z(m) 아래에 있다.
    회전이 없으므로 무한원 호모그래피는 K_rgb · K_th⁻¹, 거리 depth(m)의 물체는 시차만큼 아래로 민다.
    """
    h = k_rgb @ np.linalg.inv(k_th)
    dv = k_rgb[1, 1] * baseline_z / depth              # 열화상이 아래 → 같은 물체가 열화상에선 위에 맺힘 → RGB로 옮길 때 아래(+v)로
    shift = np.array([[1, 0, 0], [0, 1, dv], [0, 0, 1]], np.float64)
    return shift @ h


def warp_to_rgb(thermal: np.ndarray, h: np.ndarray, rgb_size=(640, 480)) -> tuple[np.ndarray, np.ndarray]:
    """열화상(2D 배열)을 RGB 해상도로 옮긴다. (옮긴 배열, 유효 영역 마스크)."""
    warped = cv2.warpPerspective(thermal, h, rgb_size, flags=cv2.INTER_LINEAR)
    valid = cv2.warpPerspective(np.full(thermal.shape[:2], 255, np.uint8), h, rgb_size, flags=cv2.INTER_NEAREST)
    return warped, valid


def map_points(pts_th: np.ndarray, h: np.ndarray) -> np.ndarray:
    """열화상 픽셀 좌표 N×2 → RGB 픽셀 좌표 N×2."""
    return cv2.perspectiveTransform(pts_th.reshape(-1, 1, 2).astype(np.float64), h).reshape(-1, 2)
```

- `cv2.warpPerspective(src, M, dsize)`에서 M은 **src → dst** 방향이다(내부에서 역행렬로 역매핑). 그래서 열화상 → RGB인 H를 그대로 넣는다. 반대로 넣으면 열지도가 작게 줄어든다.
- 유효 마스크(`valid`)는 열화상 화각(57°)이 RGB(80°)보다 좁아 생기는 **테두리 빈 영역**을 구분하려고 만든다. 빈 영역을 0 ℃로 칠하면 컬러맵에서 검게 나와 보기 흉하다.
- 점만 옮길 때는 이미지 전체를 warp하지 말고 `perspectiveTransform`으로 좌표만 바꾼다(블롭 박스).

### 5.4 `hotspot.py` — 임계 검출과 히스테리시스

```python
"""과열 영역 검출과 알람 히스테리시스. 입력은 ℃ 배열(3.5절 to_celsius 결과)."""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Blob:
    x: int
    y: int
    w: int
    h: int
    area: int
    t_max: float
    cx: float
    cy: float


def find_hot_blobs(temp_c: np.ndarray, threshold: float, min_area: int = 20) -> list[Blob]:
    mask = (temp_c >= threshold).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))     # 점 노이즈 제거
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
    blobs = []
    for i in range(1, n):                                                          # 0은 배경
        x, y, w, h, area = stats[i]
        if area < min_area:
            continue
        t_max = float(temp_c[labels == i].max())
        blobs.append(Blob(int(x), int(y), int(w), int(h), int(area), t_max, *map(float, cents[i])))
    return sorted(blobs, key=lambda b: b.t_max, reverse=True)


class Hysteresis:
    """켜짐 임계 > 꺼짐 임계, 그리고 N프레임 연속 조건으로 알람 깜빡임을 막는다."""

    def __init__(self, on: float, off: float, n_on: int = 3, n_off: int = 5):
        assert on > off
        self.on, self.off, self.n_on, self.n_off = on, off, n_on, n_off
        self.state = False
        self._cnt = 0

    def update(self, t_max: float) -> bool:
        if not self.state:
            self._cnt = self._cnt + 1 if t_max >= self.on else 0
            if self._cnt >= self.n_on:
                self.state, self._cnt = True, 0
        else:
            self._cnt = self._cnt + 1 if t_max < self.off else 0
            if self._cnt >= self.n_off:
                self.state, self._cnt = False, 0
        return self.state
```

| 단계 | 함수 | 이유 |
|------|------|------|
| 이진화 | `temp_c >= threshold` | ℃ 배열이므로 numpy 비교로 충분. `cv2.threshold`는 float32에서도 되지만 이게 더 읽기 쉽다 |
| 노이즈 제거 | `morphologyEx(MORPH_OPEN)` | 침식 → 팽창. 1~2 px 점은 사라지고 큰 블롭은 모양 유지 |
| 블롭 분리 | `connectedComponentsWithStats` | `findContours`보다 간단히 박스·면적·중심을 한 번에 준다 |
| 크기 필터 | `area >= min_area` | 노이즈·먼 작은 물체 제외. 열화상 px 기준 |
| 최고 온도 | `temp_c[labels == i].max()` | 블롭별 최고 온도 → 알람·표시 |

히스테리시스:
- 켜짐 60 ℃ / 꺼짐 55 ℃처럼 **두 임계를 다르게** 두면 59~61 ℃를 오가는 경계에서 알람이 깜빡이지 않는다.
- 추가로 **N프레임 연속** 조건: 켜짐은 3프레임(10 Hz → 0.3 s), 꺼짐은 5프레임. 노이즈 한 프레임으로 알람이 울리지 않고, 꺼질 때는 더 신중하게.
- 입력은 "이번 프레임의 최고 온도"(블롭이 없으면 화면 최고값 또는 −inf). 블롭 개수가 아니라 온도로 판정해야 임계 의미가 명확하다.

### 5.5 `overlay.py` — 열지도 합성과 박스

```python
"""RGB 위에 정렬된 열지도와 과열 박스를 그린다."""

import cv2
import numpy as np

from align import map_points        # 패키지에서는 from thermal_fusion.align import map_points


def fuse(rgb: np.ndarray, temp_warped: np.ndarray, valid: np.ndarray, t_lo: float, t_hi: float,
         alpha: float = 0.45) -> np.ndarray:
    """t_lo~t_hi(℃)를 컬러맵으로 칠해 유효 영역에만 반투명 합성."""
    norm = np.clip((temp_warped - t_lo) / (t_hi - t_lo), 0, 1)
    color = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_INFERNO)
    blended = cv2.addWeighted(rgb, 1 - alpha, color, alpha, 0)
    out = rgb.copy()
    out[valid > 0] = blended[valid > 0]                 # 열화상 화각 밖은 원본 RGB 그대로
    return out


def draw_blobs(img: np.ndarray, blobs, h: np.ndarray, alert: bool) -> np.ndarray:
    for b in blobs:
        corners = np.array([[b.x, b.y], [b.x + b.w, b.y + b.h]], np.float64)
        (x0, y0), (x1, y1) = map_points(corners, h).astype(int)     # 열화상 박스 → RGB 박스
        cv2.rectangle(img, (x0, y0), (x1, y1), (0, 0, 255) if alert else (0, 255, 255), 2)
        cv2.putText(img, f"{b.t_max:.0f}C", (x0, max(y0 - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(img, "ALERT" if alert else "normal", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 0, 255) if alert else (0, 200, 0), 2, cv2.LINE_AA)
    return img
```

- `applyColorMap`은 `uint8` 입력만 받는다. ℃ → 0~1 정규화 → ×255 → `uint8`. 정규화 범위(`t_lo`~`t_hi`)를 **고정값**(예: 20~90 ℃)으로 두면 프레임마다 색이 바뀌지 않아 비교가 쉽다. 프레임별 min~max로 하면 아무것도 없는 장면에서도 빨간색이 나온다.
- `addWeighted(rgb, 1−α, color, α, 0)`는 픽셀별 가중합. 유효 마스크 밖은 원본 RGB를 유지한다.
- 박스는 열화상 좌표 블롭의 두 모서리를 `map_points`로 RGB 좌표로 옮겨 그린다.
- `cv2.putText`의 Hershey 폰트는 `℃`·한글을 못 그린다. `C`로 쓴다.

### 5.6 합성 데이터로 검증한 결과

시뮬 데이터 없이도 카메라 모델로 정답을 만들 수 있다: 3D 점 하나를 두 카메라에 각각 투영(`u = fx·(−Y/X) + cx`, `v = fy·(−Z/X) + cy`)하고, 열화상 좌표를 `map_points`로 옮긴 결과가 RGB 투영과 같은지 본다.

| 검증 | 결과 |
|------|------|
| 설계 거리(1.5 m)의 점 3개 (중앙, 좌하, 우상) | RGB 투영과 **1e-6 px 이내 일치** |
| 설계 거리 1.5 m로 정렬, 실제 물체는 0.4 m | 약 **21 px** 어긋남 (= 381 · 0.03 · (1/0.4 − 1/1.5)) |
| 열화상 유효 영역 크기 (RGB 위) | 약 417 × 313 px |
| 85 ℃ 20×15 px 블롭 + 90 ℃ 단일 픽셀 노이즈 | 블롭 1개만 검출, 박스·최고 온도 정확 |
| 히스테리시스 (켜짐 3프레임, 꺼짐 2프레임) | 1프레임 저온에 카운트 리셋, 꺼짐 임계 사이 값(58 ℃)에선 유지 |

이 검증 코드는 6번에서 `test/test_align.py`, `test/test_hotspot.py`로 옮긴다.

### 5.7 오프라인 튜닝 흐름

```python
# tools/try_fusion.py — 4절 데이터셋 폴더를 돌며 오버레이 저장 + 판정 표
import csv, os, sys
import cv2, numpy as np
from thermal_fusion.align import thermal_to_rgb_h, warp_to_rgb
from thermal_fusion.hotspot import find_hot_blobs
from thermal_fusion.overlay import draw_blobs, fuse

folder, depth, thr = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
H = thermal_to_rgb_h(depth=depth)
os.makedirs(os.path.join(folder, "fused"), exist_ok=True)
for row in csv.DictReader(open(os.path.join(folder, "pairs.csv"))):
    rgb = cv2.imread(os.path.join(folder, row["rgb_file"]))
    raw = cv2.imread(os.path.join(folder, row["thermal_file"]), cv2.IMREAD_UNCHANGED)
    temp = raw.astype(np.float32) * 0.01 - 273.15 if row["encoding"] == "mono16" else raw.astype(np.float32)
    blobs = find_hot_blobs(temp, thr)
    warped, valid = warp_to_rgb(temp, H)
    img = draw_blobs(fuse(rgb, warped, valid, 20, 90), blobs, H, alert=bool(blobs))
    cv2.imwrite(os.path.join(folder, "fused", f"fused_{row['tag']}.png"), img)
    print(row["tag"], len(blobs), f"{blobs[0].t_max:.1f}" if blobs else "-")
```

- 정렬 튜닝: 오버레이에서 열지도 속 밸브와 RGB 밸브가 겹치는지 본다. 위아래로 어긋나면 `depth`, 좌우·전체적으로 어긋나면 K 계산(HFOV 값)을 의심한다.
- 4.4절의 RGB 정답 박스와 `map_points`로 옮긴 블롭 박스 중심 거리(px)를 재면 정렬 오차가 숫자로 나온다 → REPORT.md.
- 정상 장면 폴더로 돌려 블롭이 0개인지(오탐) 확인한다.

### 5.8 실습 체크리스트

- [ ] `intrinsics()`로 계산한 RGB K가 `/camera/camera_info`의 `k` 필드와 같은지 확인
- [ ] 5.6절 합성 검증을 Mac 로컬 venv에서 재현
- [ ] 4절 쌍 하나로 `warp_to_rgb` 결과를 저장해 RGB 위 약 417×313 영역에 열지도가 놓이는지 확인
- [ ] `depth`를 0.4 / 1.4 / 2.0으로 바꿔 가며 밸브 정렬이 어떻게 변하는지 오버레이로 비교
- [ ] 과열 장면에서 블롭 1개, 정상 장면에서 0개 나오는 `threshold`·`min_area` 찾기
- [ ] 정렬 오차(px)와 오탐·미탐 수를 표로 정리

### 5.9 헷갈리기 쉬운 것

- 이미지 v축은 **아래로** 증가, 카메라 Z(위)와 반대. 시차 보정 부호를 틀리면 오차가 두 배가 된다. 5.6 합성 검증으로 부호를 확인한다.
- `warpPerspective`의 `dsize`는 `(너비, 높이)` = `(640, 480)`. numpy `shape`는 `(480, 640)`. 순서가 반대.
- ℃ float 배열을 `warpPerspective`하면 float 그대로 나온다(좋음). `uint16` 원본을 바로 warp하면 보간 결과가 정수로 잘린다 → `to_celsius` 먼저.
- `connectedComponentsWithStats`의 0번 라벨은 배경이다. `range(1, n)`.
- `MORPH_OPEN` 커널이 블롭보다 크면 블롭이 사라진다. 멀리서 본 작은 발열체가 안 잡히면 커널부터 의심.
- `applyColorMap` 결과는 BGR이다. RGB와 섞을 때 순서가 맞는다.

### 5.10 참고

- 호모그래피와 카메라 모델: https://docs.opencv.org/4.x/d9/dab/tutorial_homography.html
- `warpPerspective`·`perspectiveTransform`: https://docs.opencv.org/4.x/da/d54/group__imgproc__transform.html
- 모폴로지: https://docs.opencv.org/4.x/d9/d61/tutorial_py_morphological_ops.html
- `connectedComponentsWithStats`: https://docs.opencv.org/4.x/d3/dc0/group__imgproc__shape.html
- 컬러맵: https://docs.opencv.org/4.x/d3/d50/group__imgproc__colormap.html

## 6. message_filters 동기화 노드화·파라미터화·단위 테스트

> 계획 6번 항목. 학습 후 기입.
