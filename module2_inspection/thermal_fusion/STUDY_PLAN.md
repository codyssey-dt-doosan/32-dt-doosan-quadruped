# thermal_fusion 학습·준비 계획 (운학)

`/camera/image`(RGB)와 `/thermal/image`(흑백)를 정렬해 과열 영역을 RGB 위에 표시하고,
과열 여부를 `/inspection/thermal_alert`(Bool)로 발행한다.
작업 착수 전에 **무엇을 공부하고 무엇을 준비해야 하는지** 정리한 문서다.

---

## 1. 현재 상태

| 항목 | 내용 |
|------|------|
| 구독 | `/camera/image` (640×480 RGB, 15 Hz), `/thermal/image` (320×240 L8, 10 Hz) |
| 발행 | `/inspection/thermal_alert` (std_msgs/Bool) |
| 코드 | `thermal_fusion/thermal_fusion_node.py` 스켈레톤. 콜백은 이미지를 버리고 0.5 s마다 False 발행 |
| 런치 | `launch/thermal_fusion.launch.py`가 `world` 인자와 `use_sim_time=True`를 넘김. 노드는 아직 `world` 파라미터를 선언하지 않음 |
| 의존성 | `package.xml`에 rclpy, sensor_msgs, std_msgs만 있음 |

- 발행 토픽 이름·타입은 관제 대시보드(`monitoring/web/app.js`, "과열"/"정상" 표시)가 이미 구독 중이므로 **바꾸면 안 되는 인터페이스**다.
- Go2 모델의 `thermal` 센서는 `type="camera"` + `L8` 포맷, 즉 **그냥 흑백 카메라**다. 밝은 물체가 밝게 나올 뿐 온도 정보가 없다. → 3절에서 방향을 먼저 정해야 한다.

### 토픽 이름은 폴더가 아니다

`/camera/image`, `/thermal/image`, `/inspection/thermal_alert`처럼 슬래시로 구분된 이름은 **ROS 2 토픽 이름**(네임스페이스)이지 디렉터리가 아니다. 저장소에 `camera/`, `thermal/`, `inspection/` 폴더는 없다. 각 이름이 정해지는 곳은 다음과 같다.

| 토픽 | 정해지는 곳 | 설명 |
|------|-------------|------|
| `/camera/image` | `simulation/config/ros_gz_bridge.yaml` | Go2 모델(`simulation/models/go2/model.sdf`)의 카메라 센서가 `<topic>camera</topic>`으로 Gazebo 토픽 `/camera`를 내고, 브리지가 ROS 토픽 `/camera/image`로 변환 |
| `/thermal/image` | 같은 브리지 파일 | 열화상 센서의 `<topic>thermal</topic>` → Gazebo `/thermal` → ROS `/thermal/image` |
| `/camera/camera_info` | 같은 브리지 파일 | RGB 카메라 내부 파라미터. 열화상용은 아직 없음 |
| `/inspection/thermal_alert` | `thermal_fusion/thermal_fusion_node.py` | 우리 노드가 `create_publisher`로 직접 만드는 이름. `monitoring/web/app.js`가 이 이름으로 구독 |

`ros2 topic list`로 실행 중인 토픽을 볼 수 있고, `ros2 topic info -v /thermal/image`로 누가 발행·구독하는지 확인할 수 있다.

---

## 2. 공통 기반 학습 (gauge_ocr과 공유)

### 2.1 ROS 2 Jazzy (Python, rclpy)

- [ ] 노드 생명주기: `rclpy.init` → `Node` 생성 → `spin` → `destroy_node` → `shutdown`
- [ ] 구독/발행 생성, 콜백 구조, 타이머
- [ ] **파라미터**: `declare_parameter` / `get_parameter`, 런치에서 넘기기, `ros2 param set`으로 런타임 튜닝
  - 임계값(과열 기준, 최소 면적 등)은 전부 파라미터로 빼야 실험이 편하다
- [ ] **QoS**: 이미지 토픽은 `qos_profile_sensor_data`(BEST_EFFORT) 권장
- [ ] **`message_filters.ApproximateTimeSynchronizer`**: RGB(15 Hz)와 열화상(10 Hz)처럼 주기가 다른 토픽을 시간 기준으로 묶을 때. 이 패키지에서 핵심
- [ ] `use_sim_time`: 시뮬 시간(`/clock`) 기준 스탬프. 런치에 이미 `True`
- [ ] ament_python 패키지 구조: `setup.py`의 `data_files`(launch, config 설치), `entry_points`, `package.xml` 의존성

참고: https://docs.ros.org/en/jazzy/Tutorials.html , https://docs.ros.org/en/jazzy/p/message_filters/

### 2.2 cv_bridge + OpenCV

- [ ] `sensor_msgs/Image` → numpy: `CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")` (RGB), `"mono8"` (열화상 L8)
- [ ] 반대 방향 `cv2_to_imgmsg`: 오버레이 이미지를 토픽으로 낼 때 (`header` 복사 잊지 말 것)
- [ ] OpenCV 기초: `cvtColor`, `threshold`, 모폴로지, `findContours`, `connectedComponentsWithStats`, `resize`, `warpPerspective`, `applyColorMap`, `addWeighted`
- [ ] 설치: `sudo apt install ros-jazzy-cv-bridge python3-opencv`

### 2.3 Gazebo Harmonic 카메라

- [ ] Go2 모델(`simulation/models/go2/model.sdf`) 카메라 스펙

  | 센서 | 해상도 | HFOV | 주기 | 포맷 | 위치 (head 링크 기준) |
  |------|--------|------|------|------|------|
  | `camera` | 640×480 | 1.396 rad (80°) | 15 Hz | RGB | (0.28, 0, 0.05) |
  | `thermal` | 320×240 | 1.0 rad (57°) | 10 Hz | L8 | (0.28, 0, 0.02) |

- [ ] 초점거리 계산: `fx = (W/2) / tan(HFOV/2)`
- [ ] `/camera/camera_info`의 `K` 행렬. RGB는 브리지에 있고 **열화상 camera_info는 브리지에 없음**
- [ ] SDF 센서·플러그인 구조 (`<sensor>`, `<plugin>`, 월드의 sensors 시스템). 열화상 센서 교체 시 필요
- [ ] `GZ_SIM_RESOURCE_PATH`로 모델 경로 잡는 방식 (통합 런치에서 설정)

참고: https://gazebosim.org/docs/harmonic , https://sdformat.org/spec

### 2.4 개발·디버깅 도구

- [ ] `ros2 topic list / echo / hz / info -v` (QoS 확인)
- [ ] `rqt_image_view`: `/camera/image`, `/thermal/image`, 오버레이 토픽 확인
- [ ] `ros2 bag record /camera/image /camera/camera_info /thermal/image /odom /tf`
  - 시뮬 없이 알고리즘만 반복 개발할 때 필수. 한 번 녹화 후 `ros2 bag play`
- [ ] 두 프레임을 같은 스탬프로 PNG 덤프 → 오프라인에서 정렬·임계 튜닝 → 노드에 이식
- [ ] `colcon build --symlink-install --packages-select thermal_fusion`
- [ ] `colcon test` + pytest: 정렬·검출 함수는 순수 함수로 두고 PNG 입력으로 단위 테스트

### 2.5 환경 준비 체크리스트

- [ ] Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic (루트 README apt 목록)
- [ ] 추가 설치: `ros-jazzy-cv-bridge`, `python3-opencv`, `ros-jazzy-rqt-image-view`, `ros-jazzy-ros2bag`
- [ ] `ros2 launch simulation full_system.launch.py world:=factory` 정상 기동 확인
- [ ] `rqt_image_view`에서 `/camera/image`, `/thermal/image` 둘 다 보이는지 확인
- [ ] 가스탱크 앞(factory `gas_tank` 정차점 (8, −5))에서 RGB·열화상 프레임 캡처해 `docs/captures/`에 저장
- [ ] rosbag 하나 녹화해두기
- [ ] `Dockerfile`에 cv_bridge 의존성 추가 필요 여부 확인

---

## 3. 먼저 결정할 것: 열화상을 "진짜"로 만들 것인가

| 선택 | 내용 | 필요한 것 |
|------|------|-----------|
| **(1) 흑백 카메라 유지** | 발열체 material을 밝게(흰색/자발광) 만들어 "뜨거우면 밝다"로 흉내 | 시뮬 모델 material만 수정. 구현 단순 |
| **(2) Gazebo 열화상 센서 사용** | `<sensor type="thermal">` + 월드에 `gz-sim-thermal-system` 플러그인, 발열체 visual에 `<temperature>` 지정 → 픽셀값이 온도 | ogre2 렌더러, 센서 시스템 설정. 출력이 L16(켈빈)이라 처리 코드 달라짐. 보고서에 "온도 ℃" 표기 가능 |

(2)가 데모 설득력이 훨씬 좋다. 공부 목록에 넣되, 시간 부족하면 (1)로 대체.
참고: Gazebo Sim 8(Harmonic) Thermal Camera 튜토리얼 — https://gazebosim.org/api/sim/8/thermalcameraigngazebo.html

(2)를 쓸 때 알아야 할 점:
- [ ] 월드 `<plugin filename="gz-sim-sensors-system">`에 `<render_engine>ogre2</render_engine>` 필요
- [ ] 센서 `<camera><image><format>L16</format>` (기본) 또는 `L8` + `<linear_resolution>` 설정
- [ ] 발열체 `<visual>` 안에 `<plugin filename="gz-sim-thermal-system" name="gz::sim::systems::Thermal"><temperature>600</temperature></plugin>` (켈빈)
- [ ] 브리지 타입은 그대로 `gz.msgs.Image` → `sensor_msgs/Image`, 인코딩이 `mono16`이 됨 → cv_bridge에서 `mono16`으로 받아 `uint16` 배열 처리
- [ ] 온도 변환: 픽셀값 × linear_resolution = 켈빈, −273.15 = ℃

---

## 4. 파이프라인과 공부할 항목

### 4.1 시간 동기화

- [ ] `message_filters.Subscriber` + `ApproximateTimeSynchronizer(slop=0.05~0.1)`로 RGB·열화상 묶기
- [ ] 대안: "최신 RGB 프레임 캐시 + 열화상 콜백에서 융합" (로봇이 정차 중이면 충분)

### 4.2 정렬(registration)

두 카메라는 같은 head 링크에 붙어 있고 위치 차이가 3 cm(수직)뿐이라 **회전·평행 이동은 거의 0**, 차이는 **화각과 해상도**다.
따라서 특징점 매칭 없이 **내부 파라미터만으로 정렬**할 수 있다.

- [ ] 초점거리

  | 카메라 | W | HFOV | fx (px) |
  |--------|---|------|---------|
  | RGB | 640 | 1.396 rad | ≈ 380 |
  | 열화상 | 320 | 1.0 rad | ≈ 293 |

- [ ] 같은 방향의 광선이 두 이미지에서 맺히는 위치: `u_rgb = cx_rgb + (fx_rgb / fx_th) · (u_th − cx_th)`
  - 배율 ≈ 380/293 ≈ 1.30 → 열화상 320×240은 RGB 상에서 약 415×311 px 크기의 중앙 영역에 해당
- [ ] 구현: `cv2.resize`로 열화상을 1.30배 키운 뒤 RGB 중앙에 배치 (또는 호모그래피 `H = K_rgb · K_th⁻¹`로 `cv2.warpPerspective`)
- [ ] 3 cm 수직 오프셋(시차)은 거리 1 m 이상에서 몇 픽셀 수준 → 무시하거나 상수 오프셋 보정
- [ ] 공부: 핀홀 모델, 내부 파라미터 행렬 K, 호모그래피, `cv2.warpPerspective` / `cv2.warpAffine`
- [ ] (심화, 실제 로봇 대비) 특징 기반 정렬: RGB·열화상은 외관이 달라 ORB 매칭이 잘 안 된다. 에지 기반 정합, 열화상용 체커보드(가열판) 캘리브레이션 개념만 알아두기

### 4.3 과열 영역 검출

- [ ] 이진화: 열화상 픽셀 > `hot_threshold` (L8이면 0~255, 온도면 ℃). 파라미터로
- [ ] 노이즈 제거: `morphologyEx(OPEN)`, `GaussianBlur`
- [ ] `findContours` / `connectedComponentsWithStats` → 면적 ≥ `min_area_px`인 블롭만
- [ ] 히스테리시스: 켜짐 임계 > 꺼짐 임계, N프레임 연속 조건으로 알람 깜빡임 방지
- [ ] 블롭 중심을 정렬 식으로 RGB 좌표로 변환 → 바운딩박스

### 4.4 융합·시각화

- [ ] `cv2.applyColorMap(thermal, cv2.COLORMAP_INFERNO 또는 JET)` → 컬러 열지도
- [ ] `cv2.addWeighted(rgb, 0.6, thermal_color_aligned, 0.4, 0)` 반투명 오버레이
- [ ] 과열 블롭에 박스·온도(또는 밝기값) 텍스트 (`cv2.rectangle`, `cv2.putText`)
- [ ] `/inspection/thermal_overlay`(sensor_msgs/Image, bgr8) 발행 → `rqt_image_view`·대시보드·보고서 캡처

### 4.5 알람 로직

- [ ] `/inspection/thermal_alert` Bool 발행 (인터페이스 고정)
- [ ] 필요하면 추가 토픽: 최고 온도 Float32, 블롭 개수 등 → 관제(수현)와 협의
- [ ] 가스 안전 모듈(채현)의 복귀 로직과 연동할지 결정

### 4.6 검증

- [ ] 발열체 있는/없는 장면에서 오탐·미탐 세기
- [ ] 정렬 정확도: 발열체 실루엣이 RGB 물체 윤곽과 겹치는지 오버레이로 확인, 픽셀 오프셋 측정
- [ ] 거리별(1, 2, 5 m) 검출 성공률 표 → REPORT.md

---

## 5. 참고 자료

- OpenCV: Image Thresholding, Morphological Transformations, Contours, ColorMaps, Geometric Transformations — https://docs.opencv.org/4.x/
- `message_filters` (ROS 2 Jazzy): https://docs.ros.org/en/jazzy/p/message_filters/
- Gazebo Thermal Camera 튜토리얼 (3절 링크) + `gz-sim` 소스의 `examples/worlds/thermal_camera.sdf`
- 핀홀 카메라·호모그래피: OpenCV `calib3d` 문서, Hartley & Zisserman 2장 (개념만)
- 실제 RGB-T 융합 배경지식: FLIR MSX(에지 오버레이) 개념, RGB-T 정합 자료 (심화, 선택)

---

## 6. 준비 작업 (시뮬·팀 협업)

- [ ] 발열체 정하기: factory의 `gas_tank_1/2`(8, −6), (10, −6) 또는 별도 "과열 모터/배관" 모델
- [ ] 선택 (1)이면 발열체 material을 `<emissive>`/밝은 색으로. 선택 (2)면 `<temperature>` 플러그인 삽입 + 월드 sensors 시스템 ogre2 확인
- [ ] 열화상 화각(57°)이 RGB(80°)보다 좁으니 정차점에서 발열체가 열화상 프레임 안에 들어오는지 확인 (태우의 `gas_tank` 정차점 (8, −5, yaw −1.57)과 비교)
- [ ] 필요하면 `/thermal/camera_info` 브리지 항목 추가 요청 (`simulation/config/ros_gz_bridge.yaml`)
- [ ] 수현(monitoring): 오버레이 이미지 토픽 대시보드 노출, 추가 정보(최고 온도) 표시 여부
- [ ] 채현(gas): 과열 알람과 가스 알람 우선순위, 복귀 트리거 연동 여부

---

## 7. 코드 설계 원칙과 수정 예정 사항

1. 정렬·검출 로직은 노드에서 분리한 순수 함수로 (PNG 테스트 가능).
2. 모든 임계값은 ROS 파라미터로. YAML을 `config/`에 두고 `setup.py` data_files로 설치.
3. 오버레이 디버그 토픽을 만든다.
4. 히스테리시스·N프레임 조건으로 알람 흔들림을 막는다.
5. `world` 파라미터를 선언해 corridor / factory 설정을 바꿀 수 있게 한다.

- `package.xml`: `cv_bridge`, `python3-opencv`, `message_filters`, `python3-numpy` 의존성 추가
- `setup.py`: `config/*.yaml` 설치 항목 추가
- 파일 구조(안):
  ```
  thermal_fusion/
    thermal_fusion_node.py   # 구독·동기화·발행
    align.py                 # K 기반 정렬 (순수 함수)
    hotspot.py               # 임계·블롭·히스테리시스 (순수 함수)
    overlay.py               # 컬러맵·블렌딩·박스
  config/
    thermal.yaml             # 임계값, 최소 면적, 배율, 오프셋
  test/
    test_align.py, test_hotspot.py
  ```

---

## 8. 학습 순서 정리

1. ROS 2 Python 기초 (노드·토픽·파라미터·런치) — 하루 (gauge_ocr과 공통) → 학습 내용: [STUDY.md](STUDY.md)
2. cv_bridge로 두 이미지 토픽 받아 PNG 저장까지 — 반나절
3. 열화상 센서 방향 결정 (3절) + 발열체 모델 협의 — 병행
4. rosbag 녹화 + 동일 스탬프 PNG 쌍 확보
5. 오프라인에서 정렬(배율·오프셋) → 임계 검출 → 오버레이 순으로 구현
6. `message_filters` 동기화 붙여 노드화, 파라미터화·단위 테스트 후 통합 런치에서 검증
