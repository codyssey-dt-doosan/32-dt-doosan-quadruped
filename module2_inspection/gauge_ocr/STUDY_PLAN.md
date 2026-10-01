# gauge_ocr 학습·준비 계획 (운학)

카메라 영상(`/camera/image`)에서 게이지를 찾아 값을 읽고 `/inspection/gauge`(Float32)로 발행한다.
작업 착수 전에 **무엇을 공부하고 무엇을 준비해야 하는지** 정리한 문서다.

---

## 1. 현재 상태

| 항목 | 내용 |
|------|------|
| 구독 | `/camera/image` (sensor_msgs/Image, 640×480 RGB, 15 Hz) |
| 발행 | `/inspection/gauge` (std_msgs/Float32) |
| 코드 | `gauge_ocr/gauge_ocr_node.py` 스켈레톤. 콜백은 이미지를 버리고 0.5 s마다 0.0 발행 |
| 런치 | `launch/gauge_ocr.launch.py`가 `world` 인자와 `use_sim_time=True`를 넘김. 노드는 아직 `world` 파라미터를 선언하지 않음 |
| 의존성 | `package.xml`에 rclpy, sensor_msgs, std_msgs만 있음 |

- 발행 토픽 이름·타입은 관제 대시보드(`monitoring/web/app.js`)가 이미 구독 중이므로 **바꾸면 안 되는 인터페이스**다.
- 시뮬의 게이지 모델(`simulation/models/gauge/model.sdf`)은 **어두운 패널 + 밝은 원판**뿐이라 바늘도 눈금도 없다. → 알고리즘 공부와 별개로 **게이지 모델 보강**이 선행 과제다 (6절).

### 토픽 이름은 폴더가 아니다

`/camera/image`, `/inspection/gauge`처럼 슬래시로 구분된 이름은 **ROS 2 토픽 이름**(네임스페이스)이지 디렉터리가 아니다. 저장소에 `camera/`, `inspection/` 폴더는 없다. 각 이름이 정해지는 곳은 다음과 같다.

| 토픽 | 정해지는 곳 | 설명 |
|------|-------------|------|
| `/camera/image` | `simulation/config/ros_gz_bridge.yaml` | Go2 모델(`simulation/models/go2/model.sdf`)의 카메라 센서가 `<topic>camera</topic>`으로 Gazebo 토픽 `/camera`를 내고, 브리지가 이를 ROS 토픽 `/camera/image`로 변환 |
| `/camera/camera_info` | 같은 브리지 파일 | 카메라 내부 파라미터(K 행렬) |
| `/inspection/gauge` | `gauge_ocr/gauge_ocr_node.py` | 우리 노드가 `create_publisher`로 직접 만드는 이름. `monitoring/web/app.js`가 이 이름으로 구독 |
| `/mission/status`, `/odom` | `patrol_path_node.py`, 브리지 | 정차 판정에 쓰려면 구독 |

`ros2 topic list`로 실행 중인 토픽을 볼 수 있고, `ros2 topic info -v /camera/image`로 누가 발행·구독하는지 확인할 수 있다.

---

## 2. 공통 기반 학습 (thermal_fusion과 공유)

### 2.1 ROS 2 Jazzy (Python, rclpy)

- [ ] 노드 생명주기: `rclpy.init` → `Node` 생성 → `spin` → `destroy_node` → `shutdown`
- [ ] 구독/발행 생성, 콜백 구조, 타이머
- [ ] **파라미터**: `declare_parameter` / `get_parameter`, 런치에서 넘기기, `ros2 param set`으로 런타임 튜닝
  - 임계값(알람 기준, 이진화 threshold 등)은 전부 파라미터로 빼야 실험이 편하다
- [ ] **QoS**: 이미지 토픽은 `qos_profile_sensor_data`(BEST_EFFORT) 권장. RELIABLE + depth 10이면 지연이 쌓인다
- [ ] `use_sim_time`: 시뮬 시간(`/clock`) 기준 스탬프. 런치에 이미 `True`
- [ ] ament_python 패키지 구조: `setup.py`의 `data_files`(launch, config 설치), `entry_points`, `package.xml` 의존성

참고: https://docs.ros.org/en/jazzy/Tutorials.html (Beginner → Client libraries, Python)

### 2.2 cv_bridge + OpenCV

- [ ] `sensor_msgs/Image` → numpy: `CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")`
  - Gazebo 카메라는 보통 `rgb8`로 들어오므로 `bgr8`로 받아 OpenCV 순서에 맞춘다
- [ ] 반대 방향 `cv2_to_imgmsg`: 디버그 오버레이를 토픽으로 낼 때 (`header` 복사 잊지 말 것)
- [ ] OpenCV 기초: `cvtColor`, `threshold`/`adaptiveThreshold`, 모폴로지, `findContours`, 블러
- [ ] 설치: `sudo apt install ros-jazzy-cv-bridge python3-opencv`

### 2.3 Gazebo Harmonic 카메라

- [ ] Go2 모델(`simulation/models/go2/model.sdf`) 카메라 스펙

  | 센서 | 해상도 | HFOV | 주기 | 포맷 | 위치 (head 링크 기준) |
  |------|--------|------|------|------|------|
  | `camera` | 640×480 | 1.396 rad (80°) | 15 Hz | RGB | (0.28, 0, 0.05) |

- [ ] 초점거리 계산: `fx = (W/2) / tan(HFOV/2)` → 약 380 px
- [ ] `/camera/camera_info`(sensor_msgs/CameraInfo)의 `K` 행렬. 브리지에 이미 매핑되어 있음
- [ ] SDF 모델 구조: `<link>` → `<visual>` → `<geometry>`, `<material>`, `<pose>`. 게이지 모델 보강 시 필요
- [ ] `GZ_SIM_RESOURCE_PATH`로 모델 경로 잡는 방식 (통합 런치에서 설정)

참고: https://gazebosim.org/docs/harmonic , https://sdformat.org/spec

### 2.4 개발·디버깅 도구

- [ ] `ros2 topic list / echo / hz / info -v` (QoS 확인)
- [ ] `rqt_image_view`: 이미지 토픽 눈으로 확인
- [ ] `ros2 bag record /camera/image /camera/camera_info /odom /tf /mission/status`
  - 시뮬 없이 알고리즘만 반복 개발할 때 필수. 한 번 녹화 후 `ros2 bag play`
- [ ] 프레임을 PNG로 덤프 → 스크립트/노트북에서 OpenCV 튜닝 → 노드에 이식하는 흐름
- [ ] `colcon build --symlink-install --packages-select gauge_ocr`
- [ ] `colcon test` + pytest: 영상처리 함수는 노드와 분리한 순수 함수로 두고 PNG 입력으로 단위 테스트

### 2.5 실행 환경 결정 — macOS에서는 Docker

내 개발 머신은 **macOS (Apple Silicon, arm64)** 다. ROS 2 Jazzy는 macOS 바이너리가 없고(Tier 3, 소스 빌드만), Nav2·`ros_gz`·`cv_bridge`·`gz_ros2_control`은 Mac 빌드가 아예 없다.
따라서 루트 README의 apt 목록은 **네이티브 Ubuntu 24.04 PC가 있을 때만** 쓰고, Mac에서는 **Docker(Ubuntu 24.04 arm64 컨테이너)** 로 돌린다.
자세한 정리는 [STUDY.md 0절](STUDY.md#0-실행-환경--macos--docker--gazebo-확인-창구).

- [x] Docker Desktop 설치 확인 (`docker --version`) — 확인됨 (29.x)
- [ ] 베이스 이미지 아키텍처 확인: 현재 `Dockerfile`은 `osrf/ros:jazzy-desktop`. arm64 태그가 없으면 에뮬레이션(RTF ≈ 0.5)이므로 이전 프로젝트처럼 `ros:jazzy` 공식 이미지 + 패키지 직접 설치로 바꾼다
  - 이전 프로젝트 검증: Humble 계열은 arm64에 `ros-gz-sim` 없음, **Jazzy + Harmonic만 arm64 네이티브 전체 스택 제공**
- [ ] `Dockerfile`에 이 패키지용 의존성 추가: `ros-jazzy-cv-bridge`, `ros-jazzy-rqt-image-view`, `ros-jazzy-ros2bag`, `ros-jazzy-foxglove-bridge`, (대시보드용) `ros-jazzy-rosbridge-server`
- [ ] 루트 README의 `docker run --network host -v /tmp/.X11-unix` 방식은 **Linux 전용**. Mac에서는 `-p 8765:8765 -p 9090:9090`로 포트를 명시하고 X11 소켓 마운트는 뺀다 → `docker-compose.yml`로 정리 (이전 프로젝트 `plant-robot-digital-twin/docker/docker-compose.yml` 참고)
- [ ] 저장소를 `-v $PWD:/workspace`로 마운트해 호스트에서 편집 → 컨테이너에서 `colcon build --symlink-install` (Dockerfile의 `COPY . /workspace`는 빌드 시점 스냅샷이라 편집이 반영되지 않는다)
- [ ] 컨테이너 안에서 `ros2 launch simulation full_system.launch.py world:=factory gui:=false` 정상 기동 확인 (`gui:=false` → `gz sim -s -r`, 서버만)
- [ ] `LIBGL_ALWAYS_SOFTWARE=1`로 카메라 센서가 CPU 렌더링되는지 확인: `ros2 topic hz /camera/image`가 15 Hz 근처인지, RTF가 얼마나 떨어지는지 기록
- [ ] `foxglove_bridge` 실행 후 Mac의 Foxglove Studio에서 `ws://localhost:8765` 접속
- [ ] Foxglove **Image 패널**(또는 noVNC 안 `rqt_image_view`)에서 `/camera/image` 확인
- [ ] 게이지 앞(factory `gauge_line1` (10, 8.7, 1.5))에서 프레임 캡처해 `docs/captures/`에 저장
- [ ] rosbag 하나 녹화해두기 (컨테이너 안 경로가 마운트된 저장소 밖이면 호스트로 안 나온다 → `docs/captures/` 아래나 마운트 경로에 저장)

### 2.6 Gazebo 화면 확인 창구 (셋 중 상황별 선택)

| 창구 | 언제 | 방법 | 비고 |
|------|------|------|------|
| **Foxglove Studio** (주력) | 개발 중 토픽·이미지·TF 확인 | 컨테이너에서 `ros2 launch foxglove_bridge foxglove_bridge_launch.xml` → Mac 앱에서 `ws://localhost:8765` | Mac GPU 렌더링이라 빠름. Gazebo 월드 자체(벽·메쉬)는 안 보이고 ROS 토픽만 보임 |
| **noVNC** | 월드 배치·물리 확인, 스크린샷 | compose에 `theasp/novnc` 컨테이너 추가, sim 컨테이너 `DISPLAY=novnc:0.0` → 브라우저 `http://localhost:8080/vnc.html`, 컨테이너에서 `gz sim -g` | 소프트웨어 렌더링 + amd64 에뮬레이션이라 느림. 대시보드 `http.server 8080`과 포트 충돌 → 하나는 다른 포트로 |
| **관제 대시보드** | 시연·보고서 캡처 | `rosbridge_server`(9090) + `monitoring/web` | 수현 담당 |

- `rqt_image_view`도 X 창이 필요하므로 noVNC 안에서만 뜬다. 평소엔 Foxglove Image 패널로 대체.
- XQuartz X11 포워딩은 Gazebo OGRE2와 궁합이 나빠 이전 프로젝트에서 noVNC로 대체했다.

---

## 3. 접근 방식 선택

| 방식 | 내용 | 장점 | 단점 |
|------|------|------|------|
| **A. 아날로그 바늘 각도 판독** (권장) | 원판 검출 → 바늘 직선 검출 → 각도 → 값 매핑 | OpenCV만으로 가능, 가볍고 시뮬에 잘 맞음 | 눈금·바늘 모델링 필요, 캘리브레이션 필요 |
| B. 디지털 숫자 OCR | 7-세그먼트/숫자 텍스처를 OCR | "OCR"이라는 이름에 부합 | Tesseract/EasyOCR 등 무거운 의존성, 렌더링 품질에 민감 |
| C. 딥러닝 검출 | YOLO 등으로 게이지·바늘 검출 | 실제 환경 확장성 | 학습 데이터 필요, 프로젝트 범위 초과 |

시뮬 데모 기준으로 **A를 기본**으로 하고, 시간이 남으면 B를 붙이는 순서를 권장한다.
(디지털 게이지 하나를 추가 모델로 만들면 "OCR" 시연도 가능)

---

## 4. 파이프라인과 공부할 항목

### 4.1 ROI(게이지 영역) 찾기

두 가지 방법이 있고, 시뮬에서는 **기하 방식이 훨씬 안정적**이다.

**(가) 기하 투영 방식** — 게이지 위치를 아니까 픽셀로 투영
- [ ] `tf2`: `/tf`에서 `map → base_link → camera` 변환 얻기 (`tf2_ros.Buffer`, `lookup_transform`)
- [ ] 핀홀 카메라 모델: 3D 점 → `K · [R|t] · X` → 픽셀. `/camera/camera_info`의 `K` 사용
- [ ] 게이지 월드 좌표: factory `gauge_line1`(10, 8.7, 1.5), `gauge_line2`(-8, 8.7, 1.5), corridor `gauge_corridor`(15, 1.55, 1.4)
- [ ] 투영된 중심 주변으로 ROI 박스 자르기 (거리에 따라 크기 조절)
- 주의: 카메라 프레임 TF가 실제로 발행되는지 확인 필요(`robot_state_publisher`가 URDF 기준으로 발행). 없으면 head 링크 기준 고정 오프셋(0.28, 0, 0.05)으로 계산

**(나) 영상 기반 방식** — 밝은 원판 찾기
- [ ] `cv2.HoughCircles` (`dp`, `minDist`, `param1/2`, `minRadius/maxRadius` 튜닝법)
- [ ] 색 기반 세그멘테이션: HSV 변환 후 밝은 원판/어두운 패널 마스크 → `findContours` → `minEnclosingCircle`
- [ ] 정면이 아닐 때 원이 타원으로 보임 → `fitEllipse` 또는 호모그래피로 정면화

### 4.2 바늘 검출

- [ ] 원판 내부만 마스킹 후 바늘 색(빨강/검정) 이진화
- [ ] `cv2.HoughLinesP`로 직선 후보 → 원 중심을 지나는 가장 긴 선 선택
- [ ] 대안: **극좌표 변환**(`cv2.warpPolar`) 후 각도별 어두운 픽셀 합의 최댓값 = 바늘 각도. 노이즈에 강하고 구현 단순
- [ ] 대안 2: 바늘 마스크의 `cv2.moments` / PCA로 주축 방향
- [ ] `atan2`로 각도 계산, 0~360° 정규화, 이미지 좌표계(y 아래 방향) 주의

### 4.3 각도 → 값 매핑 (캘리브레이션)

- [ ] 게이지 사양 정의: 최소값 각도 θ_min, 최대값 각도 θ_max, 값 범위 [v_min, v_max]
  - 보통 아날로그 게이지는 7시(225°)에서 5시(-45°)까지 270° 스윕
- [ ] 선형 보간: `v = v_min + (θ - θ_min) / (θ_max - θ_min) * (v_max - v_min)`
- [ ] 각도 랩어라운드 처리 (θ_max < θ_min인 경우)
- [ ] 이 값들을 YAML 파라미터(`config/gauge_<world>.yaml`)로 분리

### 4.4 후처리·알람

- [ ] 이동평균 / 중앙값 필터로 프레임 간 흔들림 제거
- [ ] 정차 중일 때만 판독: `/mission/status`가 `goto:gauge_*`이고 `/odom` 속도 ≈ 0일 때
- [ ] 알람 임계(`alarm_min`, `alarm_max`) 파라미터. 초과 시 로그 + (필요하면) 추가 토픽
- [ ] 판독 실패(ROI 없음, 바늘 못 찾음) 시 NaN 또는 마지막 값 유지 정책 결정 → 관제 표시와 맞춰야 함

### 4.5 디버그·검증

- [ ] 검출 원·바늘·값을 그린 오버레이 이미지 `/inspection/gauge_debug` 발행 (보고서 캡처용)
- [ ] **정답 값 확보**: 시뮬이라 바늘 각도를 우리가 정한다 → 여러 각도로 모델을 바꿔가며 판독값 비교 → 오차 통계
- [ ] 거리·각도(정면/사선)·조명 변화에 따른 성공률 표 → REPORT.md 3절 "게이지 판독 오차"

---

## 5. 참고 자료

- OpenCV 튜토리얼: Hough Circle/Line, Contours, Geometric Transformations(warpPerspective, warpPolar) — https://docs.opencv.org/4.x/
- "Analog gauge reader" 계열 오픈소스 예제 (Intel OpenVINO 샘플 `analog-gauge-reader`, GitHub `gauge reader opencv` 검색) — 파이프라인 구조 참고
- 핀홀 카메라 모델·내부 파라미터: OpenCV `calib3d` 문서, `sensor_msgs/CameraInfo` 메시지 정의
- tf2 Python 튜토리얼: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/
- (선택, 디지털 OCR) Tesseract `pytesseract`, EasyOCR, PaddleOCR 비교

---

## 6. 준비 작업 (시뮬·팀 협업)

- [ ] 게이지 모델에 **눈금 + 바늘** 추가. 두 가지 방법:
  1. `<visual>`에 얇은 박스(바늘)를 원판 중심에 붙이고 `<pose>`의 yaw로 각도 지정 → 각도를 SDF에서 바로 바꿀 수 있어 실험에 유리
  2. 눈금·바늘이 그려진 PNG를 `<material><pbr><albedo_map>` 텍스처로 입힘 → 보기 좋지만 값 바꾸려면 이미지를 다시 만들어야 함
  - 권장: 눈금은 텍스처, 바늘은 별도 박스 visual
- [ ] 바늘 색을 배경과 확실히 구분(빨강) — 이진화가 쉬워짐
- [ ] 게이지 높이(1.5 m)와 카메라 높이(로봇 0.4 m + head 0.05 m) 차이가 크다 → 사선 뷰. 정차점에서 원판이 충분히 크게 잡히는지 확인하고, 필요하면 게이지 높이나 정차 거리 조정을 태우·시뮬 담당과 상의
- [ ] 디지털 OCR 시연 시 숫자 텍스처가 있는 두 번째 게이지 모델 추가
- [ ] 태우(patrol_path): 게이지 정차점 좌표·정차 시간(dwell 5 s) 맞추기
- [ ] 수현(monitoring): `/inspection/gauge` 값 단위·범위, 알람 표시 방식, 디버그 이미지 토픽 노출 여부
- [ ] 보고서: REPORT.md "게이지 판독 오차" 지표 정의 (정답 각도 대비 절대 오차 %)

---

## 7. 코드 설계 원칙과 수정 예정 사항

1. 영상처리 로직은 노드에서 분리한다 (`reader.py`에 `read_gauge(img_bgr, cfg) -> float` 순수 함수, 노드는 변환·발행만). PNG로 테스트 가능해진다.
2. 모든 임계값은 ROS 파라미터로. YAML을 `config/`에 두고 `setup.py` data_files로 설치.
3. 디버그 이미지 토픽을 만든다.
4. 시간 필터(이동평균, N프레임 연속 조건)로 판정 흔들림을 막는다.
5. `world` 파라미터를 선언해 corridor / factory 설정을 바꿀 수 있게 한다.

- `package.xml`: `cv_bridge`, `python3-opencv`, `tf2_ros`, `geometry_msgs`, `nav_msgs` 의존성 추가
- `setup.py`: `config/*.yaml` 설치 항목 추가
- 파일 구조(안):
  ```
  gauge_ocr/
    gauge_ocr_node.py   # ROS 입출력만
    reader.py           # ROI → 바늘 각도 → 값 (순수 함수)
    projector.py        # 3D 게이지 위치 → 픽셀 ROI
  config/
    gauge_corridor.yaml
    gauge_factory.yaml
  test/
    test_reader.py      # PNG 샘플로 각도·값 검증
  ```

---

## 8. 학습 순서 정리

0. 실행 환경(macOS + Docker) 정리 + Gazebo 확인 창구 — 완료 → [STUDY.md 0절](STUDY.md)
1. ROS 2 Python 기초 (노드·토픽·파라미터·런치) — 하루 → 학습 내용: [STUDY.md](STUDY.md)
2. cv_bridge로 `/camera/image` 받아 PNG 저장까지 — 반나절 → 학습 내용: [STUDY.md 2절](STUDY.md#2-cv_bridge로-이미지-받아-png-저장)
3. rosbag 녹화 + PNG 덤프로 오프라인 데이터셋 확보 — 반나절 → 학습 내용: [STUDY.md 3절](STUDY.md#3-rosbag-녹화와-오프라인-데이터셋)
4. 게이지 모델 보강 협의 (바늘·눈금) — 병행 → 학습 내용: [STUDY.md 4절](STUDY.md#4-게이지-모델-보강)
5. OpenCV 원 검출 → 바늘 검출 → 각도·값 매핑 순으로 오프라인 구현 → 학습 내용: [STUDY.md 5절](STUDY.md#5-opencv-원-검출--바늘-검출--각도값-매핑)
6. 파라미터화·디버그 토픽·단위 테스트 정리 후 통합 런치에서 검증
