# ROS 2 통합 점검 순찰

`Plant_Inspection`은 게이지 영상 판독, 가상 열화상 온도 측정, 가스 농도·누적 노출 경보를 ROS 순찰에 연결한 씬이다. 기존 `Plant_ROS2` 이동 데모는 보존한다.

## 실행

Docker Desktop을 켜고 저장소 루트에서 다음을 실행한다.

```bash
docker compose -f unity/compose.yaml --profile inspection up -d --build
```

Unity의 `Assets/Scenes/Plant_Inspection.unity`를 열고 Play한다. 별도의 순찰 터미널 명령은 필요하지 않다. Compose의 `inspection` 서비스가 점검 노드와 통합 순찰 노드를 함께 켠다. 기존 `patrol_ros.py`, 수동 `/cmd_vel` 발행 노드는 먼저 종료한다. 명령 발행자는 통합 순찰 노드 하나여야 한다.

```bash
docker compose -f unity/compose.yaml logs -f inspection
# 종료
docker compose -f unity/compose.yaml --profile inspection down
```

두 컨테이너는 네트워크와 IPC를 공유한다. 같은 IP에서 ROS 노드를 발견한 뒤 Fast DDS의 공유 메모리로 실제 데이터가 오가므로 IPC도 공유해야 한다. 서버 포트는 기존처럼 localhost:9090만 공개한다.

## 동작

1. ROS에서 `/odom`을 보고 복도·경사로·공장 경유점을 추종한다. 가상 온도·가스 센서가 신선하지 않으면 정지한다.
2. `Inspect_Gauges`에서 멈추고 게이지 방향으로 회전한 뒤 12초간 점검한다.
3. Unity RGB 영상(640×480)에서 기존 `gauge_ocr.reader`가 패널·원판·빨간 바늘을 검출하고 각도를 0~10bar로 변환한다. 시연 게이지 정답은 6.2bar다. 값은 중앙값으로 묶으며 정답을 판독 노드에 주입하지 않는다. 판독에 실패하면 정상값을 만들어 내지 않고 점검 실패 상태로 정지한다.
4. 같은 카메라의 시야에서 표면 충돌 형상에 Raycast해 합성 온도 맵(80×60, `32FC1`, 단위 °C)을 만든다. 주변은 22°C, 지정 설비는 95°C다. ROS에서 유효 픽셀 최댓값을 계산하고 80°C 이상이면 과열 경보를 발행한다. 과열은 이 데모에서 기록·표시하며, 가스 경보가 긴급 복귀를 담당한다.
5. 가스 누출원과 로봇 사이 수평 거리에 따른 가우시안 농도장을 Unity에서 샘플링한다. ROS에서 Unity 시뮬레이션 시간을 이용해 농도를 적분해 `ppm·s`를 계산한다. 시뮬레이션 일시정지 시간은 누적하지 않고, 센서 공백은 임의로 보간하지 않는다.
6. 50ppm 또는 200ppm·s 이상이면 경보를 기억하고 2초 정지한 뒤, 이미 통과한 경유점을 역순으로 따라 시작점에 복귀한다. 농도가 낮아져도 복귀를 취소하지 않는다. 홈 도착 후 정지하며 Play를 새로 시작하면 새 임무로 초기화한다.

Unity 화면에 ROS에서 받은 판독값, 온도, 농도, 누적 노출량, 임무 상태와 합성 열화상 미리보기가 표시된다. `no fresh data`는 정상 판정이 아니라 데이터 대기/유실 상태다. 게이지 값은 점검 중 유효한 영상 판독이 있을 때만 표시한다.

## 토픽과 코드

| 토픽 | 방향 / 의미 |
|---|---|
| `/camera/image` | Unity → ROS, 실제 렌더링 RGB |
| `/thermal/image` | Unity → ROS, 합성 표면 온도 `32FC1` |
| `/gas/concentration` | Unity → ROS, 가상 농도 Float32(ppm) |
| `/inspection/gauge` | ROS 판독값 Float32(bar) |
| `/inspection/gauge_debug` | ROS 원판·바늘 검출 결과 영상 |
| `/inspection/temperature_c` | ROS 최댓값 온도 Float32(°C) |
| `/inspection/thermal_alert` | ROS 과열 Bool |
| `/gas/exposure`, `/gas/alert` | ROS 누적 노출량 Float32(ppm·s), 가스 경보 Bool |
| `/inspection/status` | ROS → Unity/순찰, 값·유효성·경보 JSON String |
| `/mission/status` | ROS → Unity/점검, 순찰·정렬·점검·복귀 상태 String |
| `/cmd_vel` | 통합 순찰 ROS 노드 → Unity |

- `Scripts/UnityPlantInspection.cs`: 센서 샘플링·열화상 표시·ROS 결과 표시.
- `Scripts/SimulatedHeatSurface.cs`: 설비 표면에 지정하는 가상 온도.
- `tools/inspection_ros.py`: 기존 게이지 판독 함수 재사용, 온도 처리, 가스 노출·경보 계산.
- `tools/inspection_patrol.py`: 단일 이동 명령 발행자. 정차·정렬·점검·역경로 복귀.
- `config/inspection.json`: 경보 기준과 데이터 유효 시간. 변경 후 `inspection` 서비스를 재시작한다.
- `config/inspection_route.json`: Unity 시작 원점 기준 ROS 경로. 게이지 정차 자세와 점검 시간 포함.

이 구성은 기존 `thermal_fusion_node.py`의 False 발행 틀이나 `source_seeking`·`return_to_home`의 미구현 노드를 실행하지 않는다. 통합 점검·순찰 노드가 이 씬의 기능을 담당한다. 기존 팀원 패키지 파일은 수정하지 않는다.

## 재생성·검증

프로젝트에 원본 제공 에셋과 `unity/`의 Scripts, Editor, Scenes, DoosanReference, object를 `.meta`와 함께 복사한다. **Doosan → ROS 2 → Create Inspection Scene**으로 이동 데모에서 점검 씬을 다시 만들 수 있다. 이 메뉴는 점검 씬·생성 에셋을 덮어쓰므로 직접 수정한 점검 씬은 먼저 다른 이름으로 저장한다. `Assets/DoosanReference/inspection_route.json`을 저장소 `unity/config/inspection_route.json`에도 복사한다.

```bash
# 노출 적분, 시간 초기화, 일시정지, 센서 공백, 경보 경계값 검사
docker compose -f unity/compose.yaml exec inspection python3 /workspace/unity-tools/test_inspection_logic.py
# Unity 순찰 시작 전에 실행하는 관측 전용 통합 검사 (이동 명령 발행 없음)
docker compose -f unity/compose.yaml exec inspection bash -lc \
  'source /opt/ros/jazzy/setup.bash && python3 /workspace/unity-tools/verify_inspection.py'
```

컨테이너 `/tmp/inspection-verification.json`, `/tmp/inspection-results.json`, `/tmp/inspection-patrol.json`에 결과를 기록하며 실제 ROS 입력에서 게이지·열화상 이미지를 캡처한다. 최초 경보 후 홈 복귀까지 관측해야 전체 검사가 완료된다.

온도·가스는 지정한 시나리오에서 생성한 합성 센서이며 열전달·유체 확산이나 실제 센서 보정은 구현하지 않는다. ppm과 경보값은 시연용 설정으로 가스 종류별 산업안전 기준을 나타내지 않는다. 다리 움직임은 기존 애니메이션이며 관절 토크 기반 보행이 아니다. 홈 복귀는 이미 통과한 고정 경로를 재사용하며 새 장애물을 우회하는 SLAM/Nav2 경로 계획은 포함하지 않는다.

## 이번 실행에서 확인한 결과

실제 Unity 렌더링 영상에서 게이지 약 6.22bar(정답 6.2bar), 합성 열화상 95°C, 최초 가스 경보 약 52.61ppm을 확인했다. 경보 후 역경로로 복귀해 시작점과 약 0.019m 거리에서 정지했다. 관측 전용 통합 검사 9개와 노출 계산 검사 5개가 통과했다. 결과는 `docs/validation/unity-inspection*.json`, 영상은 `docs/captures/unity-inspection-*.jpg`에 있다. 실행 중 재생 재시작과 짧은 센서 유실에 따른 정지·재개도 로그에 남아 있다.
