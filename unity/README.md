# Unity 영상 기준 플랜트 배치

게이지 판독·가상 온도 측정·가스 노출 경보와 복귀가 연결된 최신 시연은 `Assets/Scenes/Plant_Inspection.unity`와 [INSPECTION.md](INSPECTION.md)를 사용한다.

참고: 사용자가 제공한 `sample_robot.mp4` (61.74초). 영상의 복도 → 공장 → 설비 점검 → 복도 복귀 구성을 제공 에셋으로 배치했다.

완성 씬은 `Assets/Scenes/Plant_VideoReference.unity`다. 기존 `Assets/OutdoorsScene.unity`를 복사해 작업했다. 이 폴더는 전체 Unity 프로젝트가 아니라 기존 과제 에셋에 추가하는 씬·스크립트·설정 묶음이다.

- Spot 로봇 1대, 복도의 기존 계단·경사로, 공장 내부 모델을 사용한다.
- 복도 끝과 공장 서쪽 출입구를 연결하고 두 지형의 바닥 높이를 맞췄다.
- 제공 `fac_gastank_1` 프리팹의 압력 설비·게이지·발전기, 팔레트 더미 4개를 바닥에 배치했다.
- 충돌 메시를 기준으로 공장 안쪽 칸막이와 설비를 피하는 시연 경로를 생성한다.
- 로봇 추적 카메라와 좌측 하단 고정 카메라, 실내 조명, 별도 바닥 재질을 설정했다.

과제 에셋과 `Assets/OutdoorsScene.unity`가 있는 Unity 프로젝트에 이 폴더의 `Editor`, `Scripts`, `Scenes`, `DoosanReference`, `object`를 각각 `Assets` 아래로 복사한다. 참조 연결을 유지하려면 `.meta` 파일도 함께 복사한다. 원본 모델·프리팹·애니메이션이 없거나 GUID가 달라지면 누락된 참조를 먼저 연결해야 한다.

Unity에서 씬을 열고 **Play**를 누른다. `Space`는 일시정지, `Tab`은 자동/수동 전환, `W/S`는 전후 이동, `A/D`는 회전, `V`는 작은 카메라 표시 전환이다. 기본 이동 속도는 1.3 m/s다.

`Scripts`의 C#을 Unity의 `Assets/Scripts`에, `Editor`의 C#을 `Assets/Editor`에 넣으면 **Doosan → Build Video Reference Scene**에서 재생성할 수 있다. 재생성은 기존 `Plant_VideoReference` 작업을 덮어쓰므로 씬을 직접 수정했다면 다른 이름으로 저장한다. 원본 에셋의 기존 경로와 Unity 6 HDRP, Input System 패키지가 필요하다. 씬·재질은 원본 과제 에셋을 참조하며 원본 에셋 자체는 이 폴더에 중복 저장하지 않는다.

영상 시연의 이동은 CharacterController와 제공 보행 애니메이션을 사용한다. 녹색 파티클은 점검 위치를 보여 주는 시각 효과다.

## ROS 2 연결

`Assets/Scenes/Plant_ROS2.unity`는 별도의 ROS 제어 씬이다. `/cmd_vel`을 받아 이동하고 `/odom`, `/tf`, `/clock`, `/camera/image`, `/camera/camera_info`, `/points`, `/imu`를 ROS 2로 보낸다. 이 씬에서는 Unity 자체 순찰이 개입하지 않는다. ROS 순찰 노드와 실행·검증 방법은 [ROS2.md](ROS2.md)를 참고한다. 실제 다리 역학 제어와 가스 농도 계산은 별도 구현 항목이다.
