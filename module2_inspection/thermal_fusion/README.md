# thermal_fusion

담당: **운학**

RGB와 열화상을 정렬·융합해 과열 영역을 표시한다.

```bash
ros2 launch thermal_fusion thermal_fusion.launch.py
```

## 토픽

| 토픽 | 방향 | 내용 |
|------|------|------|
| `/thermal/image` | 구독 (Image) | 열화상 (지금 시뮬은 흑백 밝기 → `thermal_mode: brightness`) |
| `/camera/image` | 구독 (Image) | 겹쳐 그리기용 RGB |
| `/odom` | 구독 (Odometry) | 로봇 위치(map)·속도 |
| `/inspection/thermal_alert` | 발행 (Bool, 0.5 s) | 점검 위치에서 확정된 과열이 있으면 True |
| `/inspection/thermal_goal` | 발행 (PoseStamped, 2 Hz) | 점검할 지점이 있을 때 접근 위치. 없으면 빈 frame_id |
| `/inspection/thermal_hotspots` | 발행 (PoseArray, latched) | 모든 지점 위치 |
| `/inspection/thermal_overlay` | 발행 (Image) | RGB + 열지도 + 박스 (구독자 있을 때만) |
| `/inspection/thermal_state` | 발행 (String, 0.5 s) | `상태 \| hotspot_N (x, y) \| 값 \| goal/사유` |

학습·준비 계획: [STUDY_PLAN.md](STUDY_PLAN.md) · 학습 내용 정리: [STUDY.md](STUDY.md)
