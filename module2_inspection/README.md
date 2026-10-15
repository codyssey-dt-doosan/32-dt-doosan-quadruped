# module2_inspection

담당: **운학(게이지·열화상) + 태우(순찰)**

점검: OCR, 열화상 융합, 순찰 경로

## 운학 파트 인터페이스 (게이지·열화상)

설계: [docs/inspection_interface_design.md](docs/inspection_interface_design.md)

| 토픽 | 타입 | 발행 | 쓰는 곳 (회의에서 확정) |
|------|------|------|------|
| `/inspection/gauge` | Float32 | gauge_ocr, 0.5 s | 관제 |
| `/inspection/gauge_state` | String | gauge_ocr, 0.5 s | 관제·통합 |
| `/inspection/gauge_viewpoints` | PoseArray (latched) | gauge_ocr | 순찰 웨이포인트 참고 |
| `/inspection/gauge_debug` | Image | gauge_ocr (구독자 있을 때, publish_debug) | 디버그 |
| `/inspection/thermal_alert` | Bool | thermal_fusion, 0.5 s | 관제 |
| `/inspection/thermal_goal` | PoseStamped | thermal_fusion, 2 Hz | mpc_controller `goal_topics` (추가 시) |
| `/inspection/thermal_hotspots` | PoseArray (latched) | thermal_fusion | 관제 지도 |
| `/inspection/thermal_overlay` | Image | thermal_fusion | 디버그 |
| `/inspection/thermal_state` | String | thermal_fusion, 0.5 s | 관제·통합 |

상태 문자열은 `"상태 | 칸 | 칸"` 형식이고 `inspection_common.state_msg.parse_state()`로 읽는다.
좌표는 모두 `map`, goal 비활성은 빈 `frame_id`.

```bash
# 테스트 (ROS 없이)
cd module2_inspection
PYTHONPATH=inspection_common python3 -m pytest inspection_common/test -q
PYTHONPATH=inspection_common:gauge_ocr python3 -m pytest gauge_ocr/test -q
PYTHONPATH=inspection_common:thermal_fusion python3 -m pytest thermal_fusion/test -q
```
