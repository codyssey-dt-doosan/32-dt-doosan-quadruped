# gauge_ocr

담당: **운학**

카메라 영상에서 게이지 ROI를 잡고 지침/숫자를 판독한다.

```bash
ros2 launch gauge_ocr gauge_ocr.launch.py
```

| 토픽 | 방향 | 내용 |
|------|------|------|
| `/camera/image` | 구독 | 카메라 영상 |
| `/odom` | 구독 | 정차 판정 (속도 < `stop_speed`) |
| `/inspection/gauge` | 발행 (Float32, 0.5 s) | 최근 판독값 중앙값. 판독값이 없으면 발행 안 함 |
| `/inspection/gauge_debug` | 발행 (Image) | 검출 원판·바늘·값 오버레이 (구독자 있을 때만) |

- 파라미터: `config/gauge_<world>.yaml` (게이지 각도 범위·값 범위·필터)
- 판독 로직: `gauge_ocr/reader.py` (ROS 없는 순수 함수)
- 테스트: `python3 -m pytest test -q` (패키지 폴더에서). 합성 게이지와 실제 눈금판 텍스처 `test/data/dial_0_10bar.png`(0~10 bar, 바늘은 테스트에서 그려 넣음) 사용

학습·준비 계획: [STUDY_PLAN.md](STUDY_PLAN.md) · 학습 내용 정리: [STUDY.md](STUDY.md)
