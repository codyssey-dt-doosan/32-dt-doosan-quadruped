# elevation_map

담당: **도훈**

`/points`(LiDAR PointCloud2, 센서 프레임)를 로봇 중심 XY 그리드로 빈닝해 셀별 최대 높이를 낸다.

```bash
ros2 launch elevation_map elevation_map.launch.py
```

## 출력 `/elevation_map` (std_msgs/Float32MultiArray)

- `layout.dim[0]`=y(행), `layout.dim[1]`=x(열). 기본 40×40, 셀 0.1 m, 로봇이 (20,20), 전방 = 열 증가 방향
- 값 = 셀 내 최대 높이(m), **0 = 지면**. 빈 셀 = NaN
- 셀 (r, c) 의 로봇 기준 좌표: x = (c − 20)·0.1, y = (r − 20)·0.1
- 라이다 하향 빔 한계로 바닥은 전방 약 1.0 m부터 채워짐. 그 안쪽은 NaN(자기 몸체 제외 `min_range` 0.5 m)

## 파라미터

| 이름 | 기본 | 뜻 |
|---|---|---|
| `resolution` | 0.1 | 셀 크기(m) |
| `size` | 4.0 | 그리드 한 변(m) |
| `sensor_height` | 0.56 | 라이다 지상고(m). 몸체 정착 z 0.36 + `model.sdf` 라이다 z 0.20 |
| `min_range` | 0.5 | 이 반경 안 포인트 무시(자기 몸체) |

몸체 roll/pitch 보정 없음. 다리 마찰 0으로 주행 중 몸체는 수평 유지됨.
