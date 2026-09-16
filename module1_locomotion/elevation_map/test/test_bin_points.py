import numpy as np

from elevation_map.elevation_map_node import bin_points


def test_bin_points():
    # 센서 높이 0.5, 4m 그리드, 0.1m 셀 → 40x40, 로봇 중앙 (20,20)
    pts = np.array([
        [1.0, 0.0, -0.5],   # 전방 1m 바닥 → 높이 0
        [1.0, 0.0, 0.3],    # 같은 셀, 장애물 윗면 → 최대값 0.8
        [-1.95, 1.95, 0.0], # 모서리 안쪽 (0,39)
        [5.0, 0.0, 0.0],    # 범위 밖 → 무시
    ])
    g = bin_points(pts, 0.1, 4.0, 0.5)
    assert g.shape == (40, 40)
    assert np.isclose(g[20, 30], 0.8)
    assert np.isclose(g[39, 0], 0.5)
    assert np.isnan(g[20, 20])
    assert np.isnan(bin_points(np.empty((0, 3)), 0.1, 4.0, 0.5)).all()
    assert np.isnan(bin_points(pts, 0.1, 4.0, 0.5, min_range=1.5)[20, 30])  # 1m 앞 포인트 제외


if __name__ == "__main__":
    test_bin_points()
    print("ok")
