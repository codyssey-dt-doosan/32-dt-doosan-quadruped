"""과열 블롭 검출 (STUDY 5.6)."""

import numpy as np

from thermal_fusion.hotspot import find_hot_blobs


def _scene():
    t = np.full((240, 320), 25.0, np.float32)
    t[100:115, 150:170] = 85.0                       # 20×15 블롭
    t[10, 10] = 90.0                                 # 한 픽셀 노이즈
    return t


def test_single_blob_ignores_noise():
    blobs = find_hot_blobs(_scene(), 60.0)
    assert len(blobs) == 1
    b = blobs[0]
    assert (b.x, b.y, b.w, b.h) == (150, 100, 20, 15)
    assert b.t_max == 85.0
    assert (b.cx, b.cy) == (159.5, 107.0)


def test_sorted_by_temperature():
    t = _scene()
    t[200:215, 20:40] = 95.0
    assert [b.t_max for b in find_hot_blobs(t, 60.0)] == [95.0, 85.0]


def test_nothing_hot():
    assert find_hot_blobs(np.full((240, 320), 25.0, np.float32), 60.0) == []
