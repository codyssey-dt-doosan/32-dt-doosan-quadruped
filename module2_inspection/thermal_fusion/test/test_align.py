"""열화상 → RGB 정렬 (STUDY 5.6 합성 검증)."""

import numpy as np
import pytest

from inspection_common.world_map import CameraSpec
from thermal_fusion.align import k_from_spec, map_points, thermal_to_rgb_h, warp_to_rgb

KR = k_from_spec(CameraSpec(0.28, 0.0, 0.05, 0.0, 1.396, 640, 480))
KT = k_from_spec(CameraSpec(0.28, 0.0, 0.02, 0.0, 1.0, 320, 240))


def project(k, X, Y, Z):
    """카메라 기준 점 (앞 X, 왼쪽 Y, 위 Z) → 픽셀."""
    return np.array([k[0, 0] * (-Y / X) + k[0, 2], k[1, 1] * (-Z / X) + k[1, 2]])


@pytest.mark.parametrize("Y,Z", [(0.0, 0.0), (0.3, -0.2), (-0.4, 0.25)])
def test_points_at_design_depth_align(Y, Z):
    h = thermal_to_rgb_h(KR, KT, baseline_z=0.03, depth=1.5)
    th = project(KT, 1.5, Y, Z)
    rgb = project(KR, 1.5, Y, Z - 0.03)                # RGB는 열화상보다 0.03 m 위
    assert map_points(th[None], h)[0] == pytest.approx(rgb, abs=1e-6)


def test_depth_mismatch_error():
    h = thermal_to_rgb_h(KR, KT, baseline_z=0.03, depth=1.5)
    th = project(KT, 0.4, 0.0, 0.0)
    rgb = project(KR, 0.4, 0.0, -0.03)
    err = np.linalg.norm(map_points(th[None], h)[0] - rgb)
    assert err == pytest.approx(20.98, abs=0.05)       # 381.46 · 0.03 · (1/0.4 − 1/1.5)


def test_warp_valid_region_size():
    h = thermal_to_rgb_h(KR, KT)
    _, valid = warp_to_rgb(np.zeros((240, 320), np.float32), h)
    ys, xs = np.nonzero(valid)
    assert xs.max() - xs.min() + 1 == pytest.approx(417, abs=2)
    assert ys.max() - ys.min() + 1 == pytest.approx(313, abs=2)
