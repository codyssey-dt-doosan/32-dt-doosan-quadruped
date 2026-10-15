"""픽셀 → 온도 변환."""

import numpy as np
import pytest

from thermal_fusion.thermal import to_temperature


def test_l16_centi_kelvin():
    img = np.array([[35815]], np.uint16)            # 358.15 K
    assert to_temperature(img, "l16")[0, 0] == pytest.approx(85.0, abs=1e-3)


def test_l8_thermal_linear():
    img = np.array([[0, 30]], np.uint8)
    t = to_temperature(img, "l8_thermal", l8_min_k=253.15, l8_res=3.0)
    assert t[0].tolist() == pytest.approx([-20.0, 70.0], abs=1e-3)


def test_brightness_passthrough_and_gray():
    assert to_temperature(np.array([[200]], np.uint8), "brightness")[0, 0] == 200.0
    bgr = np.full((2, 2, 3), 100, np.uint8)
    out = to_temperature(bgr, "brightness")
    assert out.shape == (2, 2) and out.dtype == np.float32


def test_unknown_mode():
    with pytest.raises(ValueError):
        to_temperature(np.zeros((1, 1), np.uint8), "ir")
