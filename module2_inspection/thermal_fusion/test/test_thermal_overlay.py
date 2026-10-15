"""열지도 합성과 박스 그리기."""

import numpy as np

from thermal_fusion.hotspot import Blob
from thermal_fusion.overlay import draw_blobs, fit_to, fuse


def test_fuse_keeps_rgb_outside_valid():
    rgb = np.full((4, 4, 3), 50, np.uint8)
    temp = np.full((4, 4), 90.0, np.float32)
    valid = np.zeros((4, 4), np.uint8)
    valid[:2] = 255
    out = fuse(rgb, temp, valid, 20.0, 90.0)
    assert (out[2:] == 50).all()
    assert not (out[:2] == 50).all()


def test_draw_blobs_red_when_alert():
    img = np.zeros((480, 640, 3), np.uint8)
    blob = Blob(100, 100, 20, 20, 400, 85.0, 110.0, 110.0)
    out = draw_blobs(img, [blob], np.eye(3), alert=True)
    assert tuple(out[100, 100]) == (0, 0, 255)


def test_fuse_same_lo_hi_has_no_nan():
    rgb = np.full((4, 4, 3), 50, np.uint8)
    temp = np.full((4, 4), 60.0, np.float32)
    out = fuse(rgb, temp, np.full((4, 4), 255, np.uint8), 60.0, 60.0)
    assert out.shape == rgb.shape


def test_fit_to_resizes_only_when_size_differs():
    img = np.zeros((240, 320, 3), np.uint8)
    assert fit_to(img, 640, 480).shape == (480, 640, 3)
    same = np.zeros((480, 640, 3), np.uint8)
    assert fit_to(same, 640, 480) is same
