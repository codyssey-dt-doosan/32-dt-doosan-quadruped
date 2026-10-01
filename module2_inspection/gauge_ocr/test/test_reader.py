"""reader 순수 함수 테스트. 합성 게이지 이미지로 정답을 알고 검증한다."""

import math
import os

import cv2
import numpy as np
import pytest

from gauge_ocr.overlay import draw_overlay
from gauge_ocr.reader import find_dial, GaugeSpec, phi_to_value, read_gauge

PANEL = (76, 66, 56)    # 남색 패널 (BGR)
WALL = (88, 86, 84)     # 회색 벽
SPEC = GaugeSpec()


def synth_gauge(phi_deg, squash=1.0, r=50, center=(320, 200),
                dial=(230, 242, 242), bg=WALL, size=(640, 480)):
    """벽 + 패널 + 원판 + 빨간 한쪽 바늘 + 검은 허브. squash < 1이면 세로로 눌림."""
    big = 4                     # 크게 그려서 줄이면 안티앨리어싱
    w, h = size[0] * big, size[1] * big
    img = np.full((h, w, 3), bg, np.uint8)
    c = (center[0] * big, center[1] * big)
    rr = r * big
    half = int(rr * 1.45)
    cv2.rectangle(img, (c[0] - half, c[1] - half), (c[0] + half, c[1] + half), PANEL, -1)
    cv2.circle(img, c, rr, dial, -1, cv2.LINE_AA)
    a = math.radians(phi_deg)
    tip = (int(c[0] + 0.83 * rr * math.sin(a)), int(c[1] - 0.83 * rr * math.cos(a)))
    cv2.line(img, c, tip, (13, 13, 230), int(0.07 * rr), cv2.LINE_AA)
    cv2.circle(img, c, int(0.1 * rr), (25, 25, 25), -1, cv2.LINE_AA)
    img = cv2.resize(img, size, interpolation=cv2.INTER_AREA)
    if squash != 1.0:
        m = np.float32([[1, 0, 0], [0, squash, center[1] * (1 - squash)]])
        img = cv2.warpAffine(img, m, size, borderValue=bg)
    return img


def phi_of(value):
    return (SPEC.phi_min + (value - SPEC.v_min) / (SPEC.v_max - SPEC.v_min) * SPEC.sweep) % 360


@pytest.mark.parametrize("value", [0.0, 2.5, 3.3, 5.0, 7.5, 10.0])
def test_front_view(value):
    v, dbg = read_gauge(synth_gauge(phi_of(value)), SPEC)
    assert abs(v - value) < 0.1, (v, dbg.get("phi"))


@pytest.mark.parametrize("value", [1.0, 5.0, 9.0])
def test_oblique_view(value):
    v, _ = read_gauge(synth_gauge(phi_of(value), squash=0.75), SPEC)
    assert abs(v - value) < 0.15


@pytest.mark.parametrize("value", [2.0, 8.0])
def test_small_dial(value):
    v, _ = read_gauge(synth_gauge(phi_of(value), r=20), SPEC)
    assert abs(v - value) < 0.15


def test_dial_darker_than_wall():
    """조명 때문에 원판(회색 ~100)이 밝은 벽(~120)보다 어둡게 찍혀도 찾아야 한다."""
    img = synth_gauge(phi_of(4.0), dial=(98, 98, 95), bg=(121, 120, 118))
    v, _ = read_gauge(img, SPEC)
    assert abs(v - 4.0) < 0.15


def test_no_gauge():
    v, dbg = read_gauge(np.full((480, 640, 3), WALL, np.uint8), SPEC)
    assert math.isnan(v)
    assert "dial" not in dbg


def test_dial_center():
    d = find_dial(synth_gauge(0.0))
    assert abs(d.cx - 320) < 1.5 and abs(d.cy - 200) < 1.5
    assert abs(d.w / 2 - 50) < 2


def test_phi_to_value_wraparound():
    assert phi_to_value(225.0, SPEC) == pytest.approx(0.0)
    assert phi_to_value(0.0, SPEC) == pytest.approx(5.0)
    assert phi_to_value(135.0, SPEC) == pytest.approx(10.0)
    assert phi_to_value(170.0, SPEC) == pytest.approx(10.0)    # 죽은 구간, 최대 쪽
    assert phi_to_value(200.0, SPEC) == pytest.approx(0.0)     # 죽은 구간, 최소 쪽


def test_overlay_keeps_input():
    img = synth_gauge(phi_of(6.0))
    v, dbg = read_gauge(img, SPEC)
    out = draw_overlay(img, v, dbg)
    assert out.shape == img.shape and not np.array_equal(out, img)
    assert draw_overlay(img, float("nan"), {}).shape == img.shape


# ---------- 실제 눈금판 텍스처 (test/data/dial_0_10bar.png) ----------
# 0~10 bar, 0이 7시 반·5가 12시·10이 4시 반 → GaugeSpec 기본값과 같다. 바늘은 없어서 그려 넣는다.

DIAL_PNG = os.path.join(os.path.dirname(__file__), "data", "dial_0_10bar.png")


def textured_gauge(value, r=50, squash=1.0, brightness=1.0, bg=WALL, size=(640, 480)):
    """벽 + 패널 + 눈금판 텍스처 + 빨간 바늘. brightness < 1이면 원판을 어둡게 (조명 약함)."""
    face = cv2.imread(DIAL_PNG)
    c = face.shape[0] // 2
    a = math.radians(phi_of(value))
    tip = (int(c + 0.8 * c * math.sin(a)), int(c - 0.8 * c * math.cos(a)))
    cv2.line(face, (c, c), tip, (13, 13, 230), 14, cv2.LINE_AA)
    cv2.circle(face, (c, c), 18, (25, 25, 25), -1)
    face = (face.astype(np.float32) * brightness).astype(np.uint8)
    face = cv2.resize(face, (2 * r, 2 * r), interpolation=cv2.INTER_AREA)

    img = np.full((size[1], size[0], 3), bg, np.uint8)
    cx, cy = 320, 200
    half = int(r * 1.45)
    cv2.rectangle(img, (cx - half, cy - half), (cx + half, cy + half), PANEL, -1)
    mask = np.zeros((2 * r, 2 * r), np.uint8)
    cv2.circle(mask, (r, r), r - 1, 255, -1)
    roi = img[cy - r:cy + r, cx - r:cx + r]
    roi[mask > 0] = face[mask > 0]
    if squash != 1.0:
        m = np.float32([[1, 0, 0], [0, squash, cy * (1 - squash)]])
        img = cv2.warpAffine(img, m, size, borderValue=bg)
    return img


def test_dial_texture_exists():
    assert cv2.imread(DIAL_PNG) is not None


@pytest.mark.parametrize("value", [0.0, 1.5, 3.7, 5.0, 6.2, 8.8, 10.0])
def test_textured_front(value):
    v, _ = read_gauge(textured_gauge(value), SPEC)
    assert abs(v - value) < 0.05


@pytest.mark.parametrize("value", [2.0, 7.0])
def test_textured_oblique(value):
    v, _ = read_gauge(textured_gauge(value, squash=0.85), SPEC)
    assert abs(v - value) < 0.05


@pytest.mark.parametrize("value", [1.0, 4.5, 9.0])
def test_textured_small_and_dark(value):
    """원판 반지름 18 px, 원판이 밝은 벽보다 어두운 최악 조건."""
    img = textured_gauge(value, r=18, brightness=0.42, bg=(121, 120, 118))
    v, _ = read_gauge(img, SPEC)
    assert abs(v - value) < 0.15
