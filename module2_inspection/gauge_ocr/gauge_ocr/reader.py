"""게이지 판독 순수 함수. ROS 의존성 없음 → PNG로 단위 테스트 가능."""

from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass
class GaugeSpec:
    phi_min: float = 225.0   # 최소값 바늘 각도 (12시 기준 시계 방향, 도)
    sweep: float = 270.0     # 최소 → 최대 시계 방향 스윕 (도)
    v_min: float = 0.0
    v_max: float = 10.0


@dataclass
class Dial:
    cx: float
    cy: float
    w: float        # fitEllipse 결과 그대로 (지름 단위)
    h: float
    angle: float    # 도


# ---------- 1. 원판 찾기 ----------

def _best_ellipse(mask: np.ndarray, min_area: float, min_fill: float):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    best = None
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area or len(c) < 5:                           # fitEllipse는 점 5개 이상 필요
            continue
        (cx, cy), (w, h), ang = cv2.fitEllipse(c)
        fill = area / (math.pi * w * h / 4)                         # 타원 면적 대비 채움률 → 사각 물체 배제
        if fill >= min_fill and (best is None or area > best[0]):
            best = (area, Dial(cx, cy, w, h, ang))
    return None if best is None else best[1]


def find_dial(bgr: np.ndarray, min_area: float = 100.0, min_fill: float = 0.8) -> Dial | None:
    """① 어두운 남색 패널을 찾고 ② 그 안에서 패널보다 밝은 원(원판)을 찾는다.

    시뮬에서 원판 밝기(V ≈ 100)는 조명 받은 벽(V ≈ 120)보다 어둡다 → 화면 전체 밝기 임계로는 못 찾는다.
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    panel = cv2.inRange(hsv, (90, 20, 20), (130, 255, 90))          # 파랑 계열 H, 약간의 채도, 어두움
    panel = cv2.morphologyEx(panel, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))   # 원판 구멍 메우기
    contours, _ = cv2.findContours(panel, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    best = None
    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        if w * h < 2 * min_area:
            continue
        roi = gray[y:y + h, x:x + w]
        _, m = cv2.threshold(roi, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)   # 패널 안: 밝은 쪽 = 원판
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))       # 바늘·허브 구멍 메우기
        d = _best_ellipse(m, min_area, min_fill)
        if d is not None and (best is None or d.w * d.h > best.w * best.h):
            best = Dial(d.cx + x, d.cy + y, d.w, d.h, d.angle)
    return best


# ---------- 2. 타원 → 원 정면화 ----------

def rectify(bgr: np.ndarray, d: Dial, out_r: int = 100) -> np.ndarray:
    """타원 원판을 반지름 out_r 원으로 펴서 (2·out_r)² 이미지로 잘라 낸다."""
    a = math.radians(d.angle)
    u = np.array([math.cos(a), math.sin(a)])       # w 축 방향
    v = np.array([-math.sin(a), math.cos(a)])      # h 축 방향
    c = np.array([d.cx, d.cy])
    src = np.float32([c + d.w / 2 * u, c + d.h / 2 * v, c - d.w / 2 * u])
    oc = np.array([out_r, out_r], float)
    dst = np.float32([oc + out_r * u, oc + out_r * v, oc - out_r * u])
    M = cv2.getAffineTransform(src, dst)
    return cv2.warpAffine(bgr, M, (2 * out_r, 2 * out_r), flags=cv2.INTER_LINEAR)


# ---------- 3. 바늘 각도 ----------

def needle_mask(bgr: np.ndarray) -> np.ndarray:
    """빨강은 H가 0 근처와 180 근처 양쪽에 걸친다 → 두 범위 OR."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m1 = cv2.inRange(hsv, (0, 100, 60), (10, 255, 255))
    m2 = cv2.inRange(hsv, (170, 100, 60), (180, 255, 255))
    return cv2.bitwise_or(m1, m2)


def needle_phi(face: np.ndarray, r_in: float = 0.25, r_out: float = 0.85,
               n_angles: int = 720) -> tuple[float, float]:
    """정면화된 원판 이미지 → (φ 도, 신뢰도 0~1).

    warpPolar 출력: 행 = 각도(0~360, 이미지 x축 기준 시계 방향), 열 = 반지름.
    """
    r = face.shape[0] / 2
    mask = needle_mask(face)
    polar = cv2.warpPolar(mask, (int(r), n_angles), (r, r), r, cv2.WARP_POLAR_LINEAR)
    band = polar[:, int(r_in * r): int(r_out * r)]                  # 중심(허브)·테두리 제외
    score = band.sum(axis=1).astype(np.float64)
    if score.max() <= 0:
        return float("nan"), 0.0
    score = np.convolve(np.r_[score[-5:], score, score[:5]], np.ones(11) / 11, "valid")  # 원형 스무딩
    i = int(np.argmax(score))
    theta_img = i * 360.0 / n_angles                                 # x축 기준 시계 방향
    phi = (theta_img + 90.0) % 360.0                                 # 12시 기준 시계 방향
    conf = float(score[i] / (255.0 * band.shape[1]))
    return phi, conf


# ---------- 4. 각도 → 값 ----------

def phi_to_value(phi: float, spec: GaugeSpec) -> float:
    t = ((phi - spec.phi_min) % 360.0) / spec.sweep
    if t > 1.0:                                     # 스윕 밖(죽은 구간) → 가까운 끝으로
        t = 1.0 if (t - 1.0) * spec.sweep < (360.0 - spec.sweep) / 2 else 0.0
    return spec.v_min + t * (spec.v_max - spec.v_min)


def read_gauge(bgr: np.ndarray, spec: GaugeSpec) -> tuple[float, dict]:
    """전체 파이프라인. 실패하면 (nan, 디버그 정보)."""
    dbg: dict = {}
    d = find_dial(bgr)
    if d is None:
        return float("nan"), dbg
    dbg["dial"] = d
    face = rectify(bgr, d)
    phi, conf = needle_phi(face)
    dbg.update(face=face, phi=phi, conf=conf)
    if math.isnan(phi) or conf < 0.05:
        return float("nan"), dbg
    return phi_to_value(phi, spec), dbg
