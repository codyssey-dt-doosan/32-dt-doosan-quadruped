"""판독 결과 디버그 그림 (/inspection/gauge_debug용)."""

import math

import cv2
import numpy as np


def draw_overlay(bgr: np.ndarray, value: float, dbg: dict) -> np.ndarray:
    """판독 결과를 그린 복사본. 디버그 토픽·보고서 캡처용."""
    out = bgr.copy()
    d = dbg.get("dial")
    if d is not None:
        cv2.ellipse(out, ((d.cx, d.cy), (d.w, d.h), d.angle), (0, 255, 0), 2)
        phi = dbg.get("phi", float("nan"))
        if not math.isnan(phi):
            r = max(d.w, d.h) / 2
            a = math.radians(phi)                                  # 12시 기준 시계 방향
            tip = (int(d.cx + r * math.sin(a)), int(d.cy - r * math.cos(a)))
            cv2.line(out, (int(d.cx), int(d.cy)), tip, (255, 0, 255), 2)
        org = (int(d.cx - d.w / 2), max(int(d.cy - d.h / 2) - 8, 15))
    else:
        org = (10, 25)
    text = "gauge: ---" if math.isnan(value) else f"gauge: {value:.2f}"
    cv2.putText(out, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
    return out
