"""과열 영역(블롭) 검출. 입력은 to_temperature 결과."""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Blob:
    x: int
    y: int
    w: int
    h: int
    area: int
    t_max: float
    cx: float
    cy: float


def find_hot_blobs(temp: np.ndarray, threshold: float, min_area: int = 20) -> list:
    mask = (temp >= threshold).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))     # 점 노이즈 제거
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
    blobs = []
    for i in range(1, n):                                                          # 0은 배경
        x, y, w, h, area = stats[i]
        if area < min_area:
            continue
        t_max = float(temp[labels == i].max())
        blobs.append(Blob(int(x), int(y), int(w), int(h), int(area), t_max, *map(float, cents[i])))
    return sorted(blobs, key=lambda b: b.t_max, reverse=True)
