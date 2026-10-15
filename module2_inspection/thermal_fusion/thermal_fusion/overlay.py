"""RGB 위에 정렬된 열지도와 과열 박스를 그린다."""

import cv2
import numpy as np

from thermal_fusion.align import map_points


def fuse(rgb: np.ndarray, temp_warped: np.ndarray, valid: np.ndarray, t_lo: float, t_hi: float,
         alpha: float = 0.45) -> np.ndarray:
    """t_lo~t_hi를 컬러맵으로 칠해 유효 영역에만 반투명 합성."""
    span = max(t_hi - t_lo, 1e-6)                       # lo == hi 설정이어도 NaN이 나지 않게
    norm = np.clip((temp_warped - t_lo) / span, 0, 1)
    color = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_INFERNO)
    blended = cv2.addWeighted(rgb, 1 - alpha, color, alpha, 0)
    out = rgb.copy()
    out[valid > 0] = blended[valid > 0]                 # 열화상 화각 밖은 원본 그대로
    return out


def fit_to(img: np.ndarray, width: int, height: int) -> np.ndarray:
    """카메라 영상 크기가 맵 설정과 다르면 맵 설정 크기로 맞춘다 (정렬 행렬이 그 크기 기준)."""
    if img.shape[1] == width and img.shape[0] == height:
        return img
    return cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)


def draw_blobs(img: np.ndarray, blobs, h: np.ndarray, alert: bool) -> np.ndarray:
    color = (0, 0, 255) if alert else (0, 255, 255)
    for b in blobs:
        corners = np.array([[b.x, b.y], [b.x + b.w, b.y + b.h]], np.float64)
        (x0, y0), (x1, y1) = map_points(corners, h).astype(int)
        cv2.rectangle(img, (x0, y0), (x1, y1), color, 2)
        cv2.putText(img, f"{b.t_max:.0f}", (x0, max(y0 - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(img, "ALERT" if alert else "normal", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 0, 255) if alert else (0, 200, 0), 2, cv2.LINE_AA)
    return img
