"""열화상 → RGB 정렬. 두 카메라는 방향이 같고 높이만 다르다."""

import cv2
import numpy as np


def k_from_spec(cam) -> np.ndarray:
    """CameraSpec → K. 정사각 픽셀, 주점 = 이미지 중앙."""
    fx = cam.fx
    return np.array([[fx, 0, cam.width / 2], [0, fx, cam.height / 2], [0, 0, 1]], np.float64)


def thermal_to_rgb_h(k_rgb: np.ndarray, k_th: np.ndarray, baseline_z: float = 0.03,
                     depth: float = 1.5) -> np.ndarray:
    """열화상 픽셀 → RGB 픽셀 호모그래피. 열화상이 baseline_z(m) 아래, 물체 거리 depth(m)."""
    h = k_rgb @ np.linalg.inv(k_th)
    dv = k_rgb[1, 1] * baseline_z / depth              # 열화상이 아래 → RGB로 옮길 때 아래(+v)로
    shift = np.array([[1, 0, 0], [0, 1, dv], [0, 0, 1]], np.float64)
    return shift @ h


def warp_to_rgb(thermal: np.ndarray, h: np.ndarray, rgb_size=(640, 480)) -> tuple:
    """(옮긴 배열, 유효 영역 마스크). rgb_size는 (너비, 높이)."""
    warped = cv2.warpPerspective(thermal, h, rgb_size, flags=cv2.INTER_LINEAR)
    valid = cv2.warpPerspective(np.full(thermal.shape[:2], 255, np.uint8), h, rgb_size,
                                flags=cv2.INTER_NEAREST)
    return warped, valid


def map_points(pts_th: np.ndarray, h: np.ndarray) -> np.ndarray:
    return cv2.perspectiveTransform(pts_th.reshape(-1, 1, 2).astype(np.float64), h).reshape(-1, 2)
