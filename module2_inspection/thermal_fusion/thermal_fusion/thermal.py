"""열화상 픽셀 → 온도(℃). brightness는 지금 시뮬 흑백 센서용으로 밝기를 그대로 쓴다(단위 없음)."""

import cv2
import numpy as np

MODES = ("l16", "l8_thermal", "brightness")


def to_temperature(img: np.ndarray, mode: str, l8_min_k: float = 253.15, l8_res: float = 3.0) -> np.ndarray:
    if mode == "l16":                                   # 0.01 K 단위
        return img.astype(np.float32) * np.float32(0.01) - np.float32(273.15)
    if mode == "l8_thermal":                            # ThermalSensor 플러그인 L8
        return (np.float32(l8_min_k) + img.astype(np.float32) * np.float32(l8_res)
                - np.float32(273.15))
    if mode == "brightness":
        if img.ndim == 3:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return img.astype(np.float32)
    raise ValueError(f"thermal_mode는 {MODES} 중 하나 (받은 값: {mode})")
