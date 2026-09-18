"""가상 배터리 (ROS 비의존): 걸은 거리(%/m)와 켜져 있는 시간(%/s)에 비례해 줄어든다."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class BatteryParams:
    initial_percent: float = 100.0
    drain_per_meter: float = 0.5     # %/m
    drain_per_second: float = 0.02   # %/s (서 있어도 전력 소모)


class BatteryModel:
    def __init__(self, p: BatteryParams) -> None:
        self.p = p
        self.percent = p.initial_percent
        self._last: tuple[float, float, float] | None = None

    def update(self, t: float, x: float, y: float) -> float:
        if self._last is not None:
            t0, x0, y0 = self._last
            used = self.p.drain_per_meter * math.hypot(x - x0, y - y0) + self.p.drain_per_second * max(0.0, t - t0)
            self.percent = max(0.0, self.percent - used)
        self._last = (t, x, y)
        return self.percent
