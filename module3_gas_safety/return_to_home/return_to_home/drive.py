"""목표 지점 추종 보조 (ROS 비의존).

- 막힘 감지: 목표에 도착하지 않았는데 blocked_time 동안 위치도 방향도 거의 안 바뀜.
  바퀴/다리가 장애물에 걸려 전진도 회전도 못 하는 경우를 모두 잡는다. goal/cmd_vel 모드 공통.
- cmd_vel 모드 전용: 목표 방향 조향, 막히면 후진 후 회전하는 회피 동작.
  (goal 모드에서는 module1 mpc_controller가 장애물 회피를 맡는다.)
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class DriveParams:
    max_speed: float = 0.35          # m/s
    max_turn: float = 0.8            # rad/s
    heading_gain: float = 1.5
    blocked_time: float = 3.0        # s
    blocked_distance: float = 0.1    # m
    blocked_yaw: float = 0.15        # rad
    arrive_distance: float = 0.4     # 목표가 이보다 가까우면 도착으로 본다 (막힘 판정 제외)


class GoalDriver:
    def __init__(self, p: DriveParams | None = None) -> None:
        self.p = p or DriveParams()
        self._hist: deque = deque()   # (t, x, y, yaw, target_dist)
        self._recover: tuple[float, float, float] | None = None
        self._turn_sign = 1.0

    def blocked(self, t: float, x: float, y: float, yaw: float, target) -> bool:
        """이번 틱에 막힘으로 판정되면 True (한 번만)."""
        p = self.p
        if self._recover is not None or target is None:
            self._hist.clear()
            return False
        self._hist.append((t, x, y, yaw, math.hypot(target[0] - x, target[1] - y)))
        while t - self._hist[0][0] > p.blocked_time:
            self._hist.popleft()
        h0 = self._hist[0]
        if (t - h0[0] >= 0.9 * p.blocked_time and all(h[4] > p.arrive_distance for h in self._hist)
                and math.hypot(x - h0[1], y - h0[2]) < p.blocked_distance
                and abs(wrap(yaw - h0[3])) < p.blocked_yaw):
            self._hist.clear()
            return True
        return False

    def start_recovery(self, t: float) -> None:
        self._turn_sign = -self._turn_sign
        self._recover = (t + 1.5, t + 3.0, self._turn_sign)

    def command(self, t: float, x: float, y: float, yaw: float, target) -> tuple[float, float]:
        """cmd_vel 모드 (vx, wz). 회피 동작 중이면 그것을 우선한다."""
        if self._recover is not None:
            t_back, t_turn, sign = self._recover
            if t < t_back:
                return -0.2, 0.0
            if t < t_turn:
                return 0.0, 0.8 * sign
            self._recover = None
        if target is None:
            return 0.0, 0.0
        p = self.p
        err = wrap(math.atan2(target[1] - y, target[0] - x) - yaw)
        wz = max(-p.max_turn, min(p.max_turn, p.heading_gain * err))
        vx = p.max_speed * max(0.0, 1.0 - abs(err) / 1.2)
        return vx, wz
