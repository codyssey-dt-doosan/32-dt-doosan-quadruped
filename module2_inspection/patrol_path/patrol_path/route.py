"""ROS에 독립적인 순찰 도착·점검 대기·반복 상태."""

import math


class PatrolRoute:
    def __init__(self, waypoints, arrival_radius=0.4):
        if not waypoints:
            raise ValueError("순찰 경로에는 최소 한 개의 웨이포인트가 필요합니다")
        if not math.isfinite(arrival_radius) or arrival_radius <= 0:
            raise ValueError("arrival_radius는 양수여야 합니다")
        for wp in waypoints:
            if not all(math.isfinite(float(wp[k])) for k in ("x", "y", "yaw")):
                raise ValueError("웨이포인트 좌표와 yaw는 유한한 값이어야 합니다")
            if not math.isfinite(float(wp.get("dwell_s", 0))) or float(wp.get("dwell_s", 0)) < 0:
                raise ValueError("dwell_s는 0 이상의 유한한 값이어야 합니다")
        self.waypoints = waypoints
        self.arrival_radius = arrival_radius
        self.index = 0
        self.dwell_until = None
        self.hold_position = None

    def suspend(self):
        # 위치 수신이 끊긴 시간은 점검 시간으로 세지 않는다.
        self.dwell_until = None
        self.hold_position = None

    def step(self, position, now):
        wp = self.waypoints[self.index]
        if self.dwell_until is not None:
            if now < self.dwell_until:
                return self.hold_position, wp, "dwell"
            self.index = (self.index + 1) % len(self.waypoints)
            self.suspend()
            wp = self.waypoints[self.index]
        elif math.hypot(float(wp["x"]) - position[0], float(wp["y"]) - position[1]) < self.arrival_radius:
            dwell = float(wp.get("dwell_s", 0))
            if dwell > 0:
                self.dwell_until = now + dwell
                # MPC 정지 반경보다 도착 반경이 커도 즉시 정지하도록 현재 위치를 목표로 둔다.
                self.hold_position = position
                return position, wp, "dwell"
            self.index = (self.index + 1) % len(self.waypoints)
            wp = self.waypoints[self.index]
        return (float(wp["x"]), float(wp["y"])), wp, "goto"
