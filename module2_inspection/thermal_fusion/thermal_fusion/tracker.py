"""과열 지점 묶기와 상태 전이. 시간·위치를 인자로 받는 순수 클래스.

CANDIDATE --(confirm_frames번 관측)--> PENDING --(goal 도착·정차)--> INSPECTING
INSPECTING --(dwell_s 뒤 최고값 ≥ t_confirm)--> CONFIRMED, 아니면 FALSE_ALARM
PENDING --(inspect_timeout × max_attempts)--> UNREACHABLE
확정된 지점은 recheck_s(0이면 안 함)가 지나야 다시 후보가 된다.
"""

from dataclasses import dataclass
import math

CANDIDATE, PENDING, INSPECTING = "CANDIDATE", "PENDING", "INSPECTING"
CONFIRMED, FALSE_ALARM, UNREACHABLE = "CONFIRMED", "FALSE_ALARM", "UNREACHABLE"
DECIDED = (CONFIRMED, FALSE_ALARM, UNREACHABLE)
_HEADLINE_ORDER = (CONFIRMED, CANDIDATE, UNREACHABLE, FALSE_ALARM)


@dataclass
class Spot:
    id: int
    x: float
    y: float
    t_max: float
    from_x: float               # 처음 본 로봇 위치 → 접근 방향
    from_y: float
    n_obs: int
    last_seen: float
    state: str = CANDIDATE
    reason: str = ""
    goal: tuple | None = None
    goal_since: float | None = None
    attempts: int = 0
    inspect_start: float | None = None
    inspect_tmax: float = -math.inf
    decided_at: float | None = None


class Tracker:
    def __init__(self, plan_goal, t_confirm: float, merge_radius: float = 1.0,
                 confirm_frames: int = 3, forget_s: float = 3.0, dwell_s: float = 5.0,
                 arrive_tol: float = 0.4, stop_speed: float = 0.05,
                 inspect_timeout: float = 60.0, max_attempts: int = 2, recheck_s: float = 0.0):
        self.plan_goal = plan_goal
        self.t_confirm = t_confirm
        self.merge_radius = merge_radius
        self.confirm_frames = confirm_frames
        self.forget_s = forget_s
        self.dwell_s = dwell_s
        self.arrive_tol = arrive_tol
        self.stop_speed = stop_speed
        self.inspect_timeout = inspect_timeout
        self.max_attempts = max_attempts
        self.recheck_s = recheck_s
        self.spots: list = []
        self._next_id = 1
        self._active_id: int | None = None

    def _nearest(self, x: float, y: float):
        near = [(math.hypot(s.x - x, s.y - y), s) for s in self.spots]
        near = [(d, s) for d, s in near if d <= self.merge_radius]
        return min(near, key=lambda ds: ds[0])[1] if near else None

    def observe(self, x: float, y: float, t: float, robot_x: float, robot_y: float, now: float):
        s = self._nearest(x, y)
        if s is None:
            s = Spot(self._next_id, x, y, t, robot_x, robot_y, 0, now)
            self._next_id += 1
            self.spots.append(s)
        elif s.state in DECIDED and self.recheck_s > 0 and now - s.decided_at >= self.recheck_s:
            s.state, s.reason, s.goal, s.attempts, s.n_obs = CANDIDATE, "", None, 0, 0
            s.goal_since, s.inspect_start = None, None      # 이전 주기의 시각이 남아 즉시 시간 초과되지 않도록
            s.t_max, s.from_x, s.from_y, s.decided_at = t, robot_x, robot_y, None
        s.n_obs += 1
        s.last_seen = now
        s.t_max = max(s.t_max, t)
        if s.state == CANDIDATE and s.n_obs > 1:              # 후보일 때만 위치를 평균으로 다듬는다
            s.x += (x - s.x) / s.n_obs
            s.y += (y - s.y) / s.n_obs
        if s.state == INSPECTING:
            s.inspect_tmax = max(s.inspect_tmax, t)
        if s.state == CANDIDATE and s.n_obs >= self.confirm_frames:
            s.goal, s.reason = self.plan_goal(s.x, s.y, s.from_x, s.from_y)
            if s.goal is None:
                s.state, s.decided_at = UNREACHABLE, now
            else:
                s.state = PENDING
        return s

    def _active(self, robot_x: float, robot_y: float):
        for s in self.spots:
            if s.id == self._active_id and s.state in (PENDING, INSPECTING):
                return s
        pending = [s for s in self.spots if s.state == PENDING]
        if not pending:
            self._active_id = None
            return None
        s = min(pending, key=lambda p: math.hypot(p.goal[0] - robot_x, p.goal[1] - robot_y))
        self._active_id = s.id
        return s

    def forget(self, now: float) -> None:
        """오래 안 보인 후보를 지운다. 로봇 위치가 없어도 부를 수 있다."""
        self.spots = [s for s in self.spots
                      if not (s.state == CANDIDATE and now - s.last_seen > self.forget_s)]

    def update(self, robot_x: float, robot_y: float, speed: float, now: float) -> None:
        self.forget(now)
        s = self._active(robot_x, robot_y)
        if s is None:
            return
        if s.state == PENDING:
            if s.goal_since is None:
                s.goal_since = now
            gx, gy, _ = s.goal
            if math.hypot(robot_x - gx, robot_y - gy) <= self.arrive_tol and speed < self.stop_speed:
                s.state, s.inspect_start, s.inspect_tmax = INSPECTING, now, -math.inf
                s.reason = ""                                 # 이전 시간 초과 사유 지움
            elif now - s.goal_since > self.inspect_timeout:
                s.attempts += 1
                s.goal_since, s.reason = None, "unreachable"
                if s.attempts >= self.max_attempts:
                    s.state, s.decided_at = UNREACHABLE, now
        elif s.state == INSPECTING and now - s.inspect_start >= self.dwell_s:
            s.state = CONFIRMED if s.inspect_tmax >= self.t_confirm else FALSE_ALARM
            s.decided_at = now
        if s.state in DECIDED:
            self._active_id = None

    def goal(self):
        for s in self.spots:
            if s.id == self._active_id and s.state in (PENDING, INSPECTING):
                return s.goal
        return None

    def alert(self) -> bool:
        return any(s.state == CONFIRMED for s in self.spots)

    def headline(self):
        """상태 토픽에 보일 지점: 활성 > 확정 > 후보 > 도달 불가 > 오탐 (같으면 최근)."""
        for s in self.spots:
            if s.id == self._active_id and s.state in (PENDING, INSPECTING):
                return s
        for state in (PENDING,) + _HEADLINE_ORDER:
            same = [s for s in self.spots if s.state == state]
            if same:
                return max(same, key=lambda s: s.last_seen)
        return None
