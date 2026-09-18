"""누출원 탐색 상태 머신 (ROS 비의존). 출력은 목표 지점 (x, y) — goal 모드면 mpc_controller에, cmd_vel
모드면 GoalDriver에 넘긴다.

    SEARCH ──가스 감지──> TRACK ──빙빙 돎 / 개선 없음 / 지나침──> ESCAPE (Archimedean spiral)
      ^                     ^   └──가스 놓침──> ESCAPE(lost)            │
      │                     └──── 멀리서 더 높은 농도 (기존 봉우리 tabu) ─┤
      └──── 나선 끝, 봉우리 < found_min_conc (가짜 봉우리, tabu) ──────────┤
                              나선 끝, 봉우리 >= found_min_conc ─────────> SOURCE_FOUND

TRACK에서 강한 봉우리를 지나쳐 농도가 뚝 떨어지면(바람 반대로만 가면 누출원을 지나친다) 봉우리로 돌아가 확인한다.
TRACK 방향 = 바람 반대(hybrid 모드) + 농도 구배(최근 수 초 평면 최소제곱) + tabu·막혔던 지점 반발.
사인파로 좌우를 흔들어(weave) 측면 구배도 측정한다. 복귀는 return_to_home이 맡는다.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field


@dataclass
class SeekerParams:
    mode: str = "hybrid"                # hybrid (바람+구배) | gradient (구배만)
    lookahead: float = 1.5              # TRACK 목표를 이만큼 앞에 둔다 (m)
    waypoint_tolerance: float = 0.4
    # 농도
    conc_filter_alpha: float = 0.3      # 센서 EMA
    detect_threshold: float = 3.0       # ppm: 가스 있음
    lost_time: float = 8.0              # s 동안 detect 미만이면 플룸 놓침
    found_min_conc: float = 40.0        # 어떤 나선으로도 못 넘는 이 이상의 봉우리 = 누출원
    # 추적
    grad_window: float = 6.0            # 구배 평면 맞춤에 쓰는 최근 샘플 (s)
    min_grad: float = 0.3               # ppm/m
    weave_amplitude: float = 0.6        # rad
    weave_period: float = 6.0           # s
    upwind_weight: float = 1.0
    gradient_weight: float = 1.0
    min_wind_speed: float = 0.1         # m/s
    # local optimum 감지
    stuck_window: float = 30.0          # s
    stuck_radius: float = 1.2           # m
    noimprove_time: float = 40.0        # s
    noimprove_travel: float = 4.0       # 최고점에서 이만큼 멀어지며 이동 중이면 평평한 원거리 플룸일 뿐, 갇힌 게 아니다 (m)
    improve_ratio: float = 0.05
    overshoot_ratio: float = 0.5        # 강한 봉우리(>= found_min_conc)를 지난 뒤 농도가 이 비율 아래로 ...
    overshoot_time: float = 3.0         # ... 이 시간 떨어져 있으면 지나친 것 -> 봉우리 주변 확인 나선
    # 탈출 나선
    escape_start_radius: float = 0.5
    escape_pitch: float = 2.0           # 한 바퀴당 반경 증가 (m)
    escape_max_radius: float = 5.0
    escape_step: float = 0.6            # 나선 경유점 간격 (m)
    escape_improve_ratio: float = 0.08  # 기준보다 이만큼 높은 상태가 ...
    escape_confirm_time: float = 1.5    # ... 이 시간 이어지면 탈출 성공 (노이즈 방지)
    confirm_max_radius: float = 2.5     # 봉우리가 이미 강하면 작은 나선으로 확인만
    tabu_radius: float = 3.0            # 가짜 봉우리 반발 반경 (m)
    obstacle_clearance: float = 1.0     # 막혔던 곳 근처 경유점은 건너뜀 (m)
    detour_distance: float = 1.2        # TRACK 중 막히면 옆으로 비켜서는 거리 (m)
    # 가스 없을 때 탐색 나선
    search_pitch: float = 2.5
    search_max_radius: float = 12.0
    # 주행 가능 영역 [xmin, ymin, xmax, ymax] (빈 리스트 = 제한 없음). 밖의 경유점은 건너뜀
    bounds: list = field(default_factory=list)


def spiral(cx, cy, r0, pitch, rmax, step, theta0=0.0):
    """Archimedean spiral r = r0 + pitch·θ/2π, 경유점 간격 ~step, rmax까지."""
    pts, th = [], 0.0
    while True:
        r = r0 + pitch * th / (2 * math.pi)
        if r > rmax:
            return pts
        pts.append((cx + r * math.cos(theta0 + th), cy + r * math.sin(theta0 + th)))
        th += step / max(r, 0.3)


class SeekerCore:
    def __init__(self, params: SeekerParams | None = None) -> None:
        self.p = params or SeekerParams()
        self.state, self.reason = "WAIT", ""
        self.events: list[tuple[float, str, str]] = []
        self.start: tuple[float, float] | None = None
        self.c_f: float | None = None
        self.samples: deque = deque()   # (t, x, y, c_f)
        self.tabu: list[tuple[float, float]] = []
        self.obstacle_pts: list[tuple[float, float]] = []
        self.source: tuple[float, float] | None = None
        self.wind = None
        self.wps: list[tuple[float, float]] = []
        self.wp_idx = 0
        self.search_wps = None
        self.search_idx = 0
        self.heading = 0.0
        self.detour = None                # (지점, 만료 시각): TRACK 중 장애물 옆으로 비켜서기
        self.t = self.x = self.y = self.yaw = 0.0

    # ---------------------------------------------------------------- 공개
    def update(self, t, x, y, yaw, conc, wind=None):
        """한 스텝. conc ppm, wind (wx, wy) 월드 좌표 또는 None -> 목표 지점 (x, y) 또는 None(정지)."""
        p = self.p
        self.t, self.x, self.y, self.yaw, self.wind = t, x, y, yaw, wind
        self.c_f = conc if self.c_f is None else p.conc_filter_alpha * conc + (1 - p.conc_filter_alpha) * self.c_f
        self.samples.append((t, x, y, self.c_f))
        while t - self.samples[0][0] > p.grad_window:
            self.samples.popleft()
        if self.state == "WAIT":
            self.start, self.heading = (x, y), yaw
            self._set("SEARCH", "start")
        return getattr(self, "_" + self.state.lower())()

    def on_blocked(self) -> None:
        """주행 계층이 막힘을 알려 온다: 막힌 지점을 기억하고 상태별로 대응."""
        self.obstacle_pts.append((self.x + 0.6 * math.cos(self.yaw), self.y + 0.6 * math.sin(self.yaw)))
        self.events.append((self.t, self.state, "blocked by obstacle"))
        if self.state == "TRACK":
            level = self._recent_level()
            if level >= self.p.found_min_conc:   # 누출원 수준에서 막힘 (탱크 옆 등): 주변 확인
                self._enter_escape((self.x, self.y), level, "blocked near a strong peak, confirming")
            else:   # 장애물일 뿐 local optimum이 아니다: 옆으로 비켜선 뒤 계속 추적
                self._start_detour()
        elif self.state == "ESCAPE":
            self.wp_idx = min(self.wp_idx + 1, len(self.wps))   # 장애물 너머 경유점은 포기
        elif self.state == "SEARCH":
            self.search_idx += 1

    # ---------------------------------------------------------------- 상태
    def _search(self):
        p = self.p
        if self.search_wps is None:
            self.search_wps = spiral(self.start[0], self.start[1], p.search_pitch / 2, p.search_pitch,
                                     p.search_max_radius, 1.0, self.yaw)
        if self.c_f >= p.detect_threshold and not self._in_tabu(self.x, self.y):
            self._enter_track("gas detected (%.1f ppm)" % self.c_f)
            return self._track()
        while self.search_idx < len(self.search_wps) and self._skip(self.search_wps[self.search_idx]):
            self.search_idx += 1
        if self.search_idx >= len(self.search_wps):
            self._set("DONE", "search area exhausted, no source")
            return None
        return self.search_wps[self.search_idx]

    def _enter_track(self, reason):
        self.track_start = self.t
        self.track_pos = deque()
        self.track_best, self.track_best_pos, self.t_best = self.c_f, (self.x, self.y), self.t
        self.last_good, self.t_last_good = (self.x, self.y), self.t
        self.t_below = None
        self.heading = self.yaw
        self._set("TRACK", reason)

    def _track(self):
        p, t = self.p, self.t
        if self.c_f >= p.detect_threshold:
            self.last_good, self.t_last_good = (self.x, self.y), t
        elif t - self.t_last_good > p.lost_time:
            self._enter_escape(self.last_good, self._recent_level(), "plume lost", lost=True)
            return self._escape()
        if self.c_f > self.track_best * (1 + p.improve_ratio):
            self.track_best, self.track_best_pos, self.t_best = self.c_f, (self.x, self.y), t
        self.track_pos.append((t, self.x, self.y))
        while t - self.track_pos[0][0] > p.stuck_window:
            self.track_pos.popleft()

        if self.track_best >= p.found_min_conc and self.c_f < p.overshoot_ratio * self.track_best:
            if self.t_below is None:
                self.t_below = t
            elif t - self.t_below >= p.overshoot_time:
                # 기준은 노이즈로 부풀려진 최대값을 조금 깎은 값
                self._enter_escape(self.track_best_pos, 0.9 * self.track_best,
                                   "overshoot: %.1f ppm fell below %.0f%% of %.1f ppm peak"
                                   % (self.c_f, 100 * p.overshoot_ratio, self.track_best))
                return self._escape()
        else:
            self.t_below = None

        stuck = None
        if t - self.track_start >= p.stuck_window:
            cx = sum(q[1] for q in self.track_pos) / len(self.track_pos)
            cy = sum(q[2] for q in self.track_pos) / len(self.track_pos)
            r = max(math.hypot(q[1] - cx, q[2] - cy) for q in self.track_pos)
            if r < p.stuck_radius:
                stuck = "local optimum: circling within %.1f m for %.0f s" % (r, p.stuck_window)
        if t - self.t_best > p.noimprove_time:
            if math.hypot(self.x - self.track_best_pos[0], self.y - self.track_best_pos[1]) > p.noimprove_travel:
                self.t_best = t
            else:
                stuck = "local optimum: no improvement for %.0f s (best %.1f ppm)" % (p.noimprove_time, self.track_best)
        if stuck:
            self._enter_escape(self.track_best_pos, self._recent_level(), stuck)
            return self._escape()

        if self.detour is not None:
            pt, t_end = self.detour
            if t < t_end and math.hypot(pt[0] - self.x, pt[1] - self.y) > p.waypoint_tolerance:
                return pt
            self.detour = None

        vx, vy = 0.0, 0.0
        if p.mode == "hybrid" and self.wind is not None:
            ws = math.hypot(*self.wind)
            if ws > p.min_wind_speed:
                vx -= p.upwind_weight * self.wind[0] / ws
                vy -= p.upwind_weight * self.wind[1] / ws
        g = None if self._in_tabu(self.x, self.y) else self._gradient()  # 가짜 봉우리 근처 구배는 믿지 않는다
        if g is not None:
            gn = math.hypot(*g)
            if gn > p.min_grad:
                vx += p.gradient_weight * g[0] / gn
                vy += p.gradient_weight * g[1] / gn
        for pts, radius in ((self.tabu, p.tabu_radius), (self.obstacle_pts, p.obstacle_clearance + 0.5)):
            for (qx, qy) in pts:
                d = math.hypot(self.x - qx, self.y - qy)
                if 1e-3 < d < radius:
                    k = 2.0 * (1 - d / radius)
                    vx += k * (self.x - qx) / d
                    vy += k * (self.y - qy) / d
        if math.hypot(vx, vy) > 1e-6:
            self.heading = math.atan2(vy, vx)
        a = self.heading + p.weave_amplitude * math.sin(2 * math.pi * (t - self.track_start) / p.weave_period)
        return self._clip((self.x + p.lookahead * math.cos(a), self.y + p.lookahead * math.sin(a)))

    def _enter_escape(self, center, ref, reason, lost=False):
        p = self.p
        self.esc_center, self.esc_ref, self.esc_lost = center, ref, lost
        self.esc_radius = p.confirm_max_radius if (not lost and ref >= p.found_min_conc) else p.escape_max_radius
        self.wps = spiral(center[0], center[1], p.escape_start_radius, p.escape_pitch, self.esc_radius,
                          p.escape_step, self.yaw)
        self.wp_idx = 0
        self.t_above = None
        self._set("ESCAPE", reason)

    def _escape(self):
        p = self.p
        if self.esc_lost:
            if self.c_f >= p.detect_threshold:
                self._enter_track("plume re-acquired (%.1f ppm)" % self.c_f)
                return self._track()
        elif self.c_f > self.esc_ref * (1 + p.escape_improve_ratio) and \
                math.hypot(self.x - self.esc_center[0], self.y - self.esc_center[1]) > p.escape_start_radius:
            if self.t_above is None:
                self.t_above = self.t
            elif self.t - self.t_above >= p.escape_confirm_time:
                # 멀리서 더 높으면 기존 봉우리는 가짜. 가까우면 같은 언덕이라 tabu로 두지 않는다
                # (진짜 누출원 옆을 tabu로 찍으면 반발력이 로봇을 누출원에서 밀어낸다)
                far = math.hypot(self.x - self.esc_center[0], self.y - self.esc_center[1]) > p.tabu_radius
                if far:
                    self.tabu.append(self.esc_center)
                self._enter_track("higher concentration %.1f > %.1f ppm%s" % (
                    self.c_f, self.esc_ref, ", old peak marked tabu" if far else " on the same hill"))
                return self._track()
        else:
            self.t_above = None
        while self.wp_idx < len(self.wps) and self._skip(self.wps[self.wp_idx]):
            self.wp_idx += 1
        if self.wp_idx < len(self.wps):
            return self.wps[self.wp_idx]
        # 나선 끝까지 더 높은 곳이 없음
        if not self.esc_lost and self.esc_ref >= p.found_min_conc:
            self.source = self.esc_center
            self._set("SOURCE_FOUND", "no higher concentration within %.1f m of %.1f ppm peak at (%.2f, %.2f)"
                      % (self.esc_radius, self.esc_ref, self.source[0], self.source[1]))
            return self._source_found()
        if not self.esc_lost:
            self.tabu.append(self.esc_center)
        self._set("SEARCH", "spiral exhausted (peak %.1f ppm < %.1f), resume search" % (self.esc_ref, p.found_min_conc))
        return self._search()

    def _source_found(self):
        """누출원 위로 걸어가 선다."""
        if math.hypot(self.source[0] - self.x, self.source[1] - self.y) > self.p.waypoint_tolerance:
            return self.source
        return None

    def _done(self):
        return None

    # ---------------------------------------------------------------- 보조
    def _gradient(self):
        """최근 샘플에 평면 c = a + g·(p - 평균)을 최소제곱으로 맞춘다. 샘플이 일직선이면 그 방향 1차원 기울기."""
        s = self.samples
        n = len(s)
        if n < 10:
            return None
        mx = sum(q[1] for q in s) / n
        my = sum(q[2] for q in s) / n
        mc = sum(q[3] for q in s) / n
        sxx = sxy = syy = sxc = syc = 0.0
        for (_, x, y, c) in s:
            dx, dy, dc = x - mx, y - my, c - mc
            sxx += dx * dx
            sxy += dx * dy
            syy += dy * dy
            sxc += dx * dc
            syc += dy * dc
        tr, det = sxx + syy, sxx * syy - sxy * sxy
        if tr < n * 0.2 ** 2:                     # 거의 안 움직임
            return None
        lam_min = tr / 2 - math.sqrt(max(tr * tr / 4 - det, 0.0))
        if lam_min > n * 0.08 ** 2:               # 양방향으로 충분히 퍼짐: 2차원
            return ((syy * sxc - sxy * syc) / det, (sxx * syc - sxy * sxc) / det)
        lam_max = tr - lam_min                    # 일직선: 주축 방향 기울기
        ex, ey = (sxy, lam_max - sxx) if abs(sxy) > 1e-9 else ((1.0, 0.0) if sxx >= syy else (0.0, 1.0))
        en = math.hypot(ex, ey)
        ex, ey = ex / en, ey / en
        slope = (ex * sxc + ey * syc) / lam_max
        return (slope * ex, slope * ey)

    def _start_detour(self):
        """진행 방향 좌우 detour_distance 지점 중 주행 영역 안이고 막혔던 곳에서 먼 쪽으로 비켜선다."""
        d = self.p.detour_distance
        cands = []
        for side in (1.0, -1.0):
            a = self.yaw + side * math.pi / 2
            pt = (self.x + d * math.cos(a) - 0.3 * math.cos(self.yaw), self.y + d * math.sin(a) - 0.3 * math.sin(self.yaw))
            clear = min((math.hypot(pt[0] - q[0], pt[1] - q[1]) for q in self.obstacle_pts), default=9.0)
            cands.append((self._in_bounds(pt), clear, pt))
        best = max(cands)
        self.detour = (self._clip(best[2]), self.t + 6.0)

    def _recent_level(self):
        """갇힌 지점의 농도 수준: 최근 grad_window 평균 (노이즈 섞인 순간 최대는 기준을 과하게 올린다)."""
        return sum(q[3] for q in self.samples) / max(len(self.samples), 1)

    def _skip(self, pt):
        return (math.hypot(pt[0] - self.x, pt[1] - self.y) < self.p.waypoint_tolerance or self._in_tabu(*pt)
                or not self._in_bounds(pt) or self._near_obstacle(pt))

    def _in_bounds(self, pt, margin=0.0):
        b = self.p.bounds
        return len(b) != 4 or (b[0] + margin <= pt[0] <= b[2] - margin and b[1] + margin <= pt[1] <= b[3] - margin)

    def _clip(self, pt):
        b = self.p.bounds
        if len(b) != 4:
            return pt
        return (min(max(pt[0], b[0]), b[2]), min(max(pt[1], b[1]), b[3]))

    def _near_obstacle(self, pt):
        return any(math.hypot(pt[0] - q[0], pt[1] - q[1]) < self.p.obstacle_clearance for q in self.obstacle_pts)

    def _in_tabu(self, x, y):
        return any(math.hypot(x - q[0], y - q[1]) < self.p.tabu_radius for q in self.tabu)

    def _set(self, state, reason):
        self.state, self.reason = state, reason
        self.events.append((self.t, state, reason))
