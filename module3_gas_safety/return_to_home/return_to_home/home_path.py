"""복귀 판단과 복귀 경로 (ROS 비의존).

VisitedGraph : 지나온 지점(breadcrumb)을 가까운 것끼리 이어 만든 그래프. 집까지 최단 경로(Dijkstra).
               실제로 걸어 본 곳만 이으므로 지도 없이도 안전한 복귀 경로가 된다.
ReturnManager: 복귀 규칙 검사 + 복귀 경로 추종.
    배터리% <= safety_factor × 집까지 경로 길이 × 소모율(%/m) + reserve  -> 복귀
    그 밖에 최대 거리, 임무 시간, 위험 농도, 누출원 발견 후 대기 끝, 탐색 영역 소진.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass


class VisitedGraph:
    def __init__(self, spacing: float = 1.0, link: float = 1.8) -> None:
        self.spacing = spacing      # 새 지점을 찍는 간격 (m)
        self.link = link            # 이 거리 안의 지점끼리는 서로 갈 수 있다고 본다 (m)
        self.nodes: list[tuple[float, float]] = []
        self._adj: list[list[tuple[int, float]]] = []
        self.cost: list[float] = []  # 집(nodes[0])까지 최단 거리
        self.prev: list[int | None] = []
        self._walked: set[tuple[int, int]] = set()  # 연달아 걸어 본 간선 (항상 통행 가능)

    def visit(self, x: float, y: float) -> None:
        """현재 위치를 반영한다. 최근 지점들에서 spacing 이상 떨어졌을 때만 새 노드."""
        if self.nodes and min(math.hypot(x - c[0], y - c[1]) for c in self.nodes[-20:]) < self.spacing:
            return
        i = len(self.nodes)
        nbrs = [(j, math.hypot(x - c[0], y - c[1])) for j, c in enumerate(self.nodes)]
        nbrs = [(j, d) for j, d in nbrs if d <= self.link or j == i - 1]  # 직전 지점과는 항상 연결
        if i > 0:
            self._walked.add((i - 1, i))
        self.nodes.append((x, y))
        self._adj.append(nbrs)
        for j, d in nbrs:
            self._adj[j].append((i, d))
        self._dijkstra()

    def _dijkstra(self) -> None:
        n = len(self.nodes)
        cost, prev = [math.inf] * n, [None] * n
        cost[0], heap = 0.0, [(0.0, 0)]
        while heap:
            c, u = heapq.heappop(heap)
            if c > cost[u]:
                continue
            for v, d in self._adj[u]:
                if c + d < cost[v]:
                    cost[v], prev[v] = c + d, u
                    heapq.heappush(heap, (c + d, v))
        self.cost, self.prev = cost, prev

    def _entry(self, x: float, y: float) -> tuple[int, float]:
        """먼저 걸어갈 노드와 총 경로 길이: 가까운(link 이내) 노드 중 최선, 없으면 마지막 노드."""
        best, best_len = None, math.inf
        for j, c in enumerate(self.nodes):
            d = math.hypot(x - c[0], y - c[1])
            if d <= self.link and d + self.cost[j] < best_len:
                best, best_len = j, d + self.cost[j]
        if best is None:   # 가까운 노드가 없으면 (직선거리 + 경로)가 최소인 노드
            for j, c in enumerate(self.nodes):
                total = math.hypot(x - c[0], y - c[1]) + self.cost[j]
                if total < best_len:
                    best, best_len = j, total
        return (best if best is not None else 0), best_len

    def path_length(self, x: float, y: float) -> float:
        return self._entry(x, y)[1] if self.nodes else 0.0

    def path_home(self, x: float, y: float) -> list[int]:
        """집까지 노드 인덱스 목록 (마지막이 집 = 0)."""
        return self.path_from(self._entry(x, y)[0])

    def path_from(self, k: int) -> list[int]:
        path = []
        while k is not None:
            path.append(k)
            k = self.prev[k]
        return path

    def nearest(self, x: float, y: float, exclude=()) -> int:
        cand = [j for j in range(len(self.nodes)) if j not in exclude and self.cost[j] < math.inf]
        return min(cand, key=lambda j: math.hypot(x - self.nodes[j][0], y - self.nodes[j][1])) if cand else 0

    def remove_edge(self, a: int, b: int) -> bool:
        """a-b 사이가 막혔다: 지름길 간선이면 지우고 경로를 다시 계산한다. 걸어 본 간선은 지우지 않는다."""
        if (min(a, b), max(a, b)) in self._walked:
            return False
        self._adj[a] = [(v, d) for v, d in self._adj[a] if v != b]
        self._adj[b] = [(v, d) for v, d in self._adj[b] if v != a]
        self._dijkstra()
        return True


@dataclass
class ReturnParams:
    battery_per_meter: float = 0.5      # %/m 사전값. 실측 소모율이 더 크면 그것을 쓴다
    return_safety_factor: float = 1.5
    battery_reserve: float = 10.0       # 항상 남길 %
    max_range: float = 50.0             # 홈에서 최대 거리 (m)
    max_mission_time: float = 1200.0     # s
    danger_conc: float = 0.0            # 로봇이 넘으면 안 되는 농도 (ppm, 0 = 끔)
    return_after_found: bool = True
    found_hold_time: float = 5.0        # 누출원에서 대기 후 복귀 (s)
    breadcrumb_spacing: float = 1.0
    breadcrumb_link: float = 1.8
    waypoint_tolerance: float = 0.4
    home_tolerance: float = 0.5


class ReturnManager:
    """상태: IDLE -> RETURNING -> HOME."""

    def __init__(self, p: ReturnParams | None = None) -> None:
        self.p = p or ReturnParams()
        self.graph = VisitedGraph(self.p.breadcrumb_spacing, self.p.breadcrumb_link)
        self.state, self.reason = "IDLE", ""
        self.return_reason = ""               # 복귀를 시작한 이유 (HOME 이후에도 유지)
        self.t = 0.0
        self.t_start: float | None = None
        self.t_home: float | None = None
        self.home: tuple[float, float] | None = None
        self.t0 = 0.0
        self.x = self.y = 0.0
        self.battery: float | None = None
        self.threshold: float | None = None   # 복귀 기준 %
        self._odo = 0.0
        self._last: tuple[float, float] | None = None
        self._batt_ref: tuple[float, float] | None = None
        self._c_f: float | None = None
        self._found_t: float | None = None
        self.path: list[int] = []            # 복귀 경로 (그래프 노드 인덱스)
        self.wp_idx = 0
        self._tried: set[int] = set()        # 현재 위치에서 곧장 가다 막힌 진입 노드
        self._detour: tuple[tuple[float, float], float] | None = None   # (옆으로 비켜설 지점, 만료 시각)
        self._blocks = 0                     # 같은 목표에서 막힌 횟수

    def update(self, t: float, x: float, y: float, battery: float | None = None, conc: float | None = None,
               seeker_state: str = "") -> str | None:
        """상태를 갱신하고, 이번에 복귀를 시작해야 하면 이유를 돌려준다."""
        p = self.p
        self.t, self.x, self.y, self.battery = t, x, y, battery
        if self.home is None:
            self.home, self.t0 = (x, y), t
        if self._last is not None:
            self._odo += math.hypot(x - self._last[0], y - self._last[1])
        self._last = (x, y)
        if battery is not None and self._batt_ref is None:
            self._batt_ref = (battery, self._odo)
        if conc is not None:
            self._c_f = conc if self._c_f is None else 0.3 * conc + 0.7 * self._c_f
        if self.state != "IDLE":
            return None
        self.graph.visit(x, y)

        d_home = self.graph.path_length(x, y)
        why = None
        if battery is not None:
            self.threshold = p.return_safety_factor * d_home * self.battery_rate() + p.battery_reserve
            if battery <= self.threshold:
                why = "battery %.1f%% <= %.1f%% needed for %.1f m home" % (battery, self.threshold, d_home)
        if p.danger_conc > 0 and self._c_f is not None and self._c_f >= p.danger_conc:
            why = "concentration %.1f ppm >= danger level %.1f" % (self._c_f, p.danger_conc)
        if math.hypot(x - self.home[0], y - self.home[1]) > p.max_range:
            why = "left the allowed range (%.0f m)" % p.max_range
        if t - self.t0 > p.max_mission_time:
            why = "mission time over %.0f s" % p.max_mission_time
        if seeker_state == "DONE":
            why = "search finished without a source"
        if seeker_state == "SOURCE_FOUND" and p.return_after_found:
            if self._found_t is None:
                self._found_t = t
            elif t - self._found_t >= p.found_hold_time:
                why = "mission complete: source found"
        if why:
            self.start(why)
        return why

    def battery_rate(self) -> float:
        """복귀 규칙에 쓰는 %/m: 사전값과 실측(보행+대기 포함) 중 큰 값."""
        rate = self.p.battery_per_meter
        if self._batt_ref is not None and self.battery is not None:
            b0, o0 = self._batt_ref
            if self._odo - o0 > 5.0:
                rate = max(rate, (b0 - self.battery) / (self._odo - o0))
        return rate

    def home_path_length(self) -> float:
        return self.graph.path_length(self.x, self.y)

    def start(self, reason: str) -> None:
        if self.state != "IDLE":
            return
        self.path, self.wp_idx = self.graph.path_home(self.x, self.y), 0
        self.state, self.reason, self.return_reason, self.t_start = "RETURNING", reason, reason, self.t

    def target(self) -> tuple[float, float] | None:
        """복귀 중 다음 목표 지점. 집에 도착하면 None + HOME."""
        if self.state != "RETURNING":
            return None
        if math.hypot(self.x - self.home[0], self.y - self.home[1]) < self.p.home_tolerance:
            self.t_home = self.t
            self.state, self.reason = "HOME", "arrived home (battery %s)" % (
                "%.1f%%" % self.battery if self.battery is not None else "n/a")
            return None
        while self.wp_idx < len(self.path) - 1 and self._dist(self.wps[self.wp_idx]) < self.p.waypoint_tolerance:
            self.wp_idx += 1
            self._tried.clear()
            self._blocks = 0
        if self._detour is not None:
            pt, t_end = self._detour
            if self.t < t_end and self._dist(pt) > self.p.waypoint_tolerance:
                return pt
            self._detour = None
        return self.wps[self.wp_idx] if self.path else self.home

    @property
    def wps(self) -> list[tuple[float, float]]:
        return [self.graph.nodes[k] for k in self.path]

    def on_blocked(self) -> None:
        """다음 노드로 못 감(장애물이 사이를 막음).
        노드 사이 지름길 간선이면 지우고 직전 노드로 돌아가 다시 경로를 짠다.
        걸어 본 간선(직선으로 가다 모서리에 걸린 경우)이면 좌우로 번갈아 비켜선 뒤 다시 시도한다.
        현재 위치에서 진입 노드로 곧장 가다 여러 번 막혔으면 가장 가까운 다른 노드부터 따라간다."""
        if self.state != "RETURNING" or not self.path:
            return
        i = self.wp_idx
        if i > 0 and self.graph.remove_edge(self.path[i - 1], self.path[i]):
            self.path, self.wp_idx = self.graph.path_from(self.path[i - 1]), 0
            return
        self._blocks += 1
        if i > 0 or self._blocks <= 2:
            tx, ty = self.wps[i]
            a = math.atan2(ty - self.y, tx - self.x) + (1 if self._blocks % 2 else -1) * math.pi / 2
            self._detour = ((self.x + 0.8 * math.cos(a), self.y + 0.8 * math.sin(a)), self.t + 5.0)
            return
        self._blocks = 0
        self._tried.add(self.path[i])
        self.path, self.wp_idx = self.graph.path_from(self.graph.nearest(self.x, self.y, self._tried)), 0

    def _dist(self, pt) -> float:
        return math.hypot(self.x - pt[0], self.y - pt[1])
