import math

from return_to_home.battery import BatteryModel, BatteryParams
from return_to_home.home_path import ReturnManager, ReturnParams, VisitedGraph


def walk(points, step=0.25):
    """점들을 잇는 경로를 step 간격으로 샘플링."""
    out = []
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) / step))
        out += [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(n)]
    return out + [points[-1]]


def test_graph_shortcuts_a_loop():
    # 10 m 나갔다가 옆으로 1.5 m 비켜 되돌아옴: 최단 경로는 옆 줄을 건너 직선으로 돌아간다
    g = VisitedGraph(spacing=1.0, link=1.8)
    for x, y in walk([(0, 0), (10, 0), (10, 1.5), (1, 1.5)]):
        g.visit(x, y)
    assert g.path_length(1.0, 1.5) < 3.0
    assert g.path_home(1.0, 1.5)[-1] == 0


def test_walked_edges_are_never_removed():
    g = VisitedGraph()
    for x, y in walk([(0, 0), (5, 0)]):
        g.visit(x, y)
    assert not g.remove_edge(0, 1)
    assert g.path_length(5.0, 0.0) < 5.5


def test_return_rule_threshold():
    p = ReturnParams(battery_per_meter=0.5, return_safety_factor=1.5, battery_reserve=10.0)
    m = ReturnManager(p)
    t = 0.0
    for x, y in walk([(0, 0), (20, 0)]):
        m.update(t, x, y, battery=90.0)
        t += 0.1
    # 20 m 떨어짐: 1.5 × 20 × 0.5 + 10 = 25 %
    assert abs(m.threshold - 25.0) < 1.0
    assert m.state == "IDLE"
    assert m.update(t, 20.0, 0.0, battery=24.0) is not None
    assert m.state == "RETURNING"


def test_returns_after_source_found_hold():
    m = ReturnManager(ReturnParams(found_hold_time=5.0))
    m.update(0.0, 0.0, 0.0, battery=100.0)
    assert m.update(1.0, 0.0, 0.0, battery=100.0, seeker_state="SOURCE_FOUND") is None
    assert m.update(7.0, 0.0, 0.0, battery=100.0, seeker_state="SOURCE_FOUND") is not None


def test_follows_path_home_and_stops():
    m = ReturnManager()
    t = 0.0
    for x, y in walk([(0, 0), (6, 0), (6, 4)]):
        m.update(t, x, y, battery=100.0)
        t += 0.1
    m.start("test")
    x, y = 6.0, 4.0
    for _ in range(500):
        m.update(t, x, y, battery=100.0)
        tgt = m.target()
        if tgt is None:
            break
        d = math.hypot(tgt[0] - x, tgt[1] - y)
        s = min(0.2, d)
        x, y = x + s * (tgt[0] - x) / d, y + s * (tgt[1] - y) / d
        t += 0.1
    assert m.state == "HOME"
    assert math.hypot(x, y) < 0.6


def test_battery_drains_with_distance_and_time():
    b = BatteryModel(BatteryParams(initial_percent=100.0, drain_per_meter=0.5, drain_per_second=0.02))
    b.update(0.0, 0.0, 0.0)
    assert abs(b.update(10.0, 10.0, 0.0) - (100.0 - 5.0 - 0.2)) < 1e-9
