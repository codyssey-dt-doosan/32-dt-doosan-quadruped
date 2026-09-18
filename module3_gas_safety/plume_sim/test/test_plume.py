import math

from plume_sim.plume import GasField, GasFieldParams


def field(**kw):
    return GasField(GasFieldParams(**kw))


def test_peak_at_source():
    f = field(decoys=[])
    p = f.p
    c0 = f.mean(p.source_x, p.source_y)
    assert c0 > 0.9 * p.release_rate
    for a in range(0, 360, 30):
        x = p.source_x + 2.0 * math.cos(math.radians(a))
        y = p.source_y + 2.0 * math.sin(math.radians(a))
        assert f.mean(x, y) < c0


def test_plume_extends_downwind_not_upwind():
    f = field(decoys=[], wind_direction=0.0, source_x=0.0, source_y=0.0)
    assert f.mean(5.0, 0.0) > 5.0          # 풍하 5 m: 플룸 안
    assert f.mean(-5.0, 0.0) < 1.0         # 풍상 5 m: 배경 수준
    assert f.mean(5.0, 0.0) > f.mean(5.0, 3.0)   # 중심선이 가장 짙다


def test_decoy_is_local_maximum():
    f = field()
    dx, dy = f.decoys[0][:2]
    c = f.mean(dx, dy)
    assert all(f.mean(dx + 0.5 * math.cos(a), dy + 0.5 * math.sin(a)) < c for a in (0, 1.57, 3.14, 4.71))
    assert c < f.mean(f.p.source_x, f.p.source_y)   # 진짜 누출원보다 약하다


def test_sensor_noise_is_reproducible():
    a, b = field(seed=3), field(seed=3)
    assert [a.sample(1.0, 1.0) for _ in range(5)] == [b.sample(1.0, 1.0) for _ in range(5)]
