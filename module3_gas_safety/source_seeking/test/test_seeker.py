import math

import pytest

from source_seeking.offline_sim import load_all, run
from source_seeking.seeker import SeekerCore, SeekerParams, spiral


def test_spiral_grows_outward():
    pts = spiral(0.0, 0.0, 0.5, 2.0, 5.0, 0.6)
    radii = [math.hypot(x, y) for x, y in pts]
    assert radii[0] == pytest.approx(0.5)
    assert max(radii) <= 5.0
    assert all(b >= a - 1e-9 for a, b in zip(radii, radii[1:]))


def test_gradient_fit_recovers_plane():
    core = SeekerCore(SeekerParams())
    for i in range(60):   # 지그재그로 움직이며 c = 10 + 2x - y 측정
        x, y = 0.05 * i, 0.3 * math.sin(i / 3.0)
        core.samples.append((0.1 * i, x, y, 10 + 2 * x - y))
    gx, gy = core._gradient()
    assert gx == pytest.approx(2.0, abs=0.05)
    assert gy == pytest.approx(-1.0, abs=0.05)


@pytest.mark.parametrize("world", ["factory", "corridor"])
def test_hybrid_finds_source_and_returns(tmp_path, world):
    seeker, ret, _ = run(world, "hybrid", None, 0, str(tmp_path / world))
    src = load_all(world)["gas"]
    assert seeker.source is not None
    assert math.hypot(seeker.source[0] - src.source_x, seeker.source[1] - src.source_y) < 1.0
    assert ret.state == "HOME"


def test_gradient_only_gets_trapped_by_decoy(tmp_path):
    seeker, ret, _ = run("factory", "gradient", None, 0, str(tmp_path / "g"))
    reasons = " ".join(e[2] for e in seeker.events)
    assert "local optimum" in reasons          # 국소 최대 감지
    assert seeker.tabu                         # 가짜 봉우리 기록
    assert ret.state == "HOME"                 # 그래도 안전하게 복귀


def test_low_battery_returns_before_empty(tmp_path):
    seeker, ret, _ = run("factory", "hybrid", 25.0, 0, str(tmp_path / "b"))
    assert ret.return_reason.startswith("battery")
    assert ret.state == "HOME"
    assert ret.battery > 0.0
