import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import balance_metrics as bm  # noqa: E402
import walk_metrics as wm  # noqa: E402
from mpc_controller.rl_node import status_line  # noqa: E402

MPC_LINE = ("status mode=run qp_ms=0.00/0.00 fail=0 sat=0 sum_fz=0.0 sum_fn=0.0 sum_ft=0.0 calf_min=-0.598 tilt=0.00 "
            "ik_fail=0 ik_clamp=0 phase=0.00 active=0 servo_gap_ms=0.0/0.0/0.0 servo_stale=0 servo_reject=0 servo_lat_ms=0.0")


def test_mpc_line_unchanged(tmp_path):
    p = tmp_path / "m.log"; p.write_text(f"[walk]: {MPC_LINE}\n")
    assert bm.log_stats(str(p)) == {"mode": "run", "qp_ms": 0.0, "qp_p99": 0.0, "fail": 0, "sat": 0, "sum_fz": 0.0,
                                    "sum_fn": 0.0, "sum_ft": 0.0, "calf_min": -0.598, "hold": 0}


def test_rl_line_parses(tmp_path):
    p = tmp_path / "r.log"
    p.write_text("[rl]: " + status_line("run", (1.2, 2.5), 0, [3, 0, 0, 1.0, 3.1, 4.0, 0.5], 0.8) + "\n[rl]: hold: tilt=46.0\n")
    st = wm.log_stats(str(p))
    assert st["mode"] == "run" and st["inf_ms"] == 1.2 and st["inf_p99"] == 2.5 and st["sat"] == 3 and st["hold"] == 1
    assert "qp_ms" not in st and wm.compute_ok(st)


def test_track_floor():
    assert wm.track_floor(0.3, 0.0, 0.16, 0.0) and not wm.track_floor(0.3, 0.0, 0.14, 0.0)
    assert not wm.track_floor(0.0, 0.5, 0.0, 0.2) and wm.track_floor(0.5, 1.0, 0.3, 0.6)
    assert wm.track_floor(-0.2, 0.0, -0.12, 0.0) and not wm.track_floor(-0.2, 0.0, 0.12, 0.0)
