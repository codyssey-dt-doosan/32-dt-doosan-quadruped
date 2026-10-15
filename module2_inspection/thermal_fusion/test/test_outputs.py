"""노드 출력 결정 (ROS 없이)."""

from inspection_common.state_msg import parse_state
from thermal_fusion.outputs import active_goal, state_line
from thermal_fusion.tracker import Spot


def test_active_goal_passes_when_fresh():
    assert active_goal((1.0, 2.0, 0.0), 0.2, 1.0) == (1.0, 2.0, 0.0)


def test_active_goal_none_when_odom_stale():
    assert active_goal((1.0, 2.0, 0.0), 1.5, 1.0) is None
    assert active_goal((1.0, 2.0, 0.0), None, 1.0) is None
    assert active_goal(None, 0.1, 1.0) is None


def test_state_line_pending():
    s = Spot(1, 7.65, -6.0, 85.2, 5.0, -6.0, 3, 0.0, state="PENDING", goal=(6.1, -6.0, 0.0))
    assert state_line(s, "C") == "PENDING | hotspot_1 (7.65, -6.00) | 85.2 C | goal (6.1, -6.0)"


def test_state_line_reason_and_idle():
    s = Spot(2, 0.0, 8.9, 250.0, 0.0, 0.0, 3, 0.0, state="UNREACHABLE", reason="no_space:wall_north/link")
    state, fields = parse_state(state_line(s, "lv"))
    assert state == "UNREACHABLE" and fields[-1] == "no_space:wall_north/link"
    assert state_line(None, "C") == "IDLE | no hotspot"
    assert state_line(s, "C", no_map=True) == "IDLE | no_map"


def test_state_line_no_image():
    s = Spot(1, 7.65, -6.0, 85.2, 5.0, -6.0, 3, 0.0, state="CONFIRMED")
    assert state_line(s, "C", no_image=True) == "IDLE | no image"
    assert state_line(s, "C", no_map=True, no_image=True) == "IDLE | no_map"


def test_state_line_no_odom():
    s = Spot(1, 7.65, -6.0, 85.2, 5.0, -6.0, 3, 0.0, state="PENDING", goal=(6.1, -6.0, 0.0))
    assert state_line(s, "C", no_odom=True) == "IDLE | no odom"
    assert state_line(s, "C", no_image=True, no_odom=True) == "IDLE | no image"
