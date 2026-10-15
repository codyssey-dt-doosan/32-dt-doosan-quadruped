"""판독 중인 게이지 이름 추정."""

from gauge_ocr.naming import current_gauge_name
from inspection_common.viewpoint import Viewpoint

VPS = [
    Viewpoint("gauge_line1", 10.0, 5.7, 1.57, 2.96, 31.8, 28.1, True, ""),
    Viewpoint("gauge_bad", 0.0, 0.0, 0.0, 2.97, 0.0, 0.0, False, "no_space:x"),
]


def test_dwell_status_wins():
    assert current_gauge_name("dwell:gauge_line2", (10.0, 5.7), VPS) == "gauge_line2"


def test_nearest_viewpoint_within_radius():
    assert current_gauge_name("goto:home", (10.5, 5.2), VPS) == "gauge_line1"


def test_impossible_viewpoint_is_ignored():
    assert current_gauge_name("", (0.1, 0.0), VPS) == "unknown"


def test_unknown_when_far_or_no_pose():
    assert current_gauge_name("", (0.0, 3.0), VPS) == "unknown"
    assert current_gauge_name("", None, VPS) == "unknown"
