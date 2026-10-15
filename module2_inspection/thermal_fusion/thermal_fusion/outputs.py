"""노드가 내보낼 goal·상태 문자열을 정한다 (ROS 없이 테스트)."""

from inspection_common.state_msg import format_state


def active_goal(goal, odom_age, odom_timeout: float):
    """로봇 위치가 최신일 때만 goal을 낸다. 아니면 None → 빈 frame_id로 발행."""
    if goal is None or odom_age is None or odom_age > odom_timeout:
        return None
    return goal


def state_line(spot, unit: str, no_map: bool = False, no_image: bool = False,
               no_odom: bool = False) -> str:
    """알람(thermal_alert)은 마지막 확정 상태를 유지하고, 상태 문자열만 원인을 보여 준다."""
    if no_map:
        return format_state("IDLE", "no_map")
    if no_image:
        return format_state("IDLE", "no image")
    if no_odom:                                         # 위치를 몰라 goal·도착 판정을 멈춘 상태
        return format_state("IDLE", "no odom")
    if spot is None:
        return format_state("IDLE", "no hotspot")
    fields = [f"hotspot_{spot.id} ({spot.x:.2f}, {spot.y:.2f})", f"{spot.t_max:.1f} {unit}"]
    if spot.goal is not None and spot.state in ("PENDING", "INSPECTING"):
        fields.append(f"goal ({spot.goal[0]:.1f}, {spot.goal[1]:.1f})")
    if spot.reason:
        fields.append(spot.reason)
    return format_state(spot.state, *fields)
