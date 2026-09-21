"""다리 애니메이션 (도훈) — /cmd_vel 속도에 비례한 트롯 사인파를 관절 목표로 발행.

시각 효과 전용. 몸체 이동은 VelocityControl이 하고, 다리는 추진에 기여하지 않는다.
대각 쌍 A(FL·RR)와 B(FR·RL)가 반대 위상 → 토픽 4개(thigh/calf × A/B).
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64

TOPICS = ("thigh_a", "thigh_b", "calf_a", "calf_b")


def gait_speed(v: float, w: float, turn_weight: float) -> float:
    """보행 속도 환산(m/s). 제자리 회전도 다리가 움직이게 |w|를 섞는다."""
    return abs(v) + turn_weight * abs(w)


def leg_targets(phase: float, scale: float, amp_thigh: float, amp_calf: float) -> tuple:
    """(thigh_a, thigh_b, calf_a, calf_b) rad. scale 0=정지(다리 곧게) ~ 1=최대 보폭.

    thigh 양수 = 다리 뒤로(y축 회전). 스윙(앞으로 되돌아오는 cos<0 구간)에서만 무릎을 굽혀 발을 든다.
    """
    s, c = math.sin(phase), math.cos(phase)
    return (
        scale * amp_thigh * s,
        -scale * amp_thigh * s,
        scale * amp_calf * max(0.0, -c),
        scale * amp_calf * max(0.0, c),
    )


class LegAnimationNode(Node):
    def __init__(self) -> None:
        super().__init__("leg_animation")
        self.declare_parameter("amp_thigh", 0.3)
        self.declare_parameter("amp_calf", 0.5)
        self.declare_parameter("stride_hz_per_mps", 4.0)
        self.declare_parameter("stride_hz_max", 2.5)
        self.declare_parameter("speed_full", 0.15)  # 이 속도부터 보폭 최대
        self.declare_parameter("turn_weight", 0.3)
        self.declare_parameter("cmd_timeout", 0.5)
        self.speed = 0.0
        self.last_cmd = None
        self.phase = 0.0
        self.scale = 0.0
        self.dt = 0.02
        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)
        self.pubs = [self.create_publisher(Float64, f"/leg_animation/{t}", 10) for t in TOPICS]
        self.create_timer(self.dt, self._tick)
        self.get_logger().info("leg_animation started (도훈)")

    def _cmd_cb(self, msg: Twist) -> None:
        self.speed = gait_speed(msg.linear.x, msg.angular.z, self.get_parameter("turn_weight").value)
        self.last_cmd = self.get_clock().now()

    def _tick(self) -> None:
        timeout = self.get_parameter("cmd_timeout").value
        stale = self.last_cmd is None or (self.get_clock().now() - self.last_cmd).nanoseconds * 1e-9 > timeout
        speed = 0.0 if stale else self.speed
        hz = min(self.get_parameter("stride_hz_per_mps").value * speed, self.get_parameter("stride_hz_max").value)
        self.phase = (self.phase + math.tau * hz * self.dt) % math.tau
        # 보폭은 0.2 s 시정수로 따라가게 해서 출발·정지 때 다리가 튀지 않게
        want = min(1.0, speed / self.get_parameter("speed_full").value)
        self.scale += (want - self.scale) * self.dt / 0.2
        targets = leg_targets(
            self.phase, self.scale, self.get_parameter("amp_thigh").value, self.get_parameter("amp_calf").value
        )
        for pub, value in zip(self.pubs, targets):
            pub.publish(Float64(data=value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LegAnimationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
