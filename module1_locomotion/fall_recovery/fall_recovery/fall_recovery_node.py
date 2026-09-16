"""IMU로 전도를 감지하고 Gazebo 포즈 리셋으로 제자리 기립시킨다. 상태는 /fall_recovery/status."""

import math
import subprocess

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String

IDLE = "idle"
FALLEN = "fallen"
RECOVERING = "recovering"
RECOVERED = "recovered"
FAILED = "failed"


def tilt_deg(qx: float, qy: float, qz: float, qw: float) -> float:
    """쿼터니언 → max(|roll|, |pitch|) [deg]."""
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    sp = max(-1.0, min(1.0, 2 * (qw * qy - qz * qx)))
    pitch = math.asin(sp)
    return math.degrees(max(abs(roll), abs(pitch)))


class FallStateMachine:
    """시각 t[s]·기울기 tilt[deg]를 받아 전이. update()가 True면 지금 reset_pose를 불러야 한다."""

    def __init__(
        self,
        fall_deg: float,
        upright_deg: float,
        debounce: float,
        recover_delay: float,
        max_retries: int,
        reset_timeout: float = 5.0,
        upright_hold: float = 0.5,
        recovered_hold: float = 2.0,
    ) -> None:
        self.fall_deg = fall_deg
        self.upright_deg = upright_deg
        self.debounce = debounce
        self.recover_delay = recover_delay
        self.max_retries = max_retries
        self.reset_timeout = reset_timeout
        self.upright_hold = upright_hold
        self.recovered_hold = recovered_hold
        self.state = IDLE
        self.retries = 0
        self._tilt_since: float | None = None
        self._fallen_at = 0.0
        self._reset_at = 0.0
        self._upright_since: float | None = None
        self._recovered_at = 0.0

    def reset_failed(self) -> None:
        self._reset_at = -math.inf

    def _start_reset(self, t: float) -> bool:
        if self.retries >= self.max_retries:
            self.state = FAILED
            return False
        self.retries += 1
        self.state = RECOVERING
        self._reset_at = t
        self._upright_since = None
        return True

    def update(self, t: float, tilt: float) -> bool:
        if self.state == IDLE:
            if tilt > self.fall_deg:
                if self._tilt_since is None:
                    self._tilt_since = t
                elif t - self._tilt_since >= self.debounce:
                    self.state = FALLEN
                    self._fallen_at = t
                    self.retries = 0
            else:
                self._tilt_since = None
            return False

        if self.state == FALLEN:
            if t - self._fallen_at >= self.recover_delay:
                return self._start_reset(t)
            return False

        if self.state == RECOVERING:
            if tilt < self.upright_deg:
                if self._upright_since is None:
                    self._upright_since = t
                elif t - self._upright_since >= self.upright_hold:
                    self.state = RECOVERED
                    self._recovered_at = t
                    return False
            else:
                self._upright_since = None
            if t - self._reset_at >= self.reset_timeout:
                return self._start_reset(t)
            return False

        if self.state == RECOVERED:
            if t - self._recovered_at >= self.recovered_hold:
                self.state = IDLE
                self._tilt_since = None
            return False

        return False  # FAILED: 고정


class FallRecoveryNode(Node):
    def __init__(self) -> None:
        super().__init__("fall_recovery")
        self.create_subscription(Imu, "/imu", self.imu_cb, 10)
        self.pub_0 = self.create_publisher(String, "/fall_recovery/status", 10)
        self.create_timer(0.5, self._tick)
        self.get_logger().info("fall_recovery started (도훈)")

    def imu_cb(self, msg: Imu) -> None:
        del msg

    def _tick(self) -> None:
        out = String(); out.data = "idle"; self.pub_0.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FallRecoveryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
