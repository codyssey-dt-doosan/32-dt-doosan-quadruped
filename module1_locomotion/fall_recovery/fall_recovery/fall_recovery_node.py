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


def set_pose_req(x: float, y: float, yaw: float, z: float, name: str = "go2") -> str:
    """gz.msgs.Pose 텍스트. roll/pitch=0, yaw만 유지."""
    qz = math.sin(yaw / 2)
    qw = math.cos(yaw / 2)
    return (
        f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, '
        f"orientation: {{x: 0.0, y: 0.0, z: {qz:.4f}, w: {qw:.4f}}}"
    )


def set_pose_cmd(world: str, req: str) -> list[str]:
    return [
        "gz", "service", "-s", f"/world/{world}/set_pose",
        "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
        "--timeout", "1000", "--req", req,
    ]


class FallRecoveryNode(Node):
    def __init__(self) -> None:
        super().__init__("fall_recovery")
        self.declare_parameter("world", "corridor")
        self.declare_parameter("fall_deg", 60.0)
        self.declare_parameter("upright_deg", 20.0)
        self.declare_parameter("debounce", 0.3)
        self.declare_parameter("recover_delay", 1.5)
        self.declare_parameter("max_retries", 3)
        self.declare_parameter("stand_z", 0.4)

        self.sm = FallStateMachine(
            fall_deg=self.get_parameter("fall_deg").value,
            upright_deg=self.get_parameter("upright_deg").value,
            debounce=self.get_parameter("debounce").value,
            recover_delay=self.get_parameter("recover_delay").value,
            max_retries=self.get_parameter("max_retries").value,
        )
        self.tilt: float | None = None
        self.odom_msg: Odometry | None = None

        self.create_subscription(Imu, "/imu", self.imu_cb, 10)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.pub = self.create_publisher(String, "/fall_recovery/status", 10)
        self.create_timer(0.1, self._tick)
        self.create_timer(0.5, self._publish_status)
        self.get_logger().info("fall_recovery started (도훈)")

    def imu_cb(self, msg: Imu) -> None:
        q = msg.orientation
        self.tilt = tilt_deg(q.x, q.y, q.z, q.w)

    def odom_cb(self, msg: Odometry) -> None:
        self.odom_msg = msg

    def _publish_status(self) -> None:
        out = String()
        out.data = self.sm.state
        self.pub.publish(out)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def reset_pose(self) -> bool:
        """제자리 기립: odom x, y, yaw 유지, z=stand_z. 성공 시 True."""
        # ponytail: gz CLI subprocess. 유니티가 물리 맡거나 서비스 브리지로 가면 이 함수만 교체
        if self.odom_msg is None:
            self.get_logger().warn("odom 없음, 리셋 보류")
            return False
        p = self.odom_msg.pose.pose.position
        q = self.odom_msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        world = self.get_parameter("world").value
        z = self.get_parameter("stand_z").value
        cmd = set_pose_cmd(world, set_pose_req(p.x, p.y, yaw, z))
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=2.0)
        except (subprocess.TimeoutExpired, OSError) as e:
            self.get_logger().error(f"set_pose 호출 실패: {e}")
            return False
        ok = r.returncode == 0 and "true" in r.stdout.lower()
        if not ok:
            self.get_logger().error(
                f"set_pose 거부: rc={r.returncode} out={r.stdout.strip()} err={r.stderr.strip()}"
            )
        return ok

    def _tick(self) -> None:
        if self.tilt is None:
            return
        prev = self.sm.state
        need_reset = self.sm.update(self._now(), self.tilt)
        if need_reset:
            self.get_logger().info(f"포즈 리셋 시도 {self.sm.retries}/{self.sm.max_retries}")
            if not self.reset_pose():
                self.sm.reset_failed()
        if self.sm.state != prev:
            self.get_logger().info(f"상태 {prev} → {self.sm.state} (tilt {self.tilt:.1f}°)")
            self._publish_status()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FallRecoveryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
