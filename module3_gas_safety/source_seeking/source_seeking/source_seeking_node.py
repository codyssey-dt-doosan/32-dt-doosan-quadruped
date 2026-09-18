"""농도 구배로 누출원을 추적한다.

SeekerCore(탐색 → 추적 → local optimum 탈출 나선 → 누출원 판정)를 10 Hz로 돌린다.
  구독  /odom, /gas/concentration, /gas/wind, /return_to_home/status
  발행  /source_seeking/goal (PoseStamped, drive_mode=goal: mpc_controller가 따라감)
        /cmd_vel (drive_mode=cmd_vel일 때만: 단독 시험용)
        /source_seeking/state (String "STATE | 이유 | 농도"), /source_seeking/source (PointStamped, 발견 시)
        /source_seeking/markers (tabu·막힌 지점·추정 누출원, RViz)
return_to_home이 복귀를 시작하면(/return_to_home/status가 RETURNING·HOME) 목표 발행을 멈춘다.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray

from plume_sim.plume import declare_dataclass_params
from return_to_home.drive import DriveParams, GoalDriver
from source_seeking.run_log import RunLog
from source_seeking.seeker import SeekerCore, SeekerParams


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class SourceSeekingNode(Node):
    def __init__(self) -> None:
        super().__init__("source_seeking")
        self.declare_parameter("world", "corridor")
        rate = self.declare_parameter("rate", 10.0).value
        self.drive_mode = self.declare_parameter("drive_mode", "goal").value
        log_file = self.declare_parameter("log_file", "").value
        self.core = SeekerCore(declare_dataclass_params(self, SeekerParams))
        self.driver = GoalDriver(declare_dataclass_params(self, DriveParams))
        self.log = RunLog(log_file) if log_file else None

        self._odom: Odometry | None = None
        self._t_odom = 0.0
        self._conc: float | None = None
        self._wind = None
        self._rth = "IDLE"
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_subscription(Float32, "/gas/concentration", self.gas_concentration_cb, 10)
        self.create_subscription(Vector3Stamped, "/gas/wind", self.gas_wind_cb, 10)
        self.create_subscription(String, "/return_to_home/status", self.rth_status_cb, 10)
        self.pub_goal = self.create_publisher(PoseStamped, "/source_seeking/goal", 10)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10) if self.drive_mode == "cmd_vel" else None
        self.pub_state = self.create_publisher(String, "/source_seeking/state", 10)
        self.pub_src = self.create_publisher(PointStamped, "/source_seeking/source", 10)
        self.pub_mk = self.create_publisher(MarkerArray, "/source_seeking/markers", 10)
        self.create_timer(1.0 / rate, self._tick)
        self._n_events = 0
        self._tick_count = 0
        self._stopped = False
        self.get_logger().info("source_seeking started (채현) world=%s mode=%s drive=%s"
                               % (self.get_parameter("world").value, self.core.p.mode, self.drive_mode))

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def odom_cb(self, msg: Odometry) -> None:
        self._odom, self._t_odom = msg, self.now()

    def gas_concentration_cb(self, msg: Float32) -> None:
        self._conc = float(msg.data)

    def gas_wind_cb(self, msg: Vector3Stamped) -> None:
        self._wind = (msg.vector.x, msg.vector.y)

    def rth_status_cb(self, msg: String) -> None:
        self._rth = msg.data.split(" ", 1)[0]

    def _tick(self) -> None:
        if self._odom is None or self._conc is None:
            return
        t = self.now()
        if self._rth in ("RETURNING", "HOME"):     # 복귀는 return_to_home 담당
            if not self._stopped:
                self._stopped = True
                self.get_logger().info("return_to_home took over (%s), seeking paused" % self._rth)
            self._publish_state()
            return
        if t - self._t_odom > 1.0:                 # odom 끊김: 멈춤
            if self.pub_cmd:
                self.pub_cmd.publish(Twist())
            return
        pose = self._odom.pose.pose
        x, y, yaw = pose.position.x, pose.position.y, yaw_of(pose.orientation)
        core = self.core
        target = core.update(t, x, y, yaw, self._conc, self._wind)
        if self.driver.blocked(t, x, y, yaw, target):
            core.on_blocked()
            if self.drive_mode == "cmd_vel":
                self.driver.start_recovery(t)
        if target is not None:
            self._publish_goal(target, x, y)
        if self.pub_cmd:
            vx, wz = self.driver.command(t, x, y, yaw, target)
            cmd = Twist()
            cmd.linear.x, cmd.angular.z = float(vx), float(wz)
            self.pub_cmd.publish(cmd)

        for (_, st, why) in core.events[self._n_events:]:
            if why == "blocked by obstacle":
                self.get_logger().warn("[%s] %s" % (st, why))
            else:
                self.get_logger().info("[%s] %s" % (st, why))
        if len(core.events) != self._n_events:
            self._n_events = len(core.events)
            if core.state == "SOURCE_FOUND" and core.source:
                ps = PointStamped()
                ps.header.frame_id = self._odom.header.frame_id or "odom"
                ps.header.stamp = self.get_clock().now().to_msg()
                ps.point.x, ps.point.y = core.source
                self.pub_src.publish(ps)
            if self.log:
                self.log.write_summary(core, None, {"world": self.get_parameter("world").value, "mode": core.p.mode})
        self._publish_state()
        if self.log:
            self.log.row(t, x, y, yaw, self._conc, core, None, None)
        self._tick_count += 1
        if self._tick_count % 10 == 0:
            self._publish_markers()

    def _publish_goal(self, target, x, y) -> None:
        g = PoseStamped()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = self._odom.header.frame_id or "odom"
        g.pose.position.x, g.pose.position.y = float(target[0]), float(target[1])
        yaw = math.atan2(target[1] - y, target[0] - x)
        g.pose.orientation.z, g.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        self.pub_goal.publish(g)

    def _publish_state(self) -> None:
        c = self.core
        self.pub_state.publish(String(data="%s | %s | conc %.1f ppm" % (c.state, c.reason, c.c_f or 0.0)))

    def _publish_markers(self) -> None:
        core, frame = self.core, self._odom.header.frame_id or "odom"
        stamp = self.get_clock().now().to_msg()
        ma = MarkerArray()
        for mid, pts, rgba, scale in ((0, core.tabu, (1.0, 0.1, 0.1, 0.35), 0.6),
                                      (1, core.obstacle_pts, (0.3, 0.3, 0.3, 0.6), 0.25),
                                      (2, [core.source] if core.source else [], (0.1, 0.9, 0.2, 0.9), 0.5)):
            m = Marker()
            m.header.frame_id, m.header.stamp = frame, stamp
            m.ns, m.id, m.type, m.action = "source_seeking", mid, Marker.SPHERE_LIST, Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = scale
            m.color.r, m.color.g, m.color.b, m.color.a = rgba
            m.points = [Point(x=float(p[0]), y=float(p[1]), z=0.1) for p in pts]
            ma.markers.append(m)
        self.pub_mk.publish(ma)

    def shutdown(self) -> None:
        if self.pub_cmd:
            self.pub_cmd.publish(Twist())
        if self.log:
            self.log.close(self.core, None, {"world": self.get_parameter("world").value, "mode": self.core.p.mode})


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SourceSeekingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
