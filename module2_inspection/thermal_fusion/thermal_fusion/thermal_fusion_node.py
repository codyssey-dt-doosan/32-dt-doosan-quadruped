"""열화상에서 과열 지점을 찾아 맵 위치로 묶고, 점검 goal·알람·상태를 낸다."""

import math

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from inspection_common.viewpoint import approach_point
from inspection_common.world_map import body_height_for, load_world, resolve_world
from thermal_fusion.align import k_from_spec, thermal_to_rgb_h, warp_to_rgb
from thermal_fusion.hotspot import find_hot_blobs
from thermal_fusion.locate import locate
from thermal_fusion.outputs import active_goal, state_line
from thermal_fusion.overlay import draw_blobs, fit_to, fuse
from thermal_fusion.thermal import MODES, to_temperature
from thermal_fusion.tracker import Tracker

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
PARAMS = [
    ("world", "corridor"), ("world_file", ""), ("models_path", ""), ("locomotion", "velocity"),
    ("thermal_mode", "brightness"), ("l8_min_k", 253.15), ("l8_res", 3.0),
    ("t_detect", 240.0), ("t_confirm", 240.0), ("min_area", 20),
    ("merge_radius", 1.0), ("confirm_frames", 3), ("forget_s", 3.0), ("dwell_s", 5.0),
    ("arrive_tol", 0.4), ("stop_speed", 0.05), ("inspect_timeout", 60.0), ("max_attempts", 2),
    ("recheck_s", 0.0), ("approach_dist", 1.5), ("robot_radius", 0.35), ("max_range", 15.0),
    ("odom_timeout", 1.0), ("image_timeout", 1.0),
    ("overlay_lo", 20.0), ("overlay_hi", 90.0), ("publish_overlay", True),
]


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class ThermalFusionNode(Node):
    def __init__(self) -> None:
        super().__init__("thermal_fusion")
        for name, default in PARAMS:
            self.declare_parameter(name, default)
        p = self._p
        if p("thermal_mode") not in MODES:                 # 시작 때 바로 실패 (콜백 중 죽지 않게)
            raise ValueError(f"thermal_mode는 {MODES} 중 하나 (받은 값: {p('thermal_mode')})")
        self._bridge = CvBridge()
        self._pose = None                                  # (x, y, yaw) map
        self._speed = 0.0
        self._odom_t = None
        self._img_t = None                                 # 마지막 열화상 수신 시각
        self._rgb = None                                   # 겹쳐 그리기용 최신 RGB 메시지
        self._hits = []
        self._hotspots_key = None
        self._odom_lost = False                            # 경고 로그를 한 번만 남기기 위함
        self._map = None
        try:
            self._map = load_world(*resolve_world(p("world"), p("world_file"), p("models_path")))
        except Exception as e:  # noqa: BLE001 — 맵이 없으면 위치·goal 없이 IDLE
            self.get_logger().warn(f"맵을 못 읽어 위치 추정·goal을 끕니다: {e}")
        self._body_h = body_height_for(p("locomotion"))
        obstacles = self._map.obstacles if self._map else []
        self._tracker = Tracker(
            plan_goal=lambda tx, ty, fx, fy: approach_point(
                tx, ty, fx, fy, obstacles, p("approach_dist"), p("robot_radius")),
            t_confirm=p("t_confirm"), merge_radius=p("merge_radius"),
            confirm_frames=p("confirm_frames"), forget_s=p("forget_s"), dwell_s=p("dwell_s"),
            arrive_tol=p("arrive_tol"), stop_speed=p("stop_speed"),
            inspect_timeout=p("inspect_timeout"), max_attempts=p("max_attempts"),
            recheck_s=p("recheck_s"),
        )

        self.create_subscription(Image, "/thermal/image", self.thermal_image_cb, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/image", self.camera_image_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.pub_0 = self.create_publisher(Bool, "/inspection/thermal_alert", 10)
        self.pub_goal = self.create_publisher(PoseStamped, "/inspection/thermal_goal", 10)
        self.pub_spots = self.create_publisher(PoseArray, "/inspection/thermal_hotspots", LATCHED)
        self.pub_overlay = self.create_publisher(Image, "/inspection/thermal_overlay", qos_profile_sensor_data)
        self.pub_state = self.create_publisher(String, "/inspection/thermal_state", 10)
        self.create_timer(0.5, self._tick)
        self.get_logger().info("thermal_fusion started (운학)")

    def _p(self, name: str):
        return self.get_parameter(name).value

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _odom_age(self):
        return None if self._odom_t is None else self._now() - self._odom_t

    def odom_cb(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        self._pose = (pos.x, pos.y, yaw_of(msg.pose.pose.orientation))
        v = msg.twist.twist.linear
        self._speed = math.hypot(v.x, v.y)
        self._odom_t = self._now()

    def camera_image_cb(self, msg: Image) -> None:
        self._rgb = msg

    def thermal_image_cb(self, msg: Image) -> None:
        p = self._p
        self._img_t = self._now()
        try:
            raw = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            self.get_logger().warn(f"cv_bridge: {e}")
            return
        temp = to_temperature(raw, p("thermal_mode"), p("l8_min_k"), p("l8_res"))
        blobs = find_hot_blobs(temp, p("t_detect"), p("min_area"))
        self._hits = []
        age = self._odom_age()
        if self._map is not None and age is not None and age <= p("odom_timeout"):
            rx, ry, ryaw = self._pose
            for b in blobs:
                hit, _ = locate(b.cx, b.cy, rx, ry, ryaw, self._map.thermal, self._body_h,
                                self._map.obstacles, p("max_range"))
                if hit is not None:                        # 바닥·허공은 버린다
                    self._tracker.observe(hit.x, hit.y, b.t_max, rx, ry, self._now())
                    self._hits.append(hit)
        if p("publish_overlay") and self.pub_overlay.get_subscription_count() > 0:
            self._publish_overlay(temp, blobs)

    def _publish_overlay(self, temp, blobs) -> None:
        if self._map is None or self._rgb is None:
            return
        try:
            bgr = self._bridge.imgmsg_to_cv2(self._rgb, desired_encoding="bgr8")
        except CvBridgeError:
            return
        cam, th = self._map.camera, self._map.thermal
        bgr = fit_to(bgr, cam.width, cam.height)          # 실제 영상 크기가 맵 설정과 달라도 합성되게
        # 실제 거리로 정렬. 장애물 안에서 시작한 광선은 depth 0이 되어 0으로 나누므로 0.3 m로 하한을 둔다.
        depth = max(min((h.depth for h in self._hits), default=1.5), 0.3)
        h = thermal_to_rgb_h(k_from_spec(cam), k_from_spec(th), baseline_z=cam.z - th.z, depth=depth)
        warped, valid = warp_to_rgb(temp, h, (cam.width, cam.height))
        img = draw_blobs(fuse(bgr, warped, valid, self._p("overlay_lo"), self._p("overlay_hi")),
                         blobs, h, self._tracker.alert())
        out = self._bridge.cv2_to_imgmsg(img, encoding="bgr8")
        out.header = self._rgb.header
        self.pub_overlay.publish(out)

    def _tick(self) -> None:
        p = self._p
        age = self._odom_age()
        odom_ok = self._pose is not None and age is not None and age <= p("odom_timeout")
        if odom_ok:
            self._tracker.update(self._pose[0], self._pose[1], self._speed, self._now())
        else:
            self._tracker.forget(self._now())              # 위치 없이도 오래된 후보는 정리
        if not odom_ok and not self._odom_lost and self._odom_t is not None:
            self._odom_lost = True                         # 시작 직후(아직 미수신)는 경고하지 않는다
            self.get_logger().warn("/odom 끊김: 과열 지점 goal·도착 판정을 멈춥니다")
        elif odom_ok and self._odom_lost:
            self._odom_lost = False
            self.get_logger().info("/odom 복구")
        self.pub_0.publish(Bool(data=self._tracker.alert()))

        g = PoseStamped()
        g.header.stamp = self.get_clock().now().to_msg()
        goal = active_goal(self._tracker.goal(), age, p("odom_timeout"))
        if goal is not None:
            g.header.frame_id = "map"
            g.pose.position.x, g.pose.position.y = goal[0], goal[1]
            g.pose.orientation.z, g.pose.orientation.w = math.sin(goal[2] / 2), math.cos(goal[2] / 2)
        self.pub_goal.publish(g)                           # 비활성이면 빈 frame_id

        key = tuple((s.id, s.state, round(s.x, 2), round(s.y, 2)) for s in self._tracker.spots)
        if key != self._hotspots_key:
            self._hotspots_key = key
            arr = PoseArray()
            arr.header.frame_id = "map"
            arr.header.stamp = g.header.stamp
            for s in self._tracker.spots:
                pose = Pose()
                pose.position.x, pose.position.y = s.x, s.y
                pose.orientation.w = 1.0
                arr.poses.append(pose)
            self.pub_spots.publish(arr)

        unit = "lv" if p("thermal_mode") == "brightness" else "C"
        no_image = self._img_t is None or self._now() - self._img_t > p("image_timeout")
        text = state_line(self._tracker.headline(), unit, no_map=self._map is None, no_image=no_image,
                          no_odom=not odom_ok)
        self.pub_state.publish(String(data=text))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ThermalFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
