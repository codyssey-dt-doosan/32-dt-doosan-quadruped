"""탱크 누출원의 농도장을 가상 플랜트에 올린다.

/odom 위치에서 플룸 모델을 샘플링해 가스 센서·풍향계를 흉내 낸다.
  /gas/concentration  std_msgs/Float32             ppm (노이즈 포함)
  /gas/wind           geometry_msgs/Vector3Stamped  풍속 벡터, 월드 좌표 (노이즈 포함)
  /gas/field          nav_msgs/OccupancyGrid       평균 농도 지도 (log 스케일 0..100, RViz용, latched)
  /gas/markers        visualization_msgs/MarkerArray 실제 누출원·정체 구역 (RViz용, latched)
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray

from plume_sim.plume import GasField, GasFieldParams, declare_dataclass_params


class PlumeSimNode(Node):
    def __init__(self) -> None:
        super().__init__("plume_sim")
        self.declare_parameter("world", "corridor")
        rate = self.declare_parameter("rate", 10.0).value
        self.declare_parameter("field_bounds", [-12.5, -9.0, 12.5, 9.0])  # 농도 지도 범위
        self.params = declare_dataclass_params(self, GasFieldParams)
        self.field = GasField(self.params)
        self._odom: Odometry | None = None
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self._pub_c = self.create_publisher(Float32, "/gas/concentration", 10)
        self._pub_w = self.create_publisher(Vector3Stamped, "/gas/wind", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub_grid = self.create_publisher(OccupancyGrid, "/gas/field", latched)
        self._pub_mk = self.create_publisher(MarkerArray, "/gas/markers", latched)
        self.create_timer(1.0 / rate, self._tick)
        self._static_done = False
        p = self.params
        self.get_logger().info(
            "plume_sim started (채현) world=%s leak=(%.1f, %.1f) %.0f ppm, wind %.1f m/s -> %.0f deg, decoy %d"
            % (self.get_parameter("world").value, p.source_x, p.source_y, p.release_rate, p.wind_speed,
               math.degrees(p.wind_direction), len(self.field.decoys)))

    def odom_cb(self, msg: Odometry) -> None:
        self._odom = msg

    def _tick(self) -> None:
        if self._odom is None:
            return
        frame = self._odom.header.frame_id or "odom"
        if not self._static_done:
            self._publish_static(frame)
            self._static_done = True
        pos = self._odom.pose.pose.position
        self._pub_c.publish(Float32(data=float(self.field.sample(pos.x, pos.y))))
        wx, wy = self.field.wind()
        w = Vector3Stamped()
        w.header.stamp = self.get_clock().now().to_msg()
        w.header.frame_id = frame
        w.vector.x, w.vector.y = wx, wy
        self._pub_w.publish(w)

    def _publish_static(self, frame: str) -> None:
        stamp = self.get_clock().now().to_msg()
        x0, y0, x1, y1 = self.get_parameter("field_bounds").value
        res = 0.2
        w, h = int((x1 - x0) / res), int((y1 - y0) / res)
        g = OccupancyGrid()
        g.header.frame_id, g.header.stamp = frame, stamp
        g.info.resolution, g.info.width, g.info.height = res, w, h
        g.info.origin.position.x, g.info.origin.position.y = x0, y0
        g.info.origin.orientation.w = 1.0
        cmax = math.log1p(self.field.mean(self.params.source_x, self.params.source_y))
        g.data = [int(max(0, min(100, 100 * math.log1p(self.field.mean(x0 + (i + 0.5) * res, y0 + (j + 0.5) * res)) / cmax)))
                  for j in range(h) for i in range(w)]
        self._pub_grid.publish(g)

        ma = MarkerArray()
        pts = [(self.params.source_x, self.params.source_y, 0.5, (1.0, 0.85, 0.0))] + \
              [(d[0], d[1], 0.35, (1.0, 0.4, 0.0)) for d in self.field.decoys]
        for k, (x, y, size, rgb) in enumerate(pts):
            m = Marker()
            m.header.frame_id, m.header.stamp = frame, stamp
            m.ns, m.id, m.type, m.action = "gas_truth", k, Marker.SPHERE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.3
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = size
            m.color.r, m.color.g, m.color.b, m.color.a = rgb[0], rgb[1], rgb[2], 0.9
            ma.markers.append(m)
        self._pub_mk.publish(ma)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlumeSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
