"""트윈에 배터리가 없어 /odom 이동 거리와 시간에 비례해 줄어드는 가상 배터리를 /battery로 낸다.

  ros2 param set /battery_sim percent 25.0     # 주행 중 배터리를 강제로 낮춰 복귀 시험
"""

from __future__ import annotations

import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from return_to_home.battery import BatteryModel, BatteryParams
from return_to_home.params import declare_dataclass_params


class BatterySimNode(Node):
    def __init__(self) -> None:
        super().__init__("battery_sim")
        self.model = BatteryModel(declare_dataclass_params(self, BatteryParams))
        rate = self.declare_parameter("rate", 5.0).value
        self.declare_parameter("percent", -1.0)   # 실행 중 설정하면 잔량을 덮어쓴다
        self.add_on_set_parameters_callback(self._on_params)
        self._odom: Odometry | None = None
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self._pub = self.create_publisher(BatteryState, "/battery", 10)
        self.create_timer(1.0 / rate, self._tick)
        self._warned: set[int] = {lv for lv in (50, 30, 20, 10) if self.model.percent <= lv}  # 시작부터 낮으면 경고 생략
        self.get_logger().info("battery_sim started (채현) %.0f%%, %.2f %%/m, %.3f %%/s" % (
            self.model.percent, self.model.p.drain_per_meter, self.model.p.drain_per_second))

    def _on_params(self, params) -> SetParametersResult:
        for p in params:
            if p.name == "percent" and p.value >= 0.0:
                self.model.percent = float(p.value)
                self.get_logger().warn("battery set to %.1f%%" % p.value)
        return SetParametersResult(successful=True)

    def odom_cb(self, msg: Odometry) -> None:
        self._odom = msg

    def _tick(self) -> None:
        if self._odom is None:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        pos = self._odom.pose.pose.position
        pct = self.model.update(t, pos.x, pos.y)
        for level in (50, 30, 20, 10):
            if pct <= level and level not in self._warned:
                self._warned.add(level)
                self.get_logger().warn("battery %.1f%%" % pct)
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = pct / 100.0
        msg.present = True
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BatterySimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
