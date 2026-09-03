"""복도/공장 웨이포인트와 점검 정차점을 순회한다."""

from __future__ import annotations

import math
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
import yaml


class PatrolPathNode(Node):
    def __init__(self) -> None:
        super().__init__("patrol_path")
        self.declare_parameter("world", "corridor")
        world = self.get_parameter("world").get_parameter_value().string_value

        share = Path(get_package_share_directory("patrol_path"))
        yaml_path = share / "config" / f"waypoints_{world}.yaml"
        if not yaml_path.exists():
            yaml_path = share / "config" / "waypoints_corridor.yaml"
        with yaml_path.open(encoding="utf-8") as f:
            self._wps = yaml.safe_load(f)["waypoints"]

        self._idx = 0
        self._pose = (0.0, 0.0)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self._goal_pub = self.create_publisher(PoseStamped, "/patrol/goal", 10)
        self._status_pub = self.create_publisher(String, "/mission/status", 10)
        self.create_timer(0.5, self._tick)
        self.get_logger().info(f"patrol_path started (태우) world={world} n={len(self._wps)}")

    def odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._pose = (p.x, p.y)

    def _tick(self) -> None:
        wp = self._wps[self._idx]
        dx = wp["x"] - self._pose[0]
        dy = wp["y"] - self._pose[1]
        if math.hypot(dx, dy) < 0.4:
            self._idx = (self._idx + 1) % len(self._wps)
            wp = self._wps[self._idx]

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = float(wp["x"])
        goal.pose.position.y = float(wp["y"])
        yaw = float(wp["yaw"])
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        self._goal_pub.publish(goal)

        status = String()
        status.data = f"goto:{wp['name']}"
        self._status_pub.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PatrolPathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
