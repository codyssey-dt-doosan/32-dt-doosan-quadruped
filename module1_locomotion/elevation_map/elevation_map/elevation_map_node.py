"""LiDAR 포인트를 로봇 중심 XY 그리드로 빈닝해 셀별 최대 높이(지면 기준)를 낸다.

traversability 파라미터를 켜면 값이 높이가 아니라 막힘 표시(1.0 막힘 / 0.0 통과 지면 / NaN 미관측)다 — traversability.py.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from elevation_map.traversability import imu_roll_pitch, trav_grid

ATT_MAX_AGE = 0.5  # traversability: 이보다 낡은 /imu 자세면 발행 보류(s, sim) — mpc는 1 s 무수신에 정지


def bin_points(xyz: np.ndarray, resolution: float, size: float, sensor_height: float, min_range: float = 0.0) -> np.ndarray:
    """(N,3) 센서 프레임 포인트 → (n,n) 그리드. 값=셀 내 최대 z + sensor_height (0=지면), 빈 셀=NaN.

    row = y축, col = x축. 로봇이 그리드 중앙, 전방이 col 증가 방향. min_range 안쪽(자기 몸체)은 버림.
    """
    n = int(round(size / resolution))
    grid = np.full((n, n), np.nan, dtype=np.float32)
    if xyz.size == 0:
        return grid
    xyz = xyz[np.hypot(xyz[:, 0], xyz[:, 1]) >= min_range]
    idx = np.floor((xyz[:, :2] + size / 2) / resolution).astype(int)
    ok = (idx >= 0).all(axis=1) & (idx < n).all(axis=1)
    col, row, z = idx[ok, 0], idx[ok, 1], xyz[ok, 2] + sensor_height
    # ponytail: 몸체 roll/pitch 보정 없음(높이 모드). 경사 지형은 traversability 모드가 /imu로 보정
    np.fmax.at(grid, (row, col), z.astype(np.float32))
    return grid


class ElevationMapNode(Node):
    def __init__(self) -> None:
        super().__init__("elevation_map")
        self.declare_parameter("resolution", 0.1)
        self.declare_parameter("size", 4.0)
        self.declare_parameter("sensor_height", 0.56)  # 몸체 정착 z 0.36 + 라이다 0.20 (model.sdf)
        self.declare_parameter("min_range", 0.5)  # 몸체·다리 자기 반사 제외
        self.declare_parameter("traversability", False)  # true면 /elevation_map 값 = 막힘 표시(높이 아님)
        self.declare_parameter("trav_max_step", 0.15)  # 발밑과 첫 지면 사이 허용 턱·낙차(m)
        self.declare_parameter("trav_max_slope_deg", 13.0)  # 연속 반사점 사이 허용 기울기 — 12° 경사로 통과
        self.declare_parameter("trav_step_tol", 0.03)  # 연속 반사점 높이차 여유(m)
        self.att = None  # (roll, pitch, 받은 sim 시각 s)
        self.trav = self.get_parameter("traversability").value
        if self.trav:
            self.create_subscription(Imu, "/imu", self.imu_cb, 10)
        self.create_subscription(PointCloud2, "/points", self.points_cb, 10)
        self.pub = self.create_publisher(Float32MultiArray, "/elevation_map", 10)
        self.get_logger().info(f"elevation_map started (도훈, {'traversability: 막힘 표시' if self.trav else '높이'})")

    def imu_cb(self, msg: Imu) -> None:
        q = msg.orientation
        rp = imu_roll_pitch(q.x, q.y, q.z, q.w)
        if rp is not None:
            self.att = (*rp, self.get_clock().now().nanoseconds * 1e-9)

    def points_cb(self, msg: PointCloud2) -> None:
        res = self.get_parameter("resolution").value
        size = self.get_parameter("size").value
        h = self.get_parameter("sensor_height").value
        r0 = self.get_parameter("min_range").value
        xyz = read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]  # gz는 미검출을 inf로 줌
        if self.trav:
            now = self.get_clock().now().nanoseconds * 1e-9
            if self.att is None or not 0.0 <= now - self.att[2] <= ATT_MAX_AGE:  # 음수 = sim 시계 되감김(재기동)
                self.get_logger().warn("traversability: /imu 자세가 없거나 낡음 → 발행 보류", throttle_duration_sec=2.0)
                return
            grid = trav_grid(
                xyz, self.att[0], self.att[1], res, size, h, r0,
                self.get_parameter("trav_max_step").value,
                self.get_parameter("trav_max_slope_deg").value,
                self.get_parameter("trav_step_tol").value,
            )
        else:
            grid = bin_points(xyz, res, size, h, r0)
        out = Float32MultiArray()
        out.layout.dim = [
            MultiArrayDimension(label="y", size=grid.shape[0], stride=grid.size),
            MultiArrayDimension(label="x", size=grid.shape[1], stride=grid.shape[1]),
        ]
        out.data = grid.ravel().tolist()
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ElevationMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
