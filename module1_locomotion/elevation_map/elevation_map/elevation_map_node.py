"""LiDAR 포인트를 로봇 중심 XY 그리드로 빈닝해 셀별 최대 높이(지면 기준)를 낸다."""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


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
    # ponytail: 몸체 roll/pitch 보정 없음. 경사 지형 필요하면 /imu 자세로 회전
    np.fmax.at(grid, (row, col), z.astype(np.float32))
    return grid


class ElevationMapNode(Node):
    def __init__(self) -> None:
        super().__init__("elevation_map")
        self.declare_parameter("resolution", 0.1)
        self.declare_parameter("size", 4.0)
        self.declare_parameter("sensor_height", 0.56)  # 몸체 정착 z 0.36 + 라이다 0.20 (model.sdf)
        self.declare_parameter("min_range", 0.5)  # 몸체·다리 자기 반사 제외
        self.create_subscription(PointCloud2, "/points", self.points_cb, 10)
        self.pub = self.create_publisher(Float32MultiArray, "/elevation_map", 10)
        self.get_logger().info("elevation_map started (도훈)")

    def points_cb(self, msg: PointCloud2) -> None:
        res = self.get_parameter("resolution").value
        size = self.get_parameter("size").value
        h = self.get_parameter("sensor_height").value
        r0 = self.get_parameter("min_range").value
        xyz = read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]  # gz는 미검출을 inf로 줌
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
