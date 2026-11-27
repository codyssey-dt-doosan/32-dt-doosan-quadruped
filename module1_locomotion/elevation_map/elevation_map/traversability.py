"""통과 가능 판정(⑤-2, traversability 모드) — 그리드 값 1.0 막힘 / 0.0 통과 지면 / NaN 미관측.

mpc_controller는 그대로 '값 > 0.15 = 막힘'으로 읽는다. 규칙(오프라인 설계 패널 3안 중 채택 — vault
DT-작업노트-2026-11-27-C안-경사-인지-설계 §4):
1. 점을 방위별로 빔 고도 순(가까운 → 먼)으로 늘어놓고, IMU roll·pitch로 중력 기준 높이 z를 낸다(발밑 지면 = 0).
2. 같은 방위 연속 두 점의 중력 기준 높이차가 step_tol + 수평거리·tan(max_slope)를 넘으면 막는다:
   오르면 앞 점~이 점, 내려가면 빔이 앞 점 높이 아래로 내려간 곳(그늘)~이 점 광선 구간.
3. 방위 첫 점은 발밑과 비교: 지면선(다음 점·다다음 점과 잇는 두 직선)을 발밑까지 연장한 높이와 몸체 기준 높이 중
   덜 극단인 쪽(부호가 다르면 0)이 ±max_step을 넘으면 턱·낙차. 경사로 위·들머리·끝 전이에서는 어느 하나가 0 근처다.
4. 둘레 연장 높이의 25·75 % 분위가 같은 쪽이면 몸체가 뜨거나 묻힌 것으로 보고 발밑 기준을 옮긴다(legged는 ≈ 0).
5. 가장 낮은 빔이 이웃 방위까지 안 돌아온 방위는 바닥이 빔보다 가파르게 꺼진 것(하강 계단·구멍) — 닿았어야 할 곳부터 막는다.
6. 나머지 관측 점의 셀은 통과, 막힘이 통과를 덮는다. 막힘 구간은 반 칸 앞당겨 칸 경계에 걸친 턱도 잡는다.
"""
import math

import numpy as np

AZ = 359          # 방위 칸 — go2 라이다 수평 360개가 −π~π 양끝을 포함(simulation/models/go2/model.sdf)
LOW_BEAM = -0.5   # 라이다 가장 낮은 빔 고도(rad, 같은 파일)


def imu_roll_pitch(x: float, y: float, z: float, w: float):
    """IMU 쿼터니언 → (roll, pitch) rad(ROS, pitch + = 앞 숙임). 유한하지 않거나 길이 0이면 None."""
    q = np.array([x, y, z, w], dtype=float)
    n = np.linalg.norm(q)
    if not np.isfinite(q).all() or n < 1e-9:
        return None
    x, y, z, w = q / n
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    return float(roll), float(pitch)


def _rot_level(roll, pitch):
    """몸체 → 수평(yaw 유지) 회전 Ry(pitch)·Rx(roll)."""
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    return np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])


def _less_extreme(a, b):
    """같은 부호면 0에 가까운 쪽, 부호가 다르면 0."""
    return np.clip(a, np.minimum(b, 0), np.maximum(b, 0))


def trav_grid(xyz, roll, pitch, res, size, sensor_height, min_range, max_step, max_slope_deg, step_tol):
    """(N,3) 센서 프레임 포인트 + 몸체 roll·pitch → (n,n) float32 막힘 표시 그리드. 행 = y, 열 = x, 중력 수평·몸체 yaw."""
    n, half, ray_max = int(round(size / res)), size / 2, 0.75 * size
    grid = np.full((n, n), np.nan, np.float32)

    def mark(x, y, v):
        r, c = np.floor((y + half) / res).astype(int), np.floor((x + half) / res).astype(int)
        ok = (r >= 0) & (r < n) & (c >= 0) & (c < n)
        grid[r[ok], c[ok]] = v

    xyz = xyz[np.isfinite(xyz).all(axis=1)]  # NaN 하나가 lift(분위)를 NaN으로 만들어 막힘을 지운다
    rs = np.hypot(xyz[:, 0], xyz[:, 1])
    az = np.round((np.arctan2(xyz[:, 1], xyz[:, 0]) + math.pi) * (AZ / (2 * math.pi))).astype(int) % AZ
    low = np.full(AZ, np.inf)
    np.minimum.at(low, az, xyz[:, 2] / np.maximum(rs, 1e-6))  # 방위별 가장 낮게 돌아온 빔(tan 고도, 자기 반사 포함)
    low = np.minimum(low, np.minimum(np.roll(low, 1), np.roll(low, -1)))  # 이웃 방위로 넘어간 점도 인정
    keep = rs >= min_range
    xyz, rs, az = xyz[keep], rs[keep], az[keep]
    if len(xyz) < 2:
        return grid
    h, tan_s, R = sensor_height, math.tan(math.radians(max_slope_deg)), _rot_level(roll, pitch)
    o = np.lexsort((xyz[:, 2] / rs, az))                      # 방위별로 빔 고도 순(가까운 → 먼)
    zb = xyz[o, 2] + h                                        # 몸체 기준 높이
    g = xyz[o] @ R.T
    az, x, y, z = az[o], g[:, 0], g[:, 1], g[:, 2] + h        # 중력 기준, 발밑 지면 = 0
    r = np.hypot(x, y)
    first = np.r_[True, az[1:] != az[:-1]]
    nxt = np.r_[~first[1:], False]
    sl = np.where(nxt, np.diff(z, append=0) / np.maximum(np.diff(r, append=0), 1e-3), 0.0)
    j = np.minimum(np.arange(len(z)) + 2, len(z) - 1)
    sl2 = np.where(nxt & (az[j] == az), (z[j] - z) / np.maximum(r[j] - r, 1e-3), sl)
    ext = _less_extreme(z - r * sl, z - r * sl2)              # 지면선을 발밑까지 연장한 높이(두 직선 중 덜 극단)
    gnd = first & nxt & (np.abs(sl) <= tan_s)
    lift = _less_extreme(*np.percentile(ext[gnd], [25, 75])) if gnd.any() else 0.0  # 몸체가 뜬(−)·묻힌(+) 높이
    z, zb, ext, h = z - lift, zb - lift, ext - lift, h - lift
    rp = np.where(first, min_range, np.r_[0.0, r[:-1]])       # 앞 점(방위 첫 점은 발밑)
    zp = np.where(first, 0.0, np.r_[0.0, z[:-1]])
    dz = np.where(first, _less_extreme(zb, ext), z - zp)
    bad = np.abs(dz) > np.where(first, max_step, step_tol + np.abs(r - rp) * tan_s)
    mark(x, y, 0.0)
    a = np.arange(AZ) * (2 * math.pi / AZ) - math.pi
    d = np.c_[np.cos(a), np.sin(a), np.full(AZ, math.tan(LOW_BEAM))] @ R.T  # 방위별 가장 낮은 빔 방향(중력 기준)
    dr = np.hypot(d[:, 0], d[:, 1])
    hole = (low > math.tan(LOW_BEAM + 0.025)) & (d[:, 2] < 0)
    fr = np.full(AZ, ray_max)
    fr[az[first]] = r[first]
    rn = np.r_[np.where(dz < 0, np.clip(r * (h - zp) / np.maximum(h - z, 1e-3), rp, r), rp)[bad], (h * dr / -d[:, 2])[hole]]
    rf = np.r_[r[bad], fr[hole]]
    ux, uy = np.r_[x[bad] / r[bad], (d[:, 0] / dr)[hole]], np.r_[y[bad] / r[bad], (d[:, 1] / dr)[hole]]
    t = np.minimum(rn[:, None] - res / 2 + np.arange(0, ray_max, res / 2), np.minimum(rf, ray_max)[:, None])
    mark(ux[:, None] * t, uy[:, None] * t, 1.0)
    return grid
