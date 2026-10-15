"""시뮬 SDF에서 게이지·장애물·카메라 사양을 읽는다. ROS 없이 동작.

포즈는 (x, y, z, yaw)만 합성한다. 시뮬 맵의 모델은 roll·pitch가 0이다.
"""

from dataclasses import dataclass
import math
import os
import xml.etree.ElementTree as ET

VELOCITY_BODY_HEIGHT = 0.4      # worlds/*.sdf의 go2 include z
LEGGED_BODY_HEIGHT = 0.295      # mpc_controller balance_node "height"
FLOOR_TOP = 0.05                # 윗면이 이보다 낮으면 바닥으로 보고 장애물에서 뺀다
ROBOT_MODELS = ("go2",)


@dataclass(frozen=True)
class CameraSpec:
    x: float                    # base_link 기준 장착 위치
    y: float
    z: float
    pitch: float                # SDF pitch (양수 = 아래로 숙임)
    hfov: float
    width: int
    height: int

    @property
    def fx(self) -> float:
        return (self.width / 2) / math.tan(self.hfov / 2)

    @property
    def half_vfov(self) -> float:
        return math.atan((self.height / 2) / self.fx)


@dataclass(frozen=True)
class Obstacle:
    name: str                   # "<모델>/<링크>"
    kind: str                   # "box" | "cylinder"
    x: float
    y: float
    yaw: float
    sx: float                   # box 크기 (cylinder는 0)
    sy: float
    r: float                    # cylinder 반지름 (box는 0)
    z_min: float
    z_max: float


@dataclass(frozen=True)
class Gauge:
    name: str
    x: float
    y: float
    z: float
    yaw: float                  # 원판이 보는 방향 (모델 +x)
    dial_r: float
    face: float                 # 모델 원점 → 원판 앞면 (+x, m)


@dataclass
class WorldMap:
    gauges: list
    obstacles: list
    camera: CameraSpec
    thermal: CameraSpec


def _pose(el) -> tuple:
    """<pose> → (x, y, z, pitch, yaw). 없으면 0."""
    p = el.find("pose") if el is not None else None
    if p is None or not (p.text or "").strip():
        return 0.0, 0.0, 0.0, 0.0, 0.0
    v = [float(t) for t in p.text.split()]
    v += [0.0] * (6 - len(v))
    return v[0], v[1], v[2], v[4], v[5]


def _compose(parent: tuple, child: tuple) -> tuple:
    px, py, pz, _, pyaw = parent
    cx, cy, cz, cpitch, cyaw = child
    c, s = math.cos(pyaw), math.sin(pyaw)
    return px + c * cx - s * cy, py + s * cx + c * cy, pz + cz, cpitch, pyaw + cyaw


def _collisions(model_el, model_pose: tuple, name: str) -> list:
    out = []
    for link in model_el.findall("link"):
        lp = _compose(model_pose, _pose(link))
        for col in link.findall("collision"):
            x, y, z, _, yaw = _compose(lp, _pose(col))
            geo = col.find("geometry")
            box, cyl = geo.find("box"), geo.find("cylinder")
            oname = f"{name}/{link.get('name')}"
            if box is not None:
                sx, sy, sz = (float(t) for t in box.find("size").text.split())
                o = Obstacle(oname, "box", x, y, yaw, sx, sy, 0.0, z - sz / 2, z + sz / 2)
            elif cyl is not None:
                r, length = float(cyl.findtext("radius")), float(cyl.findtext("length"))
                o = Obstacle(oname, "cylinder", x, y, 0.0, 0.0, 0.0, r, z - length / 2, z + length / 2)
            else:
                continue                                    # mesh 등은 지금 맵에 없음
            if o.z_max > FLOOR_TOP:
                out.append(o)
    return out


def _gauge(model_el, name: str, pose: tuple) -> Gauge:
    dial = model_el.find("link[@name='dial']")
    if dial is None:
        raise ValueError(f"{name}: gauge 모델에 dial 링크가 없음")
    cyl = dial.find("visual/geometry/cylinder")
    r, length = float(cyl.findtext("radius")), float(cyl.findtext("length"))
    x, y, z, _, yaw = pose
    return Gauge(name, x, y, z, yaw, r, _pose(dial)[0] + length / 2)


def _camera(model_el, sensor_name: str) -> CameraSpec:
    for link in model_el.findall("link"):
        s = link.find(f"sensor[@name='{sensor_name}']")
        if s is None:
            continue
        x, y, z, pitch, _ = _compose(_pose(link), _pose(s))
        cam = s.find("camera")
        return CameraSpec(
            x, y, z, pitch, float(cam.findtext("horizontal_fov")),
            int(cam.findtext("image/width")), int(cam.findtext("image/height")),
        )
    raise ValueError(f"go2 모델에 센서 {sensor_name}가 없음")


def load_world(world_file: str, models_dir: str) -> WorldMap:
    world = ET.parse(world_file).getroot().find("world")
    if world is None:
        raise ValueError(f"{world_file}: <world>가 없음")
    cache: dict = {}

    def model(kind: str):
        if kind not in cache:
            path = os.path.join(models_dir, kind, "model.sdf")
            if not os.path.exists(path):
                raise FileNotFoundError(path)
            cache[kind] = ET.parse(path).getroot().find("model")
        return cache[kind]

    gauges, obstacles = [], []
    for m in world.findall("model"):
        if m.get("name") in ROBOT_MODELS:
            continue
        obstacles += _collisions(m, _pose(m), m.get("name"))
    for inc in world.findall("include"):
        kind = inc.findtext("uri", "").strip().removeprefix("model://")
        if kind in ROBOT_MODELS:
            continue
        name = inc.findtext("name", kind).strip()
        pose = _pose(inc)
        m = model(kind)
        obstacles += _collisions(m, pose, name)
        if kind == "gauge":
            gauges.append(_gauge(m, name, pose))
    robot = model("go2")
    return WorldMap(gauges, obstacles, _camera(robot, "camera"), _camera(robot, "thermal"))


def resolve_world(world: str, world_file: str = "", models_dir: str = "",
                  sim_share: str = "") -> tuple:
    """(월드 파일, models 폴더). 직접 준 경로 > sim_share > 설치된 simulation 패키지."""
    if not (world_file and models_dir) and not sim_share:
        from ament_index_python.packages import get_package_share_directory  # ROS 환경에서만
        sim_share = get_package_share_directory("simulation")
    wf = world_file or os.path.join(sim_share, "worlds", f"{world}.sdf")
    md = models_dir or os.path.join(sim_share, "models")
    for path in (wf, md):
        if not os.path.exists(path):
            raise FileNotFoundError(path)
    return wf, md


def body_height_for(locomotion: str) -> float:
    heights = {"velocity": VELOCITY_BODY_HEIGHT, "legged": LEGGED_BODY_HEIGHT}
    if locomotion not in heights:
        raise ValueError(f"locomotion은 velocity | legged (받은 값: {locomotion})")
    return heights[locomotion]
