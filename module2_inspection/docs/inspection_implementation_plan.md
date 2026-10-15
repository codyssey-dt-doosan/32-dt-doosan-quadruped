# module2 점검 인터페이스 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 게이지 점검 위치 계산·상태 토픽과 열화상 과열 지점 추적(위치·goal·알람)을 공용 패키지 `inspection_common` 위에 구현한다.

**Architecture:** ROS 없이 import되는 순수 함수 층(`inspection_common`, 각 패키지의 `*.py`)과 얇은 ROS 노드 층으로 나눈다. 맵은 `simulation/` SDF를 읽어서 쓰고, 출력은 ROS 표준 메시지 토픽이다. 다른 담당자의 파일은 고치지 않는다.

**Tech Stack:** Python 3, ROS 2 Jazzy (rclpy, ament_python), OpenCV, NumPy, xml.etree, pytest

**Spec:** `module2_inspection/docs/inspection_interface_design.md`

> **구현 후 변경 (2026-10-15).** 아래 코드 블록은 실제 코드에 맞춰 고쳐 두었지만, 최종 기준은 저장소의 코드와 테스트다.
> - Task 4: 게이지 yaw 테스트 허용오차 `abs=1e-4` (SDF의 yaw가 −1.5708로 반올림돼 있음)
> - Task 7: `test_l8_thermal_linear`는 `t[0].tolist()`로 비교 (계획의 `t.tolist()`는 2차원이라 틀림)
> - Task 7: `fuse`는 `t_hi == t_lo`여도 NaN이 안 나게 분모 하한, `fit_to`로 카메라 영상 크기를 맵 설정에 맞춤
> - Task 9: 재점검으로 되돌릴 때 `goal_since`, `inspect_start`도 초기화 (안 하면 바로 시간 초과). 회귀 테스트 `test_recheck_does_not_time_out_immediately`
> - Task 9: INSPECTING 진입 시 이전 시간 초과 사유 `reason`을 지움. 후보 정리를 `forget(now)`로 분리해 odom 없이도 호출
> - Task 10: 겹쳐 그리기 depth 하한 0.3 m, `thermal_mode`를 시작 시 검증, odom이 끊기면 상태 `IDLE | no odom` + 경고 로그
> - Task 11: README 표에 `/inspection/gauge_debug` 추가, YAML 실수 파라미터에 소수점 주석

## Global Constraints

- **커밋하지 않는다.** 각 Task 끝의 "기록" 단계는 바뀐 파일 목록을 남기는 것뿐이다. 커밋·푸시는 운학 님이 요청할 때만.
- 수정 가능한 경로: `module2_inspection/inspection_common/**`, `module2_inspection/gauge_ocr/**`, `module2_inspection/thermal_fusion/**`, `module2_inspection/README.md`, `module2_inspection/docs/**`. 그 밖(`simulation/`, `module1_locomotion/`, `module3_gas_safety/`, `monitoring/`, `patrol_path/`, 루트 README, `docs/architecture.md`, Dockerfile)은 **읽기만**.
- `simulation/package.xml`이 `gauge_ocr`, `thermal_fusion`에 의존하므로 우리 package.xml에 `simulation` 의존을 **넣지 않는다** (순환). 런타임 `get_package_share_directory("simulation")`만 쓴다.
- 기존 토픽 `/inspection/gauge`(`std_msgs/Float32`, 0.5 s, 값 없으면 발행 안 함), `/inspection/thermal_alert`(`std_msgs/Bool`, 0.5 s)는 이름·타입·주기 유지.
- 좌표계 `frame_id = "map"`. goal 비활성 = 빈 `frame_id`의 `PoseStamped`. goal은 활성 동안 2 Hz.
- 목록형 토픽 QoS: `TRANSIENT_LOCAL`, depth 1. 영상: `qos_profile_sensor_data`.
- 상태 문자열: `" | "` 구분, 첫 칸 대문자 상태. `/mission/status`에는 **발행하지 않는다**.
- 몸통 높이: `velocity` 0.4 m(월드 go2 include z), `legged` 0.295 m(`mpc_controller/balance_node.py` `height`).
- 기본값: `margin_deg` 5.0, `arrive_tol` 0.4, `robot_radius` 0.35, `min_dial_px` 20(지름), `dwell_s` 5.0, `merge_radius` 1.0, `confirm_frames` 3, `forget_s` 3.0, `inspect_timeout` 60.0, `max_attempts` 2, `recheck_s` 0.0, `approach_dist` 1.5, `odom_timeout` 1.0.
- 코드 스타일: 기존 `gauge_ocr`처럼 한국어 docstring·짧은 줄 끝 주석, `from __future__` 없이 Python 3.10+ 문법.

### 테스트 실행 방법 (모든 Task 공통)

ROS 없이 Mac에서 돈다. 저장소 루트의 `module2_inspection/`에서:

```bash
V=/private/tmp/claude-501/-Users-leeunhak-Desktop-----32-dt-doosan-quadruped/d72e0094-27a5-4e6e-9d75-fcc2a40fa280/scratchpad/venv/bin/python
# venv가 없으면: python3 -m venv <위 venv 경로> && <venv>/bin/python -m pip install numpy opencv-python-headless pytest pyyaml
cd /Users/leeunhak/Desktop/동미대/32-dt-doosan-quadruped/module2_inspection
PYTHONPATH=inspection_common $V -m pytest inspection_common/test -q
PYTHONPATH=inspection_common:gauge_ocr $V -m pytest gauge_ocr/test -q
PYTHONPATH=inspection_common:thermal_fusion $V -m pytest thermal_fusion/test -q
```

패키지마다 따로 돌린다 (테스트 파일 이름이 겹치지 않게 했지만 `test/`에 `__init__.py`가 없으므로).

## Review Focus

1. **게이지 앞 공간이 좁은 맵** (corridor + legged): 위치를 억지로 내지 말고 `no_space:<장애물>` 사유와 함께 불가로 나와야 한다 → Task 4 `test_corridor_legged_is_no_space`, Task 5 `test_main_exit_code_when_impossible`.
2. **햇빛 받은 바닥이 가장 밝은 brightness 모드**: 바닥 픽셀이 과열 지점으로 등록되면 안 된다 → Task 8 `test_pixel_below_horizon_hits_floor_first`.
3. **확정된 지점을 다시 보는 경우**: 순찰 중 같은 탱크를 다시 보면 새 지점·새 goal이 생기면 안 된다 → Task 9 `test_decided_spot_is_not_recreated`.
4. **`/odom`이 끊긴 경우**: 로봇 위치를 모르는데 goal을 계속 내면 안 된다 → Task 10 `test_active_goal_none_when_odom_stale`.
5. **너무 작은 원판**: 틀린 값을 내지 말고 판독 실패(NaN)여야 한다 → Task 5 `test_dial_12px_is_not_read`.

---

### Task 1: `inspection_common` 패키지 뼈대 + 상태 문자열

**Files:**
- Create: `module2_inspection/inspection_common/package.xml`
- Create: `module2_inspection/inspection_common/setup.py`
- Create: `module2_inspection/inspection_common/setup.cfg`
- Create: `module2_inspection/inspection_common/resource/inspection_common` (빈 파일)
- Create: `module2_inspection/inspection_common/inspection_common/__init__.py` (빈 파일)
- Create: `module2_inspection/inspection_common/inspection_common/state_msg.py`
- Test: `module2_inspection/inspection_common/test/test_state_msg.py`

**Interfaces:**
- Produces: `format_state(state: str, *fields: object) -> str`, `parse_state(text: str) -> tuple[str, list[str]]`, 상수 `SEP = " | "`

- [ ] **Step 1: 패키지 파일 만들기**

`package.xml`:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>inspection_common</name>
  <version>0.1.0</version>
  <description>module2 점검 공용: 맵(SDF) 읽기, 기하, 점검 위치, 상태 문자열</description>
  <maintainer email="team@doosan.local">운학</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>ament_index_python</exec_depend>

  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

`setup.py`:
```python
from setuptools import find_packages, setup

package_name = "inspection_common"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="운학",
    maintainer_email="team@doosan.local",
    description="module2 점검 공용: 맵(SDF) 읽기, 기하, 점검 위치, 상태 문자열",
    license="Apache-2.0",
    tests_require=["pytest"],
)
```

`setup.cfg`:
```
[develop]
script_dir=$base/lib/inspection_common
[install]
install_scripts=$base/lib/inspection_common
```

`resource/inspection_common`, `inspection_common/__init__.py`: 빈 파일.

- [ ] **Step 2: 실패하는 테스트 작성** — `test/test_state_msg.py`

```python
"""상태 문자열 만들기·읽기."""

import pytest

from inspection_common.state_msg import format_state, parse_state


def test_format_joins_with_separator():
    assert format_state("READING", "gauge_line1", "4.23 bar") == "READING | gauge_line1 | 4.23 bar"


def test_state_only():
    assert format_state("IDLE") == "IDLE"


def test_roundtrip():
    text = format_state("PENDING", "hotspot_1 (7.65, -6.00)", "85.2 C", "goal (6.1, -6.0)")
    assert parse_state(text) == ("PENDING", ["hotspot_1 (7.65, -6.00)", "85.2 C", "goal (6.1, -6.0)"])


def test_parse_strips_spaces():
    assert parse_state("NO_GAUGE |unknown ") == ("NO_GAUGE", ["unknown"])


@pytest.mark.parametrize("bad", ["reading", "", "NO GAUGE", "A|B"])
def test_state_must_be_upper_single_word(bad):
    with pytest.raises(ValueError):
        format_state(bad)


def test_field_cannot_contain_bar():
    with pytest.raises(ValueError):
        format_state("READING", "a|b")
```

- [ ] **Step 3: 실패 확인**

Run: `PYTHONPATH=inspection_common $V -m pytest inspection_common/test/test_state_msg.py -q`
Expected: FAIL, `ModuleNotFoundError: No module named 'inspection_common.state_msg'`

- [ ] **Step 4: 구현** — `inspection_common/state_msg.py`

```python
"""노드 상태 문자열. 채현 님 /mission/status 형식("상태 | 이유 | 값")과 같은 구분자를 쓴다."""

SEP = " | "


def format_state(state: str, *fields: object) -> str:
    """첫 칸은 대문자 한 단어 상태, 나머지는 자유 문자열. '|'는 칸 안에 못 넣는다."""
    if not state or state != state.upper() or " " in state or "|" in state:
        raise ValueError(f"상태는 대문자 한 단어: {state!r}")
    texts = [str(f) for f in fields]
    for t in texts:
        if "|" in t:
            raise ValueError(f"칸에 '|'를 넣을 수 없음: {t!r}")
    return SEP.join([state, *texts])


def parse_state(text: str) -> tuple[str, list[str]]:
    parts = [p.strip() for p in text.split("|")]
    return parts[0], parts[1:]
```

- [ ] **Step 5: 통과 확인**

Run: `PYTHONPATH=inspection_common $V -m pytest inspection_common/test/test_state_msg.py -q`
Expected: 9 passed

- [ ] **Step 6: 기록 (커밋 안 함)** — 위 Files 목록을 작업 기록에 남긴다.

---

### Task 2: `world_map` — SDF에서 게이지·장애물·카메라 읽기

**Files:**
- Create: `module2_inspection/inspection_common/inspection_common/world_map.py`
- Test: `module2_inspection/inspection_common/test/test_world_map.py`

**Interfaces:**
- Produces:
  - `CameraSpec(x, y, z, pitch: float, hfov: float, width: int, height: int)` + 속성 `fx`, `half_vfov`
  - `Obstacle(name: str, kind: "box"|"cylinder", x, y, yaw, sx, sy, r, z_min, z_max)`; 이름은 `"<모델이름>/<링크이름>"`
  - `Gauge(name, x, y, z, yaw, dial_r, face)`; `face` = 모델 원점에서 원판 앞면까지(+x 방향)
  - `WorldMap(gauges: list[Gauge], obstacles: list[Obstacle], camera: CameraSpec, thermal: CameraSpec)`
  - `load_world(world_file: str, models_dir: str) -> WorldMap`
  - `resolve_world(world: str, world_file: str = "", models_dir: str = "", sim_share: str = "") -> tuple[str, str]`
  - `body_height_for(locomotion: str) -> float`
  - 상수 `VELOCITY_BODY_HEIGHT = 0.4`, `LEGGED_BODY_HEIGHT = 0.295`, `FLOOR_TOP = 0.05`

- [ ] **Step 1: 실패하는 테스트 작성** — `test/test_world_map.py`

```python
"""저장소의 실제 시뮬 맵을 읽는다."""

import math
from pathlib import Path

import pytest

from inspection_common.world_map import (
    body_height_for, load_world, resolve_world,
)

SIM = Path(__file__).resolve().parents[3] / "simulation"


def _load(world):
    return load_world(*resolve_world(world, sim_share=str(SIM)))


def test_factory_gauges():
    wm = _load("factory")
    names = [g.name for g in wm.gauges]
    assert names == ["gauge_line1", "gauge_line2"]
    g = wm.gauges[0]
    assert (g.x, g.y, g.z) == pytest.approx((10.0, 8.7, 1.5))
    assert g.yaw == pytest.approx(-1.5708)
    assert g.dial_r == pytest.approx(0.12)
    assert g.face == pytest.approx(0.04)


def test_corridor_gauge():
    wm = _load("corridor")
    assert [(g.name, g.z) for g in wm.gauges] == [("gauge_corridor", 1.4)]


def test_factory_obstacles_without_floor_and_robot():
    wm = _load("factory")
    models = sorted({o.name.split("/")[0] for o in wm.obstacles})
    assert models == sorted([
        "wall_north", "wall_south", "wall_west", "wall_east",
        "rack_a", "rack_b", "gauge_line1", "gauge_line2", "gas_tank_1", "gas_tank_2",
    ])


def test_gas_tank_is_cylinder():
    tank = next(o for o in _load("factory").obstacles if o.name.startswith("gas_tank_1/"))
    assert tank.kind == "cylinder"
    assert (tank.x, tank.y, tank.r) == pytest.approx((8.0, -6.0, 0.35))
    assert (tank.z_min, tank.z_max) == pytest.approx((0.0, 1.5))


def test_rotated_rack_box():
    rack = next(o for o in _load("factory").obstacles if o.name.startswith("rack_a/"))
    assert rack.kind == "box"
    assert (rack.x, rack.y, rack.yaw, rack.sx, rack.sy) == pytest.approx((6, 4, 0.3, 0.6, 0.4))
    assert (rack.z_min, rack.z_max) == pytest.approx((0.0, 0.8))


def test_camera_specs():
    wm = _load("factory")
    cam, th = wm.camera, wm.thermal
    assert (cam.x, cam.y, cam.z, cam.pitch) == pytest.approx((0.28, 0.0, 0.05, 0.0))
    assert (cam.hfov, cam.width, cam.height) == (pytest.approx(1.396), 640, 480)
    assert cam.fx == pytest.approx(381.46, abs=0.01)
    assert math.degrees(cam.half_vfov) == pytest.approx(32.18, abs=0.01)
    assert (th.z, th.hfov, th.width, th.height) == (pytest.approx(0.02), pytest.approx(1.0), 320, 240)


def test_resolve_missing_world_raises():
    with pytest.raises(FileNotFoundError):
        resolve_world("no_such_world", sim_share=str(SIM))


def test_missing_model_dir_raises(tmp_path):
    world = tmp_path / "w.sdf"
    world.write_text(
        '<sdf version="1.10"><world name="w"><include><uri>model://missing</uri>'
        "<name>m</name></include></world></sdf>"
    )
    with pytest.raises(FileNotFoundError, match="missing"):
        load_world(str(world), str(SIM / "models"))


def test_world_without_gauges(tmp_path):
    world = tmp_path / "w.sdf"
    world.write_text('<sdf version="1.10"><world name="w"></world></sdf>')
    wm = load_world(str(world), str(SIM / "models"))
    assert wm.gauges == [] and wm.obstacles == []


def test_body_height_for():
    assert body_height_for("velocity") == 0.4
    assert body_height_for("legged") == 0.295
    with pytest.raises(ValueError):
        body_height_for("wheels")
```

- [ ] **Step 2: 실패 확인**

Run: `PYTHONPATH=inspection_common $V -m pytest inspection_common/test/test_world_map.py -q`
Expected: FAIL, `ModuleNotFoundError: No module named 'inspection_common.world_map'`

- [ ] **Step 3: 구현** — `inspection_common/world_map.py`

```python
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
```

- [ ] **Step 4: 통과 확인**

Run: `PYTHONPATH=inspection_common $V -m pytest inspection_common/test/test_world_map.py -q`
Expected: 10 passed

- [ ] **Step 5: 기록 (커밋 안 함)**

---

### Task 3: `geometry` — 충돌, 광선, 카메라 각도

**Files:**
- Create: `module2_inspection/inspection_common/inspection_common/geometry.py`
- Test: `module2_inspection/inspection_common/test/test_geometry.py`

**Interfaces:**
- Consumes: `Obstacle`, `CameraSpec`, `load_world`, `resolve_world` (Task 2)
- Produces:
  - `wrap(a: float) -> float` (−π~π)
  - `point_distance(o: Obstacle, x, y) -> float` (2D, 안이면 0)
  - `circle_collision(x, y, radius, obstacles) -> str | None` (장애물 이름)
  - `ray_hits(x, y, angle, obstacles, max_range=50.0) -> list[tuple[float, Obstacle]]` (가까운 순, 수평 거리)
  - `segment_blocked(x0, y0, x1, y1, obstacles, z_from=-inf) -> str | None`
  - `camera_xyz(rx, ry, ryaw, cam, body_height) -> tuple[float, float, float]`
  - `pixel_angles(u, v, cam) -> tuple[float, float]` (수평각 왼쪽 +, 올려본각 위 +)

- [ ] **Step 1: 실패하는 테스트 작성** — `test/test_geometry.py`

```python
"""2D 기하 함수."""

import math
from pathlib import Path

import pytest

from inspection_common.geometry import (
    camera_xyz, circle_collision, pixel_angles, point_distance, ray_hits, segment_blocked, wrap,
)
from inspection_common.world_map import CameraSpec, Obstacle, load_world, resolve_world

SIM = Path(__file__).resolve().parents[3] / "simulation"
CAM = CameraSpec(0.28, 0.0, 0.05, 0.0, 1.396, 640, 480)
TANK = Obstacle("tank/link", "cylinder", 2.0, 0.0, 0.0, 0.0, 0.0, 0.35, 0.0, 1.5)
# 가로 2 m × 세로 0.2 m 판을 90° 돌림 → x ∈ [2.9, 3.1], y ∈ [-1, 1]
PLATE = Obstacle("plate/link", "box", 3.0, 0.0, math.pi / 2, 2.0, 0.2, 0.0, 0.0, 1.0)
LOW = Obstacle("low/link", "box", 1.0, 0.0, 0.0, 0.2, 0.2, 0.0, 0.0, 0.3)


def test_wrap():
    assert wrap(3 * math.pi / 2) == pytest.approx(-math.pi / 2)


def test_point_distance_rotated_box():
    assert point_distance(PLATE, 2.5, 0.0) == pytest.approx(0.4)
    assert point_distance(PLATE, 3.0, 0.5) == 0.0
    assert point_distance(PLATE, 3.0, 1.5) == pytest.approx(0.5)


def test_circle_collision():
    assert circle_collision(1.5, 0.0, 0.35, [TANK]) == "tank/link"   # 표면까지 0.15 m
    assert circle_collision(1.0, 0.0, 0.35, [TANK]) is None          # 표면까지 0.65 m


def test_ray_hits_sorted():
    hits = ray_hits(0.0, 0.0, 0.0, [PLATE, TANK])
    assert [o.name for _, o in hits] == ["tank/link", "plate/link"]
    assert hits[0][0] == pytest.approx(1.65)
    assert hits[1][0] == pytest.approx(2.9)


def test_ray_miss_and_range():
    assert ray_hits(0.0, 0.0, math.pi / 2, [PLATE, TANK]) == []
    assert ray_hits(0.0, 0.0, 0.0, [PLATE], max_range=2.0) == []


def test_ray_from_inside_is_zero():
    assert ray_hits(2.0, 0.0, 0.0, [TANK])[0][0] == 0.0


def test_segment_blocked_ignores_low_obstacle():
    assert segment_blocked(0.0, 0.0, 2.5, 0.0, [LOW]) == "low/link"
    assert segment_blocked(0.0, 0.0, 2.5, 0.0, [LOW], z_from=0.45) is None


def test_camera_xyz():
    x, y, z = camera_xyz(1.0, 2.0, math.pi / 2, CAM, 0.4)
    assert (x, y, z) == pytest.approx((1.0, 2.28, 0.45))


def test_pixel_angles():
    assert pixel_angles(320, 240, CAM) == pytest.approx((0.0, 0.0))
    az, _ = pixel_angles(0, 240, CAM)
    assert az == pytest.approx(1.396 / 2)                         # 왼쪽 끝 = +hfov/2
    _, el = pixel_angles(320, 0, CAM)
    assert el == pytest.approx(CAM.half_vfov)


def test_ray_hits_real_gas_tank():
    wm = load_world(*resolve_world("factory", sim_share=str(SIM)))
    d, o = ray_hits(8.0, -4.5, -math.pi / 2, wm.obstacles)[0]
    assert o.name.startswith("gas_tank_1/")
    assert d == pytest.approx(1.15)
```

- [ ] **Step 2: 실패 확인**

Run: `PYTHONPATH=inspection_common $V -m pytest inspection_common/test/test_geometry.py -q`
Expected: FAIL, `ModuleNotFoundError: No module named 'inspection_common.geometry'`

- [ ] **Step 3: 구현** — `inspection_common/geometry.py`

```python
"""2D 기하: 로봇 몸체 충돌, 시야 가림, 광선 쏘기, 픽셀 → 각도. 거리는 모두 수평(m)."""

import math

from inspection_common.world_map import CameraSpec, Obstacle


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _to_local(o: Obstacle, x: float, y: float) -> tuple:
    dx, dy = x - o.x, y - o.y
    c, s = math.cos(o.yaw), math.sin(o.yaw)
    return c * dx + s * dy, -s * dx + c * dy


def point_distance(o: Obstacle, x: float, y: float) -> float:
    """점에서 장애물 바닥 모양까지 거리. 안이면 0."""
    if o.kind == "cylinder":
        return max(0.0, math.hypot(x - o.x, y - o.y) - o.r)
    lx, ly = _to_local(o, x, y)
    return math.hypot(max(abs(lx) - o.sx / 2, 0.0), max(abs(ly) - o.sy / 2, 0.0))


def circle_collision(x: float, y: float, radius: float, obstacles) -> str | None:
    for o in obstacles:
        if point_distance(o, x, y) < radius:
            return o.name
    return None


def _ray_enter(o: Obstacle, x: float, y: float, dx: float, dy: float) -> float | None:
    """반직선이 장애물에 처음 들어가는 거리. 시작점이 안이면 0."""
    if o.kind == "cylinder":
        fx, fy = x - o.x, y - o.y
        b = fx * dx + fy * dy
        disc = b * b - (fx * fx + fy * fy - o.r * o.r)
        if disc < 0:
            return None
        t0, t1 = -b - math.sqrt(disc), -b + math.sqrt(disc)
        return None if t1 < 0 else max(t0, 0.0)
    lx, ly = _to_local(o, x, y)
    c, s = math.cos(o.yaw), math.sin(o.yaw)
    ldx, ldy = c * dx + s * dy, -s * dx + c * dy
    t_near, t_far = -math.inf, math.inf
    for p, d, half in ((lx, ldx, o.sx / 2), (ly, ldy, o.sy / 2)):          # slab 방식
        if abs(d) < 1e-12:
            if abs(p) > half:
                return None
            continue
        ta, tb = (-half - p) / d, (half - p) / d
        t_near, t_far = max(t_near, min(ta, tb)), min(t_far, max(ta, tb))
    if t_near > t_far or t_far < 0:
        return None
    return max(t_near, 0.0)


def ray_hits(x: float, y: float, angle: float, obstacles, max_range: float = 50.0) -> list:
    dx, dy = math.cos(angle), math.sin(angle)
    hits = []
    for o in obstacles:
        t = _ray_enter(o, x, y, dx, dy)
        if t is not None and t <= max_range:
            hits.append((t, o))
    hits.sort(key=lambda h: h[0])
    return hits


def segment_blocked(x0: float, y0: float, x1: float, y1: float, obstacles,
                    z_from: float = -math.inf) -> str | None:
    """두 점 사이를 가리는 장애물 이름. 윗면이 z_from 이하인 낮은 장애물은 무시."""
    length = math.hypot(x1 - x0, y1 - y0)
    if length == 0:
        return None
    for _, o in ray_hits(x0, y0, math.atan2(y1 - y0, x1 - x0), obstacles, length):
        if o.z_max > z_from:
            return o.name
    return None


def camera_xyz(rx: float, ry: float, ryaw: float, cam: CameraSpec, body_height: float) -> tuple:
    c, s = math.cos(ryaw), math.sin(ryaw)
    return rx + c * cam.x - s * cam.y, ry + s * cam.x + c * cam.y, body_height + cam.z


def pixel_angles(u: float, v: float, cam: CameraSpec) -> tuple:
    """픽셀 → (수평각 왼쪽 +, 올려본각 위 +) rad. 주점 = 이미지 중앙."""
    du, dv = cam.width / 2 - u, cam.height / 2 - v
    return math.atan2(du, cam.fx), math.atan2(dv, math.hypot(cam.fx, du)) - cam.pitch
```

- [ ] **Step 4: 통과 확인**

Run: `PYTHONPATH=inspection_common $V -m pytest inspection_common/test/test_geometry.py -q`
Expected: 10 passed

- [ ] **Step 5: 기록 (커밋 안 함)**

---

### Task 4: `viewpoint` — 게이지 점검 위치, 과열 지점 접근 위치

**Files:**
- Create: `module2_inspection/inspection_common/inspection_common/viewpoint.py`
- Test: `module2_inspection/inspection_common/test/test_viewpoint.py`

**Interfaces:**
- Consumes: `Gauge`, `CameraSpec`, `Obstacle`, `load_world`, `resolve_world`, `body_height_for` (Task 2); `circle_collision`, `segment_blocked`, `camera_xyz`, `wrap` (Task 3)
- Produces:
  - `Viewpoint(name, x, y, yaw, distance, dial_px, dial_px_far, ok: bool, reason: str)`; `reason`은 `""` / `"no_space:<장애물>"` / `"blocked:<장애물>"` / `"too_small"`
  - `gauge_viewpoint(g, cam, obstacles, body_height, margin_deg=5.0, arrive_tol=0.4, robot_radius=0.35, min_dial_px=20.0) -> Viewpoint`
  - `approach_point(tx, ty, from_x, from_y, obstacles, dist=1.5, robot_radius=0.35) -> tuple[tuple[float, float, float] | None, str]`

- [ ] **Step 1: 실패하는 테스트 작성** — `test/test_viewpoint.py`

```python
"""점검 위치 계산. 숫자는 설계 문서 4-2절 표."""

import math
from pathlib import Path

import pytest

from inspection_common.viewpoint import approach_point, gauge_viewpoint
from inspection_common.world_map import Obstacle, body_height_for, load_world, resolve_world

SIM = Path(__file__).resolve().parents[3] / "simulation"


def _vp(world, locomotion, index=0, **kw):
    wm = load_world(*resolve_world(world, sim_share=str(SIM)))
    return gauge_viewpoint(wm.gauges[index], wm.camera, wm.obstacles, body_height_for(locomotion), **kw)


def test_factory_velocity():
    v = _vp("factory", "velocity")
    assert v.ok and v.reason == ""
    assert v.distance == pytest.approx(2.959, abs=0.005)
    assert (v.x, v.y) == pytest.approx((10.0, 8.7 - 0.04 - 2.959), abs=0.005)
    assert v.yaw == pytest.approx(math.pi / 2)
    assert v.dial_px == pytest.approx(31.8, abs=0.2)
    assert v.dial_px_far == pytest.approx(28.1, abs=0.2)


def test_factory_legged_is_farther():
    v = _vp("factory", "legged")
    assert v.ok
    assert v.distance == pytest.approx(3.163, abs=0.005)


def test_factory_second_gauge():
    v = _vp("factory", "velocity", index=1)
    assert v.ok and (v.x, v.y) == pytest.approx((-8.0, 5.70), abs=0.01)


def test_corridor_velocity_fits():
    v = _vp("corridor", "velocity")
    assert v.ok
    assert v.distance == pytest.approx(2.764, abs=0.005)


def test_corridor_legged_is_no_space():
    v = _vp("corridor", "legged")
    assert not v.ok
    assert v.reason.startswith("no_space:wall_south")


def test_too_small_when_threshold_high():
    v = _vp("factory", "velocity", min_dial_px=40)
    assert not v.ok and v.reason == "too_small"


def test_blocked_by_obstacle_in_between():
    wm = load_world(*resolve_world("factory", sim_share=str(SIM)))
    wall = Obstacle("screen/link", "box", 10.0, 7.5, 0.0, 1.0, 0.1, 0.0, 0.0, 2.0)
    v = gauge_viewpoint(wm.gauges[0], wm.camera, wm.obstacles + [wall], 0.4)
    assert not v.ok and v.reason == "blocked:screen/link"


TANK = Obstacle("tank/link", "cylinder", 8.0, -6.0, 0.0, 0.0, 0.0, 0.35, 0.0, 1.5)


def test_approach_straight():
    goal, reason = approach_point(7.65, -6.0, 5.0, -6.0, [TANK])
    assert reason == ""
    assert goal == pytest.approx((6.15, -6.0, 0.0))


def test_approach_rotates_when_blocked():
    rack = Obstacle("rack/link", "box", 6.15, -6.0, 0.0, 0.4, 0.4, 0.0, 0.0, 0.8)
    goal, reason = approach_point(7.65, -6.0, 5.0, -6.0, [TANK, rack])
    assert reason == ""
    x, y, yaw = goal
    assert math.hypot(x - 7.65, y + 6.0) == pytest.approx(1.5)
    assert abs(math.degrees(math.atan2(y + 6.0, x - 7.65))) == pytest.approx(140.0)   # ±20°는 rack에 걸리고 ±40°에서 성공
    assert math.hypot(math.cos(yaw) - (7.65 - x) / 1.5, math.sin(yaw) - (-6.0 - y) / 1.5) < 1e-6


def test_approach_impossible():
    box = Obstacle("cage/link", "box", 7.65, -6.0, 0.0, 6.0, 6.0, 0.0, 0.0, 2.0)
    goal, reason = approach_point(7.65, -6.0, 5.0, -6.0, [box])
    assert goal is None and reason.startswith("no_space:cage")
```

- [ ] **Step 2: 실패 확인**

Run: `PYTHONPATH=inspection_common $V -m pytest inspection_common/test/test_viewpoint.py -q`
Expected: FAIL, `ModuleNotFoundError: No module named 'inspection_common.viewpoint'`

- [ ] **Step 3: 구현** — `inspection_common/viewpoint.py`

```python
"""점검 위치 계산: 게이지를 화면에 담는 정차 위치, 과열 지점에 다가갈 위치."""

from dataclasses import dataclass
import math

from inspection_common.geometry import camera_xyz, circle_collision, segment_blocked, wrap
from inspection_common.world_map import CameraSpec, Gauge

APPROACH_OFFSETS_DEG = (0, 20, -20, 40, -40, 60, -60)


@dataclass(frozen=True)
class Viewpoint:
    name: str
    x: float
    y: float
    yaw: float
    distance: float             # 원판 앞면 → base_link 수평 거리
    dial_px: float              # 정위치에서 원판 지름(px)
    dial_px_far: float          # arrive_tol만큼 멀리 섰을 때
    ok: bool
    reason: str                 # "" | no_space:<장애물> | blocked:<장애물> | too_small


def gauge_viewpoint(g: Gauge, cam: CameraSpec, obstacles, body_height: float,
                    margin_deg: float = 5.0, arrive_tol: float = 0.4,
                    robot_radius: float = 0.35, min_dial_px: float = 20.0) -> Viewpoint:
    """원판 윗끝이 화면 위 가장자리(여유각 margin) 안에 드는 가장 가까운 정면 위치.

    arrive_tol만큼 더 가까이 서도 화면에 들어오도록 그만큼 물러선다.
    """
    cz = body_height + cam.z
    limit = cam.half_vfov - math.radians(margin_deg) - cam.pitch
    if limit <= 0:
        raise ValueError("여유각이 반화각보다 크다")
    cam_d = max(g.z + g.dial_r - cz, 0.0) / math.tan(limit) + arrive_tol
    dist = cam_d + cam.x
    ux, uy = math.cos(g.yaw), math.sin(g.yaw)
    face_x, face_y = g.x + ux * g.face, g.y + uy * g.face
    x, y = face_x + ux * dist, face_y + uy * dist
    yaw = wrap(g.yaw + math.pi)

    def px(d: float) -> float:
        return 2 * cam.fx * g.dial_r / math.hypot(d, g.z - cz)

    near, far = px(cam_d), px(cam_d + arrive_tol)
    others = [o for o in obstacles if not o.name.startswith(g.name + "/")]
    reason = ""
    hit = circle_collision(x, y, robot_radius, obstacles)
    if hit:
        reason = f"no_space:{hit}"
    else:
        cx, cy, _ = camera_xyz(x, y, yaw, cam, body_height)
        blk = segment_blocked(cx, cy, face_x, face_y, others, z_from=cz)
        if blk:
            reason = f"blocked:{blk}"
        elif far < min_dial_px:
            reason = "too_small"
    return Viewpoint(g.name, x, y, yaw, dist, near, far, reason == "", reason)


def approach_point(tx: float, ty: float, from_x: float, from_y: float, obstacles,
                   dist: float = 1.5, robot_radius: float = 0.35) -> tuple:
    """과열 지점 (tx, ty)를 처음 본 쪽에서 dist 떨어진 위치와 정면 yaw.

    막히면 ±20°, ±40°, ±60° 돌려 본다. 모두 안 되면 (None, 마지막 사유).
    """
    base = math.atan2(from_y - ty, from_x - tx)
    last = "no_space"
    for off in APPROACH_OFFSETS_DEG:
        a = base + math.radians(off)
        x, y = tx + dist * math.cos(a), ty + dist * math.sin(a)
        hit = circle_collision(x, y, robot_radius, obstacles)
        if hit:
            last = f"no_space:{hit}"
            continue
        ex, ey = tx + 0.05 * math.cos(a), ty + 0.05 * math.sin(a)   # 지점은 표면 위라 살짝 앞까지만 본다
        blk = segment_blocked(x, y, ex, ey, obstacles)
        if blk:
            last = f"blocked:{blk}"
            continue
        return (x, y, wrap(a + math.pi)), ""
    return None, last
```

- [ ] **Step 4: 통과 확인**

Run: `PYTHONPATH=inspection_common $V -m pytest inspection_common/test -q`
Expected: state_msg 9 + world_map 10 + geometry 10 + viewpoint 10 = 39 passed

- [ ] **Step 5: 기록 (커밋 안 함)**

---

### Task 5: 게이지 — `make_viewpoints` 도구, 이름 추정, 작은 원판 한계 테스트

**Files:**
- Create: `module2_inspection/gauge_ocr/gauge_ocr/make_viewpoints.py`
- Create: `module2_inspection/gauge_ocr/gauge_ocr/naming.py`
- Modify: `module2_inspection/gauge_ocr/setup.py` (console_scripts에 `make_viewpoints` 추가)
- Modify: `module2_inspection/gauge_ocr/package.xml` (`inspection_common`, `geometry_msgs` exec_depend 추가 — Task 6 노드도 씀)
- Test: `module2_inspection/gauge_ocr/test/test_make_viewpoints.py`
- Test: `module2_inspection/gauge_ocr/test/test_naming.py`
- Test: `module2_inspection/gauge_ocr/test/test_reader_small.py`

**Interfaces:**
- Consumes: `load_world`, `resolve_world`, `body_height_for`, `WorldMap` (Task 2); `gauge_viewpoint`, `Viewpoint` (Task 4)
- Produces:
  - `compute_viewpoints(wm: WorldMap, body_height: float, **kw) -> list[Viewpoint]`
  - `to_yaml(vps: list[Viewpoint], dwell_s: float) -> str`
  - `main(argv: list[str] | None = None) -> int` (모두 가능 0, 하나라도 불가 1)
  - `current_gauge_name(status: str, robot_xy: tuple | None, viewpoints: list[Viewpoint], radius: float = 1.0) -> str`

- [ ] **Step 1: 실패하는 테스트 작성**

`test/test_make_viewpoints.py`:
```python
"""make_viewpoints 도구. 출력은 순찰 waypoints YAML에 그대로 붙일 수 있어야 한다."""

from pathlib import Path

import yaml

from gauge_ocr.make_viewpoints import compute_viewpoints, main, to_yaml
from inspection_common.viewpoint import Viewpoint
from inspection_common.world_map import load_world, resolve_world

SIM = str(Path(__file__).resolve().parents[3] / "simulation")


def test_compute_factory():
    wm = load_world(*resolve_world("factory", sim_share=SIM))
    vps = compute_viewpoints(wm, 0.4)
    assert [v.name for v in vps] == ["gauge_line1", "gauge_line2"]
    assert all(v.ok for v in vps)


def test_yaml_is_patrol_format():
    vps = [
        Viewpoint("g1", 10.0, 5.701, 1.5708, 2.96, 31.8, 28.1, True, ""),
        Viewpoint("g2", 0.0, 0.0, 0.0, 2.97, 0.0, 0.0, False, "no_space:wall_south/link"),
    ]
    text = to_yaml(vps, 5.0)
    assert "# g2: 불가 (no_space:wall_south/link)" in text
    data = yaml.safe_load("waypoints:\n" + text)
    assert data["waypoints"] == [{"name": "g1", "x": 10.0, "y": 5.7, "yaw": 1.57, "dwell_s": 5.0}]


def test_main_factory(capsys):
    assert main(["--world", "factory", "--sim-share", SIM]) == 0
    out = capsys.readouterr().out
    assert "{name: gauge_line1, x: 10.00, y: 5.70, yaw: 1.57, dwell_s: 5.0}" in out


def test_main_exit_code_when_impossible(capsys):
    assert main(["--world", "corridor", "--locomotion", "legged", "--sim-share", SIM]) == 1
    assert "gauge_corridor: 불가 (no_space:wall_south" in capsys.readouterr().out
```

`test/test_naming.py`:
```python
"""판독 중인 게이지 이름 추정."""

from gauge_ocr.naming import current_gauge_name
from inspection_common.viewpoint import Viewpoint

VPS = [
    Viewpoint("gauge_line1", 10.0, 5.7, 1.57, 2.96, 31.8, 28.1, True, ""),
    Viewpoint("gauge_bad", 0.0, 0.0, 0.0, 2.97, 0.0, 0.0, False, "no_space:x"),
]


def test_dwell_status_wins():
    assert current_gauge_name("dwell:gauge_line2", (10.0, 5.7), VPS) == "gauge_line2"


def test_nearest_viewpoint_within_radius():
    assert current_gauge_name("goto:home", (10.5, 5.2), VPS) == "gauge_line1"


def test_impossible_viewpoint_is_ignored():
    assert current_gauge_name("", (0.1, 0.0), VPS) == "unknown"


def test_unknown_when_far_or_no_pose():
    assert current_gauge_name("", (0.0, 3.0), VPS) == "unknown"
    assert current_gauge_name("", None, VPS) == "unknown"
```

`test/test_reader_small.py` (특성 테스트 — 현재 reader로 바로 통과해야 정상. 한계를 고정해 두는 용도):
```python
"""작은 원판 판독 한계. 점검 위치에서 원판 지름은 약 26~34 px (설계 4-2절)."""

import math

import pytest

from gauge_ocr.reader import read_gauge
from test_reader import phi_of, SPEC, synth_gauge


@pytest.mark.parametrize("diam", [16, 20, 24, 32])
@pytest.mark.parametrize("value", [0.5, 2.5, 5.0, 7.5, 9.5])
@pytest.mark.parametrize("squash", [1.0, 0.9])
def test_small_dial_reads(diam, value, squash):
    v, _ = read_gauge(synth_gauge(phi_of(value), squash=squash, r=diam // 2), SPEC)
    assert abs(v - value) < 0.2


@pytest.mark.parametrize("value", [0.5, 5.0, 9.5])
def test_dial_12px_is_not_read(value):
    v, _ = read_gauge(synth_gauge(phi_of(value), r=6), SPEC)
    assert math.isnan(v)
```

- [ ] **Step 2: 실패 확인**

Run: `PYTHONPATH=inspection_common:gauge_ocr $V -m pytest gauge_ocr/test -q`
Expected: `test_make_viewpoints.py`, `test_naming.py`는 `ModuleNotFoundError`로 FAIL. `test_reader_small.py`(43개)와 기존 `test_reader.py`(29개)는 PASS.

- [ ] **Step 3: 구현**

`gauge_ocr/make_viewpoints.py`:
```python
"""게이지 점검 위치를 계산해 순찰 waypoints YAML 형식으로 출력한다.

    ros2 run gauge_ocr make_viewpoints --world factory
    python -m gauge_ocr.make_viewpoints --world factory --sim-share <저장소>/simulation   # ROS 없이
"""

import argparse
import sys

from inspection_common.viewpoint import gauge_viewpoint
from inspection_common.world_map import body_height_for, load_world, resolve_world


def compute_viewpoints(wm, body_height: float, **kw) -> list:
    return [gauge_viewpoint(g, wm.camera, wm.obstacles, body_height, **kw) for g in wm.gauges]


def to_yaml(vps: list, dwell_s: float) -> str:
    lines = ["  # make_viewpoints 결과 (순찰 waypoints에 붙여 넣기 전 태우 님 검토)"]
    for v in vps:
        if v.ok:
            lines.append(
                f"  - {{name: {v.name}, x: {v.x:.2f}, y: {v.y:.2f}, yaw: {v.yaw:.2f}, dwell_s: {dwell_s:.1f}}}"
                f"  # 원판 {v.dial_px:.0f}~{v.dial_px_far:.0f} px"
            )
        else:
            lines.append(f"  # {v.name}: 불가 ({v.reason})")
    return "\n".join(lines) + "\n"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="게이지 점검 위치 계산")
    ap.add_argument("--world", default="factory")
    ap.add_argument("--sim-share", default="", help="simulation 폴더 (없으면 설치된 패키지)")
    ap.add_argument("--locomotion", default="velocity", choices=("velocity", "legged"))
    ap.add_argument("--margin-deg", type=float, default=5.0)
    ap.add_argument("--arrive-tol", type=float, default=0.4)
    ap.add_argument("--robot-radius", type=float, default=0.35)
    ap.add_argument("--min-dial-px", type=float, default=20.0)
    ap.add_argument("--dwell", type=float, default=5.0)
    a = ap.parse_args(argv)
    wm = load_world(*resolve_world(a.world, sim_share=a.sim_share))
    vps = compute_viewpoints(
        wm, body_height_for(a.locomotion), margin_deg=a.margin_deg, arrive_tol=a.arrive_tol,
        robot_radius=a.robot_radius, min_dial_px=a.min_dial_px,
    )
    sys.stdout.write(to_yaml(vps, a.dwell))
    return 0 if all(v.ok for v in vps) else 1


if __name__ == "__main__":
    sys.exit(main())
```

`gauge_ocr/naming.py`:
```python
"""지금 판독 중인 게이지 이름: 순찰 상태 → 가까운 점검 위치 → unknown."""

import math


def current_gauge_name(status: str, robot_xy, viewpoints, radius: float = 1.0) -> str:
    if status.startswith("dwell:"):
        return status[len("dwell:"):]
    if robot_xy is not None:
        near = [(math.hypot(v.x - robot_xy[0], v.y - robot_xy[1]), v.name) for v in viewpoints if v.ok]
        if near and min(near)[0] <= radius:
            return min(near)[1]
    return "unknown"
```

`setup.py` 의 `entry_points`를 다음으로 바꾼다:
```python
    entry_points={
        "console_scripts": [
            "gauge_ocr_node = gauge_ocr.gauge_ocr_node:main",
            "make_viewpoints = gauge_ocr.make_viewpoints:main",
        ],
    },
```

`package.xml`의 `<exec_depend>launch_ros</exec_depend>` 아래에 추가:
```xml
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>inspection_common</exec_depend>
```

- [ ] **Step 4: 통과 확인**

Run: `PYTHONPATH=inspection_common:gauge_ocr $V -m pytest gauge_ocr/test -q`
Expected: 29 + 4 + 4 + 43 = 80 passed

- [ ] **Step 5: 기록 (커밋 안 함)**

---

### Task 6: 게이지 노드 — 점검 위치 토픽, 상태 토픽, 보행 모드 인자

**Files:**
- Modify: `module2_inspection/gauge_ocr/gauge_ocr/gauge_ocr_node.py` (전체 교체)
- Modify: `module2_inspection/gauge_ocr/launch/gauge_ocr.launch.py` (`locomotion` 인자)
- Modify: `module2_inspection/gauge_ocr/config/gauge_factory.yaml`, `gauge_corridor.yaml` (점검 위치 파라미터)
- Modify: `module2_inspection/gauge_ocr/README.md` (토픽 표)

**Interfaces:**
- Consumes: `compute_viewpoints` (Task 5), `current_gauge_name` (Task 5), `format_state` (Task 1), `load_world`, `resolve_world`, `body_height_for` (Task 2), 기존 `read_gauge`, `GaugeSpec`, `draw_overlay`
- Produces (토픽): `/inspection/gauge_viewpoints` (`PoseArray`, latched), `/inspection/gauge_state` (`String`, 0.5 s). `/inspection/gauge`는 그대로.

ROS가 없어서 이 Task는 단위 테스트 대신 **구문 확인 + 순수 함수 테스트 재실행**으로 확인한다. 노드 동작 확인은 ROS 환경 준비 후.

- [ ] **Step 1: 노드 교체** — `gauge_ocr/gauge_ocr_node.py`

```python
"""카메라 영상에서 게이지를 찾아 값을 판독하고, 점검 위치·상태를 알린다."""

from collections import deque
import math

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String

from gauge_ocr.make_viewpoints import compute_viewpoints
from gauge_ocr.naming import current_gauge_name
from gauge_ocr.overlay import draw_overlay
from gauge_ocr.reader import GaugeSpec, read_gauge
from inspection_common.state_msg import format_state
from inspection_common.world_map import body_height_for, load_world, resolve_world

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class GaugeOcrNode(Node):
    def __init__(self) -> None:
        super().__init__("gauge_ocr")
        self.declare_parameter("world", "corridor")
        self.declare_parameter("world_file", "")          # 비우면 설치된 simulation 패키지에서 찾음
        self.declare_parameter("models_path", "")
        self.declare_parameter("locomotion", "velocity")  # velocity | legged → 몸통 높이
        self.declare_parameter("phi_min", 225.0)
        self.declare_parameter("sweep", 270.0)
        self.declare_parameter("v_min", 0.0)
        self.declare_parameter("v_max", 10.0)
        self.declare_parameter("window", 15)              # 중앙값 프레임 수
        self.declare_parameter("stop_speed", 0.05)        # m/s 미만이면 정차
        self.declare_parameter("require_stop", True)      # False면 항상 판독 (bag·단독 테스트용)
        self.declare_parameter("publish_debug", True)
        self.declare_parameter("margin_deg", 5.0)         # 점검 위치 계산
        self.declare_parameter("arrive_tol", 0.4)
        self.declare_parameter("robot_radius", 0.35)
        self.declare_parameter("min_dial_px", 20.0)

        self._bridge = CvBridge()
        self._buf = deque(maxlen=self.get_parameter("window").value)
        self._speed = 0.0
        self._xy = None
        self._status = ""
        self._last_conf = 0.0
        self._viewpoints = []
        self._no_map = False

        self.create_subscription(
            Image, "/camera/image", self.camera_image_cb, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_subscription(String, "/mission/status", self.status_cb, 10)
        self.pub_0 = self.create_publisher(Float32, "/inspection/gauge", 10)
        self.pub_debug = self.create_publisher(
            Image, "/inspection/gauge_debug", qos_profile_sensor_data
        )
        self.pub_vp = self.create_publisher(PoseArray, "/inspection/gauge_viewpoints", LATCHED)
        self.pub_state = self.create_publisher(String, "/inspection/gauge_state", 10)
        self._load_viewpoints()
        self.create_timer(0.5, self._tick)
        self.get_logger().info("gauge_ocr started (운학)")

    def _p(self, name: str):
        return self.get_parameter(name).value

    def _spec(self) -> GaugeSpec:
        return GaugeSpec(self._p("phi_min"), self._p("sweep"), self._p("v_min"), self._p("v_max"))

    def _load_viewpoints(self) -> None:
        """시뮬 맵에서 점검 위치를 계산해 한 번 발행(latched). 실패해도 판독은 계속."""
        try:
            wm = load_world(*resolve_world(self._p("world"), self._p("world_file"), self._p("models_path")))
        except Exception as e:  # noqa: BLE001 — 맵이 없어도 판독은 해야 한다
            self.get_logger().warn(f"맵을 못 읽어 점검 위치 기능을 끕니다: {e}")
            self._no_map = True
            return
        self._viewpoints = compute_viewpoints(
            wm, body_height_for(self._p("locomotion")), margin_deg=self._p("margin_deg"),
            arrive_tol=self._p("arrive_tol"), robot_radius=self._p("robot_radius"),
            min_dial_px=self._p("min_dial_px"),
        )
        msg = PoseArray()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        for v in self._viewpoints:
            if not v.ok:
                self.get_logger().warn(f"{v.name}: 점검 위치 불가 ({v.reason})")
                continue
            p = Pose()
            p.position.x, p.position.y = v.x, v.y
            p.orientation.z, p.orientation.w = math.sin(v.yaw / 2), math.cos(v.yaw / 2)
            msg.poses.append(p)
        self.pub_vp.publish(msg)

    def _stopped(self) -> bool:
        """정차 중에만 판독. 게이지가 화면에 없으면 read_gauge가 NaN을 내므로 위치는 따로 안 본다."""
        if not self._p("require_stop"):
            return True
        return self._speed < self._p("stop_speed")

    def odom_cb(self, msg: Odometry) -> None:
        v = msg.twist.twist.linear
        self._speed = math.hypot(v.x, v.y)
        p = msg.pose.pose.position
        self._xy = (p.x, p.y)

    def status_cb(self, msg: String) -> None:
        self._status = msg.data

    def camera_image_cb(self, msg: Image) -> None:
        if not self._stopped():
            self._buf.clear()                             # 이동하면 이전 게이지 값을 버린다
            return
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().warn(f"cv_bridge: {e}")
            return
        value, dbg = read_gauge(bgr, self._spec())
        self._last_conf = float(dbg.get("conf", 0.0))
        if not math.isnan(value):
            self._buf.append(value)
        if self._p("publish_debug") and self.pub_debug.get_subscription_count() > 0:
            out = self._bridge.cv2_to_imgmsg(draw_overlay(bgr, value, dbg), encoding="bgr8")
            out.header = msg.header
            self.pub_debug.publish(out)

    def _state_text(self) -> str:
        extra = ["no_map"] if self._no_map else []
        if not self._stopped():
            return format_state("IDLE", "moving", *extra)
        name = current_gauge_name(self._status, self._xy, self._viewpoints)
        if self._buf:
            value = float(np.median(self._buf))
            return format_state("READING", name, f"{value:.2f} bar", f"conf {self._last_conf:.2f}", *extra)
        return format_state("NO_GAUGE", name, *extra)

    def _tick(self) -> None:
        self.pub_state.publish(String(data=self._state_text()))
        if not self._buf:
            return                                        # 판독값 없음 → 발행 안 함 (대시보드는 마지막 값 유지)
        out = Float32()
        out.data = float(np.median(self._buf))
        self.pub_0.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GaugeOcrNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: 런치 교체** — `launch/gauge_ocr.launch.py`

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    world = LaunchConfiguration("world")
    params_file = PathJoinSubstitution(
        [FindPackageShare("gauge_ocr"), "config", ["gauge_", world, ".yaml"]]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="corridor"),
            DeclareLaunchArgument("locomotion", default_value="velocity"),  # full_system이 넘겨 줌
            Node(
                package="gauge_ocr",
                executable="gauge_ocr_node",
                name="gauge_ocr",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "world": world,
                        "locomotion": LaunchConfiguration("locomotion"),
                        "use_sim_time": True,
                    },
                ],
            ),
        ]
    )
```

- [ ] **Step 3: YAML에 점검 위치 파라미터 추가** — `config/gauge_factory.yaml`, `config/gauge_corridor.yaml` 둘 다 `publish_debug: true` 아래에:

```yaml
    # 점검 위치 계산 (/inspection/gauge_viewpoints, make_viewpoints와 같은 기본값)
    margin_deg: 5.0         # 화면 위 가장자리 여유각
    arrive_tol: 0.4         # 순찰 도착 오차만큼 더 가까이 서도 화면에 들어오게
    robot_radius: 0.35      # 장애물 검사용 몸체 반경
    min_dial_px: 20.0       # 멀리 섰을 때 원판 지름이 이보다 작으면 불가
```

- [ ] **Step 4: README 토픽 표 갱신** — `gauge_ocr/README.md`의 토픽 표에 두 줄 추가, 구독 줄 추가:

```markdown
| `/inspection/gauge_viewpoints` | 발행 (PoseArray, latched) | 게이지 점검 위치(map). 불가한 게이지는 빠지고 로그에 사유 |
| `/inspection/gauge_state` | 발행 (String, 0.5 s) | `IDLE \| moving`, `READING \| <게이지> \| <값> bar \| conf <c>`, `NO_GAUGE \| <게이지>` |
| `/mission/status` | 구독 (String) | `dwell:<이름>`에서 게이지 이름을 얻음 (읽기만) |
```

그리고 README 끝에 도구 사용법:
````markdown
## 점검 위치 계산

```bash
ros2 run gauge_ocr make_viewpoints --world factory [--locomotion legged]
# ROS 없이: PYTHONPATH=module2_inspection/inspection_common:module2_inspection/gauge_ocr \
#   python3 -m gauge_ocr.make_viewpoints --world factory --sim-share simulation
```
출력은 순찰 `waypoints_<world>.yaml` 형식. 불가한 게이지는 주석으로 사유가 나오고 종료 코드 1.
````

- [ ] **Step 5: 구문 확인 + 테스트 재실행**

Run: `$V -m py_compile gauge_ocr/gauge_ocr/gauge_ocr_node.py gauge_ocr/launch/gauge_ocr.launch.py && PYTHONPATH=inspection_common:gauge_ocr $V -m pytest gauge_ocr/test -q`
Expected: 구문 오류 없음, 80 passed

Run: `PYTHONPATH=inspection_common:gauge_ocr $V -m gauge_ocr.make_viewpoints --world factory --sim-share ../simulation`
Expected: `gauge_line1`, `gauge_line2` 두 줄, 종료 코드 0

- [ ] **Step 6: 기록 (커밋 안 함)**

---

### Task 7: 열화상 순수 함수 — 온도 변환, 블롭 검출, 정렬, 겹쳐 그리기

**Files:**
- Create: `module2_inspection/thermal_fusion/thermal_fusion/thermal.py`
- Create: `module2_inspection/thermal_fusion/thermal_fusion/hotspot.py`
- Create: `module2_inspection/thermal_fusion/thermal_fusion/align.py`
- Create: `module2_inspection/thermal_fusion/thermal_fusion/overlay.py`
- Test: `module2_inspection/thermal_fusion/test/test_thermal.py`
- Test: `module2_inspection/thermal_fusion/test/test_hotspot.py`
- Test: `module2_inspection/thermal_fusion/test/test_align.py`
- Test: `module2_inspection/thermal_fusion/test/test_thermal_overlay.py`

**Interfaces:**
- Consumes: `CameraSpec` (Task 2)
- Produces:
  - `MODES = ("l16", "l8_thermal", "brightness")`, `to_temperature(img, mode, l8_min_k=253.15, l8_res=3.0) -> np.ndarray[float32]`
  - `Blob(x, y, w, h, area, t_max, cx, cy)`, `find_hot_blobs(temp, threshold, min_area=20) -> list[Blob]` (t_max 내림차순)
  - `k_from_spec(cam) -> np.ndarray(3×3)`, `thermal_to_rgb_h(k_rgb, k_th, baseline_z=0.03, depth=1.5) -> np.ndarray`, `warp_to_rgb(thermal, h, rgb_size=(640, 480)) -> (warped, valid)`, `map_points(pts_th, h) -> np.ndarray(N×2)`
  - `fuse(rgb, temp_warped, valid, t_lo, t_hi, alpha=0.45) -> np.ndarray`, `draw_blobs(img, blobs, h, alert) -> np.ndarray`

- [ ] **Step 1: 실패하는 테스트 작성**

`test/test_thermal.py`:
```python
"""픽셀 → 온도 변환."""

import numpy as np
import pytest

from thermal_fusion.thermal import to_temperature


def test_l16_centi_kelvin():
    img = np.array([[35815]], np.uint16)            # 358.15 K
    assert to_temperature(img, "l16")[0, 0] == pytest.approx(85.0, abs=1e-3)


def test_l8_thermal_linear():
    img = np.array([[0, 30]], np.uint8)
    t = to_temperature(img, "l8_thermal", l8_min_k=253.15, l8_res=3.0)
    assert t[0].tolist() == pytest.approx([-20.0, 70.0], abs=1e-3)


def test_brightness_passthrough_and_gray():
    assert to_temperature(np.array([[200]], np.uint8), "brightness")[0, 0] == 200.0
    bgr = np.full((2, 2, 3), 100, np.uint8)
    out = to_temperature(bgr, "brightness")
    assert out.shape == (2, 2) and out.dtype == np.float32


def test_unknown_mode():
    with pytest.raises(ValueError):
        to_temperature(np.zeros((1, 1), np.uint8), "ir")
```

`test/test_hotspot.py`:
```python
"""과열 블롭 검출 (STUDY 5.6)."""

import numpy as np

from thermal_fusion.hotspot import find_hot_blobs


def _scene():
    t = np.full((240, 320), 25.0, np.float32)
    t[100:115, 150:170] = 85.0                       # 20×15 블롭
    t[10, 10] = 90.0                                 # 한 픽셀 노이즈
    return t


def test_single_blob_ignores_noise():
    blobs = find_hot_blobs(_scene(), 60.0)
    assert len(blobs) == 1
    b = blobs[0]
    assert (b.x, b.y, b.w, b.h) == (150, 100, 20, 15)
    assert b.t_max == 85.0
    assert (b.cx, b.cy) == (159.5, 107.0)


def test_sorted_by_temperature():
    t = _scene()
    t[200:215, 20:40] = 95.0
    assert [b.t_max for b in find_hot_blobs(t, 60.0)] == [95.0, 85.0]


def test_nothing_hot():
    assert find_hot_blobs(np.full((240, 320), 25.0, np.float32), 60.0) == []
```

`test/test_align.py`:
```python
"""열화상 → RGB 정렬 (STUDY 5.6 합성 검증)."""

import numpy as np
import pytest

from inspection_common.world_map import CameraSpec
from thermal_fusion.align import k_from_spec, map_points, thermal_to_rgb_h, warp_to_rgb

KR = k_from_spec(CameraSpec(0.28, 0.0, 0.05, 0.0, 1.396, 640, 480))
KT = k_from_spec(CameraSpec(0.28, 0.0, 0.02, 0.0, 1.0, 320, 240))


def project(k, X, Y, Z):
    """카메라 기준 점 (앞 X, 왼쪽 Y, 위 Z) → 픽셀."""
    return np.array([k[0, 0] * (-Y / X) + k[0, 2], k[1, 1] * (-Z / X) + k[1, 2]])


@pytest.mark.parametrize("Y,Z", [(0.0, 0.0), (0.3, -0.2), (-0.4, 0.25)])
def test_points_at_design_depth_align(Y, Z):
    h = thermal_to_rgb_h(KR, KT, baseline_z=0.03, depth=1.5)
    th = project(KT, 1.5, Y, Z)
    rgb = project(KR, 1.5, Y, Z - 0.03)                # RGB는 열화상보다 0.03 m 위
    assert map_points(th[None], h)[0] == pytest.approx(rgb, abs=1e-6)


def test_depth_mismatch_error():
    h = thermal_to_rgb_h(KR, KT, baseline_z=0.03, depth=1.5)
    th = project(KT, 0.4, 0.0, 0.0)
    rgb = project(KR, 0.4, 0.0, -0.03)
    err = np.linalg.norm(map_points(th[None], h)[0] - rgb)
    assert err == pytest.approx(20.98, abs=0.05)       # 381.46 · 0.03 · (1/0.4 − 1/1.5)


def test_warp_valid_region_size():
    h = thermal_to_rgb_h(KR, KT)
    _, valid = warp_to_rgb(np.zeros((240, 320), np.float32), h)
    ys, xs = np.nonzero(valid)
    assert xs.max() - xs.min() + 1 == pytest.approx(417, abs=2)
    assert ys.max() - ys.min() + 1 == pytest.approx(313, abs=2)
```

`test/test_thermal_overlay.py`:
```python
"""열지도 합성과 박스 그리기."""

import numpy as np

from thermal_fusion.hotspot import Blob
from thermal_fusion.overlay import draw_blobs, fuse


def test_fuse_keeps_rgb_outside_valid():
    rgb = np.full((4, 4, 3), 50, np.uint8)
    temp = np.full((4, 4), 90.0, np.float32)
    valid = np.zeros((4, 4), np.uint8)
    valid[:2] = 255
    out = fuse(rgb, temp, valid, 20.0, 90.0)
    assert (out[2:] == 50).all()
    assert not (out[:2] == 50).all()


def test_draw_blobs_red_when_alert():
    img = np.zeros((480, 640, 3), np.uint8)
    blob = Blob(100, 100, 20, 20, 400, 85.0, 110.0, 110.0)
    out = draw_blobs(img, [blob], np.eye(3), alert=True)
    assert tuple(out[100, 100]) == (0, 0, 255)
```

- [ ] **Step 2: 실패 확인**

Run: `PYTHONPATH=inspection_common:thermal_fusion $V -m pytest thermal_fusion/test -q`
Expected: FAIL, `ModuleNotFoundError` (thermal, hotspot, align, overlay)

- [ ] **Step 3: 구현**

`thermal_fusion/thermal.py`:
```python
"""열화상 픽셀 → 온도(℃). brightness는 지금 시뮬 흑백 센서용으로 밝기를 그대로 쓴다(단위 없음)."""

import cv2
import numpy as np

MODES = ("l16", "l8_thermal", "brightness")


def to_temperature(img: np.ndarray, mode: str, l8_min_k: float = 253.15, l8_res: float = 3.0) -> np.ndarray:
    if mode == "l16":                                   # 0.01 K 단위
        return img.astype(np.float32) * np.float32(0.01) - np.float32(273.15)
    if mode == "l8_thermal":                            # ThermalSensor 플러그인 L8
        return (np.float32(l8_min_k) + img.astype(np.float32) * np.float32(l8_res)
                - np.float32(273.15))
    if mode == "brightness":
        if img.ndim == 3:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return img.astype(np.float32)
    raise ValueError(f"thermal_mode는 {MODES} 중 하나 (받은 값: {mode})")
```

`thermal_fusion/hotspot.py`:
```python
"""과열 영역(블롭) 검출. 입력은 to_temperature 결과."""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Blob:
    x: int
    y: int
    w: int
    h: int
    area: int
    t_max: float
    cx: float
    cy: float


def find_hot_blobs(temp: np.ndarray, threshold: float, min_area: int = 20) -> list:
    mask = (temp >= threshold).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))     # 점 노이즈 제거
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
    blobs = []
    for i in range(1, n):                                                          # 0은 배경
        x, y, w, h, area = stats[i]
        if area < min_area:
            continue
        t_max = float(temp[labels == i].max())
        blobs.append(Blob(int(x), int(y), int(w), int(h), int(area), t_max, *map(float, cents[i])))
    return sorted(blobs, key=lambda b: b.t_max, reverse=True)
```

`thermal_fusion/align.py`:
```python
"""열화상 → RGB 정렬. 두 카메라는 방향이 같고 높이만 다르다."""

import cv2
import numpy as np


def k_from_spec(cam) -> np.ndarray:
    """CameraSpec → K. 정사각 픽셀, 주점 = 이미지 중앙."""
    fx = cam.fx
    return np.array([[fx, 0, cam.width / 2], [0, fx, cam.height / 2], [0, 0, 1]], np.float64)


def thermal_to_rgb_h(k_rgb: np.ndarray, k_th: np.ndarray, baseline_z: float = 0.03,
                     depth: float = 1.5) -> np.ndarray:
    """열화상 픽셀 → RGB 픽셀 호모그래피. 열화상이 baseline_z(m) 아래, 물체 거리 depth(m)."""
    h = k_rgb @ np.linalg.inv(k_th)
    dv = k_rgb[1, 1] * baseline_z / depth              # 열화상이 아래 → RGB로 옮길 때 아래(+v)로
    shift = np.array([[1, 0, 0], [0, 1, dv], [0, 0, 1]], np.float64)
    return shift @ h


def warp_to_rgb(thermal: np.ndarray, h: np.ndarray, rgb_size=(640, 480)) -> tuple:
    """(옮긴 배열, 유효 영역 마스크). rgb_size는 (너비, 높이)."""
    warped = cv2.warpPerspective(thermal, h, rgb_size, flags=cv2.INTER_LINEAR)
    valid = cv2.warpPerspective(np.full(thermal.shape[:2], 255, np.uint8), h, rgb_size,
                                flags=cv2.INTER_NEAREST)
    return warped, valid


def map_points(pts_th: np.ndarray, h: np.ndarray) -> np.ndarray:
    return cv2.perspectiveTransform(pts_th.reshape(-1, 1, 2).astype(np.float64), h).reshape(-1, 2)
```

`thermal_fusion/overlay.py`:
```python
"""RGB 위에 정렬된 열지도와 과열 박스를 그린다."""

import cv2
import numpy as np

from thermal_fusion.align import map_points


def fuse(rgb: np.ndarray, temp_warped: np.ndarray, valid: np.ndarray, t_lo: float, t_hi: float,
         alpha: float = 0.45) -> np.ndarray:
    """t_lo~t_hi를 컬러맵으로 칠해 유효 영역에만 반투명 합성."""
    span = max(t_hi - t_lo, 1e-6)                       # lo == hi 설정이어도 NaN이 나지 않게
    norm = np.clip((temp_warped - t_lo) / span, 0, 1)
    color = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_INFERNO)
    blended = cv2.addWeighted(rgb, 1 - alpha, color, alpha, 0)
    out = rgb.copy()
    out[valid > 0] = blended[valid > 0]                 # 열화상 화각 밖은 원본 그대로
    return out


def draw_blobs(img: np.ndarray, blobs, h: np.ndarray, alert: bool) -> np.ndarray:
    color = (0, 0, 255) if alert else (0, 255, 255)
    for b in blobs:
        corners = np.array([[b.x, b.y], [b.x + b.w, b.y + b.h]], np.float64)
        (x0, y0), (x1, y1) = map_points(corners, h).astype(int)
        cv2.rectangle(img, (x0, y0), (x1, y1), color, 2)
        cv2.putText(img, f"{b.t_max:.0f}", (x0, max(y0 - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(img, "ALERT" if alert else "normal", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 0, 255) if alert else (0, 200, 0), 2, cv2.LINE_AA)
    return img
```

- [ ] **Step 4: 통과 확인**

Run: `PYTHONPATH=inspection_common:thermal_fusion $V -m pytest thermal_fusion/test -q`
Expected: 4 + 3 + 5 + 2 = 14 passed

- [ ] **Step 5: 기록 (커밋 안 함)**

---

### Task 8: `locate` — 열화상 픽셀 → 맵 위 과열 지점

**Files:**
- Create: `module2_inspection/thermal_fusion/thermal_fusion/locate.py`
- Test: `module2_inspection/thermal_fusion/test/test_locate.py`

**Interfaces:**
- Consumes: `pixel_angles`, `camera_xyz`, `ray_hits` (Task 3); `CameraSpec`, `load_world`, `resolve_world` (Task 2)
- Produces:
  - `Hit(x, y, z, dist, depth, name)`; `dist` = 카메라→지점 수평 거리, `depth` = 카메라 광축 방향 거리(정렬용)
  - `locate(u, v, rx, ry, ryaw, cam, body_height, obstacles, max_range=15.0) -> tuple[Hit | None, str]`; 실패 사유 `"floor"` / `"no_hit"`

- [ ] **Step 1: 실패하는 테스트 작성** — `test/test_locate.py`

```python
"""픽셀 → 맵 위치. factory 실제 맵으로 확인."""

import math
from pathlib import Path

import pytest

from inspection_common.world_map import load_world, resolve_world
from thermal_fusion.locate import locate

SIM = Path(__file__).resolve().parents[3] / "simulation"
WM = load_world(*resolve_world("factory", sim_share=str(SIM)))
TH = WM.thermal                                          # 320×240, hfov 1.0, z 0.02


def _loc(u, v, rx, ry, ryaw):
    return locate(u, v, rx, ry, ryaw, TH, 0.4, WM.obstacles)


def test_center_pixel_hits_tank_west_surface():
    hit, reason = _loc(160, 120, 5.0, -6.0, 0.0)
    assert reason == ""
    assert hit.name.startswith("gas_tank_1/")
    assert (hit.x, hit.y) == pytest.approx((7.65, -6.0), abs=1e-6)
    assert hit.dist == pytest.approx(7.65 - 5.28)
    assert hit.z == pytest.approx(0.42)


def test_pixel_below_horizon_hits_floor_first():
    hit, reason = _loc(160, 239, 5.0, -6.0, 0.0)          # 바닥 1.0 m 앞 < 탱크 2.37 m
    assert hit is None and reason == "floor"


def test_upper_pixel_hits_north_wall():
    hit, reason = _loc(160, 0, 0.0, 0.0, math.pi / 2)
    assert reason == ""
    assert hit.name.startswith("wall_north/")
    assert hit.y == pytest.approx(9.0)
    assert hit.z == pytest.approx(0.42 + (9.0 - 0.28) * 120 / math.hypot(TH.fx, 0), rel=1e-3)


def test_left_pixel_turns_left():
    hit, _ = _loc(0, 120, 0.0, 0.0, math.pi / 2)          # 북쪽을 보며 왼쪽 끝 → 서쪽으로 0.5 rad
    assert hit.x < 0.0
    assert math.atan2(hit.y - 0.28, hit.x) == pytest.approx(math.pi / 2 + 0.5, abs=1e-6)


def test_no_hit_when_out_of_range():
    hit, reason = locate(160, 100, 0.0, 0.0, math.pi / 2, TH, 0.4, WM.obstacles, max_range=3.0)
    assert hit is None and reason == "no_hit"
```

- [ ] **Step 2: 실패 확인**

Run: `PYTHONPATH=inspection_common:thermal_fusion $V -m pytest thermal_fusion/test/test_locate.py -q`
Expected: FAIL, `ModuleNotFoundError: No module named 'thermal_fusion.locate'`

- [ ] **Step 3: 구현** — `thermal_fusion/locate.py`

```python
"""열화상 픽셀 → 맵 위 과열 지점. 2D 광선 + 높이 확인 + 바닥 제외.

LiDAR(/points)로 바꿀 때는 같은 인자·반환으로 함수를 하나 더 만들어 노드에서 바꿔 부른다.
"""

from dataclasses import dataclass
import math

from inspection_common.geometry import camera_xyz, pixel_angles, ray_hits


@dataclass(frozen=True)
class Hit:
    x: float
    y: float
    z: float
    dist: float                 # 카메라 → 지점 수평 거리
    depth: float                # 광축 방향 거리 (정렬 호모그래피용)
    name: str


def locate(u: float, v: float, rx: float, ry: float, ryaw: float, cam, body_height: float,
           obstacles, max_range: float = 15.0) -> tuple:
    az, el = pixel_angles(u, v, cam)
    cx, cy, cz = camera_xyz(rx, ry, ryaw, cam, body_height)
    ang = ryaw + az
    ground = cz / math.tan(-el) if el < 0 else math.inf     # 수평선 아래면 바닥에 닿는 거리
    for d, o in ray_hits(cx, cy, ang, obstacles, max_range):
        if d > ground:
            break
        z = cz + d * math.tan(el)
        if o.z_min <= z <= o.z_max:                          # 장애물 위·아래로 지나가면 다음 것
            return Hit(cx + d * math.cos(ang), cy + d * math.sin(ang), z, d, d * math.cos(az), o.name), ""
    if ground <= max_range:
        return None, "floor"
    return None, "no_hit"
```

- [ ] **Step 4: 통과 확인**

Run: `PYTHONPATH=inspection_common:thermal_fusion $V -m pytest thermal_fusion/test -q`
Expected: 14 + 5 = 19 passed

- [ ] **Step 5: 기록 (커밋 안 함)**

---

### Task 9: `tracker` — 같은 지점 묶기와 상태 전이

**Files:**
- Create: `module2_inspection/thermal_fusion/thermal_fusion/tracker.py`
- Test: `module2_inspection/thermal_fusion/test/test_tracker.py`

**Interfaces:**
- Produces:
  - 상태 상수 `CANDIDATE, PENDING, INSPECTING, CONFIRMED, FALSE_ALARM, UNREACHABLE` (문자열 = 이름)
  - `Spot` (필드: `id, x, y, t_max, from_x, from_y, n_obs, last_seen, state, reason, goal, goal_since, attempts, inspect_start, inspect_tmax, decided_at`)
  - `Tracker(plan_goal, t_confirm, merge_radius=1.0, confirm_frames=3, forget_s=3.0, dwell_s=5.0, arrive_tol=0.4, stop_speed=0.05, inspect_timeout=60.0, max_attempts=2, recheck_s=0.0)`
    - `plan_goal(tx, ty, from_x, from_y) -> (goal (x, y, yaw) | None, reason)` — 노드에서는 `approach_point`
    - `observe(x, y, t, robot_x, robot_y, now) -> Spot`
    - `update(robot_x, robot_y, speed, now) -> None`
    - `goal() -> tuple | None`, `alert() -> bool`, `headline() -> Spot | None`, 속성 `spots: list[Spot]`

- [ ] **Step 1: 실패하는 테스트 작성** — `test/test_tracker.py`

```python
"""과열 지점 추적 상태 전이."""

import pytest

from thermal_fusion.tracker import (
    CANDIDATE, CONFIRMED, FALSE_ALARM, INSPECTING, PENDING, Tracker, UNREACHABLE,
)


def planner(tx, ty, fx, fy):
    return (tx - 1.5, ty, 0.0), ""


def make(**kw):
    return Tracker(plan_goal=planner, t_confirm=60.0, **kw)


def see(tr, n, x=7.65, y=-6.0, t=85.0, start=0.0):
    for i in range(n):
        tr.observe(x + 0.05 * (i % 2), y, t, 5.0, -6.0, start + 0.1 * i)


def test_repeated_detections_merge_and_become_pending():
    tr = make()
    see(tr, 2)
    assert len(tr.spots) == 1 and tr.spots[0].state == CANDIDATE
    see(tr, 1, start=0.2)
    s = tr.spots[0]
    assert s.state == PENDING and s.n_obs == 3
    assert tr.goal() is None                              # update 전에는 활성 지점 없음
    tr.update(5.0, -6.0, 0.3, 0.3)
    assert tr.goal() == pytest.approx((s.x - 1.5, -6.0, 0.0))


def test_far_detection_is_new_spot():
    tr = make()
    tr.observe(7.65, -6.0, 85.0, 5.0, -6.0, 0.0)
    tr.observe(0.0, 9.0, 85.0, 0.0, 0.0, 0.0)
    assert len(tr.spots) == 2


def test_candidate_forgotten():
    tr = make()
    see(tr, 2)
    tr.update(5.0, -6.0, 0.0, 3.5)
    assert tr.spots == []


def _pending(tr):
    see(tr, 3)
    tr.update(0.0, 0.0, 0.5, 1.0)
    return tr.spots[0]


def test_inspect_confirms():
    tr = make()
    s = _pending(tr)
    gx, gy, _ = s.goal
    tr.update(gx + 0.2, gy, 0.01, 10.0)
    assert s.state == INSPECTING
    tr.observe(s.x, s.y, 80.0, gx, gy, 12.0)
    tr.update(gx, gy, 0.0, 14.9)
    assert s.state == INSPECTING
    tr.update(gx, gy, 0.0, 15.0)
    assert s.state == CONFIRMED
    assert tr.alert() is True and tr.goal() is None


def test_inspect_without_heat_is_false_alarm():
    tr = make()
    s = _pending(tr)
    gx, gy, _ = s.goal
    tr.update(gx, gy, 0.0, 10.0)
    tr.update(gx, gy, 0.0, 15.0)
    assert s.state == FALSE_ALARM and tr.alert() is False


def test_moving_robot_does_not_start_inspection():
    tr = make()
    s = _pending(tr)
    gx, gy, _ = s.goal
    tr.update(gx, gy, 0.3, 10.0)
    assert s.state == PENDING


def test_timeout_retries_once_then_unreachable():
    tr = make()
    s = _pending(tr)                                      # goal_since = 1.0
    tr.update(0.0, 0.0, 0.5, 61.5)
    assert s.state == PENDING and s.reason == "unreachable" and s.attempts == 1
    tr.update(0.0, 0.0, 0.5, 62.0)                        # 재시도 시작
    tr.update(0.0, 0.0, 0.5, 122.5)
    assert s.state == UNREACHABLE and tr.goal() is None and tr.alert() is False


def test_planner_failure_is_unreachable():
    tr = Tracker(plan_goal=lambda *a: (None, "no_space:wall"), t_confirm=60.0)
    see(tr, 3)
    s = tr.spots[0]
    assert s.state == UNREACHABLE and s.reason == "no_space:wall"


def test_decided_spot_is_not_recreated():
    tr = make()
    s = _pending(tr)
    gx, gy, _ = s.goal
    tr.update(gx, gy, 0.0, 10.0)
    tr.observe(s.x, s.y, 80.0, gx, gy, 11.0)
    tr.update(gx, gy, 0.0, 15.0)
    see(tr, 5, start=100.0)
    assert len(tr.spots) == 1 and s.state == CONFIRMED
    tr.update(gx, gy, 0.0, 101.0)
    assert tr.goal() is None


def test_recheck_resets_after_time():
    tr = make(recheck_s=600.0)
    s = _pending(tr)
    gx, gy, _ = s.goal
    tr.update(gx, gy, 0.0, 10.0)
    tr.update(gx, gy, 0.0, 15.0)                          # FALSE_ALARM at 15
    tr.observe(s.x, s.y, 85.0, 5.0, -6.0, 700.0)
    assert s.state == CANDIDATE and s.n_obs == 1


def test_active_spot_is_sticky():
    tr = make()
    see(tr, 3, x=7.65)                                    # goal (6.15, -6)
    see(tr, 3, x=0.0, y=9.0)                              # goal (-1.5, 9)
    tr.update(6.0, -6.0, 0.5, 1.0)
    first = tr.goal()
    tr.update(-1.0, 9.0, 0.5, 2.0)                        # 로봇이 다른 지점 쪽으로 밀려도
    assert tr.goal() == first


def test_headline_prefers_active_then_confirmed():
    tr = make()
    assert tr.headline() is None
    see(tr, 2)
    assert tr.headline().state == CANDIDATE
    see(tr, 1, start=0.2)
    tr.update(5.0, -6.0, 0.5, 0.3)
    assert tr.headline().state == PENDING
```

- [ ] **Step 2: 실패 확인**

Run: `PYTHONPATH=inspection_common:thermal_fusion $V -m pytest thermal_fusion/test/test_tracker.py -q`
Expected: FAIL, `ModuleNotFoundError: No module named 'thermal_fusion.tracker'`

- [ ] **Step 3: 구현** — `thermal_fusion/tracker.py`

```python
"""과열 지점 묶기와 상태 전이. 시간·위치를 인자로 받는 순수 클래스.

CANDIDATE --(confirm_frames번 관측)--> PENDING --(goal 도착·정차)--> INSPECTING
INSPECTING --(dwell_s 뒤 최고값 ≥ t_confirm)--> CONFIRMED, 아니면 FALSE_ALARM
PENDING --(inspect_timeout × max_attempts)--> UNREACHABLE
확정된 지점은 recheck_s(0이면 안 함)가 지나야 다시 후보가 된다.
"""

from dataclasses import dataclass
import math

CANDIDATE, PENDING, INSPECTING = "CANDIDATE", "PENDING", "INSPECTING"
CONFIRMED, FALSE_ALARM, UNREACHABLE = "CONFIRMED", "FALSE_ALARM", "UNREACHABLE"
DECIDED = (CONFIRMED, FALSE_ALARM, UNREACHABLE)
_HEADLINE_ORDER = (CONFIRMED, CANDIDATE, UNREACHABLE, FALSE_ALARM)


@dataclass
class Spot:
    id: int
    x: float
    y: float
    t_max: float
    from_x: float               # 처음 본 로봇 위치 → 접근 방향
    from_y: float
    n_obs: int
    last_seen: float
    state: str = CANDIDATE
    reason: str = ""
    goal: tuple | None = None
    goal_since: float | None = None
    attempts: int = 0
    inspect_start: float | None = None
    inspect_tmax: float = -math.inf
    decided_at: float | None = None


class Tracker:
    def __init__(self, plan_goal, t_confirm: float, merge_radius: float = 1.0,
                 confirm_frames: int = 3, forget_s: float = 3.0, dwell_s: float = 5.0,
                 arrive_tol: float = 0.4, stop_speed: float = 0.05,
                 inspect_timeout: float = 60.0, max_attempts: int = 2, recheck_s: float = 0.0):
        self.plan_goal = plan_goal
        self.t_confirm = t_confirm
        self.merge_radius = merge_radius
        self.confirm_frames = confirm_frames
        self.forget_s = forget_s
        self.dwell_s = dwell_s
        self.arrive_tol = arrive_tol
        self.stop_speed = stop_speed
        self.inspect_timeout = inspect_timeout
        self.max_attempts = max_attempts
        self.recheck_s = recheck_s
        self.spots: list = []
        self._next_id = 1
        self._active_id: int | None = None

    def _nearest(self, x: float, y: float):
        near = [(math.hypot(s.x - x, s.y - y), s) for s in self.spots]
        near = [(d, s) for d, s in near if d <= self.merge_radius]
        return min(near, key=lambda ds: ds[0])[1] if near else None

    def observe(self, x: float, y: float, t: float, robot_x: float, robot_y: float, now: float):
        s = self._nearest(x, y)
        if s is None:
            s = Spot(self._next_id, x, y, t, robot_x, robot_y, 0, now)
            self._next_id += 1
            self.spots.append(s)
        elif s.state in DECIDED and self.recheck_s > 0 and now - s.decided_at >= self.recheck_s:
            s.state, s.reason, s.goal, s.attempts, s.n_obs = CANDIDATE, "", None, 0, 0
            s.goal_since, s.inspect_start = None, None      # 이전 주기의 시각이 남아 즉시 시간 초과되지 않도록
            s.t_max, s.from_x, s.from_y, s.decided_at = t, robot_x, robot_y, None
        s.n_obs += 1
        s.last_seen = now
        s.t_max = max(s.t_max, t)
        if s.state == CANDIDATE and s.n_obs > 1:              # 후보일 때만 위치를 평균으로 다듬는다
            s.x += (x - s.x) / s.n_obs
            s.y += (y - s.y) / s.n_obs
        if s.state == INSPECTING:
            s.inspect_tmax = max(s.inspect_tmax, t)
        if s.state == CANDIDATE and s.n_obs >= self.confirm_frames:
            s.goal, s.reason = self.plan_goal(s.x, s.y, s.from_x, s.from_y)
            if s.goal is None:
                s.state, s.decided_at = UNREACHABLE, now
            else:
                s.state = PENDING
        return s

    def _active(self, robot_x: float, robot_y: float):
        for s in self.spots:
            if s.id == self._active_id and s.state in (PENDING, INSPECTING):
                return s
        pending = [s for s in self.spots if s.state == PENDING]
        if not pending:
            self._active_id = None
            return None
        s = min(pending, key=lambda p: math.hypot(p.goal[0] - robot_x, p.goal[1] - robot_y))
        self._active_id = s.id
        return s

    def forget(self, now: float) -> None:
        """오래 안 보인 후보를 지운다. 로봇 위치가 없어도 부를 수 있다."""
        self.spots = [s for s in self.spots
                      if not (s.state == CANDIDATE and now - s.last_seen > self.forget_s)]

    def update(self, robot_x: float, robot_y: float, speed: float, now: float) -> None:
        self.forget(now)
        s = self._active(robot_x, robot_y)
        if s is None:
            return
        if s.state == PENDING:
            if s.goal_since is None:
                s.goal_since = now
            gx, gy, _ = s.goal
            if math.hypot(robot_x - gx, robot_y - gy) <= self.arrive_tol and speed < self.stop_speed:
                s.state, s.inspect_start, s.inspect_tmax = INSPECTING, now, -math.inf
                s.reason = ""                                 # 이전 시간 초과 사유 지움
            elif now - s.goal_since > self.inspect_timeout:
                s.attempts += 1
                s.goal_since, s.reason = None, "unreachable"
                if s.attempts >= self.max_attempts:
                    s.state, s.decided_at = UNREACHABLE, now
        elif s.state == INSPECTING and now - s.inspect_start >= self.dwell_s:
            s.state = CONFIRMED if s.inspect_tmax >= self.t_confirm else FALSE_ALARM
            s.decided_at = now
        if s.state in DECIDED:
            self._active_id = None

    def goal(self):
        for s in self.spots:
            if s.id == self._active_id and s.state in (PENDING, INSPECTING):
                return s.goal
        return None

    def alert(self) -> bool:
        return any(s.state == CONFIRMED for s in self.spots)

    def headline(self):
        """상태 토픽에 보일 지점: 활성 > 확정 > 후보 > 도달 불가 > 오탐 (같으면 최근)."""
        for s in self.spots:
            if s.id == self._active_id and s.state in (PENDING, INSPECTING):
                return s
        for state in (PENDING,) + _HEADLINE_ORDER:
            same = [s for s in self.spots if s.state == state]
            if same:
                return max(same, key=lambda s: s.last_seen)
        return None
```

- [ ] **Step 4: 통과 확인**

Run: `PYTHONPATH=inspection_common:thermal_fusion $V -m pytest thermal_fusion/test -q`
Expected: 19 + 12 = 31 passed

- [ ] **Step 5: 기록 (커밋 안 함)**

---

### Task 10: 열화상 노드 — 출력 함수, 노드, 런치, 설정

**Files:**
- Create: `module2_inspection/thermal_fusion/thermal_fusion/outputs.py`
- Test: `module2_inspection/thermal_fusion/test/test_outputs.py`
- Modify: `module2_inspection/thermal_fusion/thermal_fusion/thermal_fusion_node.py` (전체 교체)
- Modify: `module2_inspection/thermal_fusion/launch/thermal_fusion.launch.py` (전체 교체)
- Create: `module2_inspection/thermal_fusion/config/thermal_factory.yaml`, `config/thermal_corridor.yaml`
- Modify: `module2_inspection/thermal_fusion/setup.py` (config 설치)
- Modify: `module2_inspection/thermal_fusion/package.xml` (의존성)
- Modify: `module2_inspection/thermal_fusion/README.md` (토픽 표)

**Interfaces:**
- Consumes: Task 2 `load_world`, `resolve_world`, `body_height_for`; Task 4 `approach_point`; Task 7 전부; Task 8 `locate`; Task 9 `Tracker`; Task 1 `format_state`
- Produces:
  - `active_goal(goal, odom_age: float | None, odom_timeout: float) -> tuple | None`
  - `state_line(spot, unit: str, no_map: bool = False, no_image: bool = False) -> str`
  - 토픽: `/inspection/thermal_alert`(Bool 0.5 s), `/inspection/thermal_goal`(PoseStamped 2 Hz), `/inspection/thermal_hotspots`(PoseArray latched), `/inspection/thermal_overlay`(Image), `/inspection/thermal_state`(String)

- [ ] **Step 1: 실패하는 테스트 작성** — `test/test_outputs.py`

```python
"""노드 출력 결정 (ROS 없이)."""

from inspection_common.state_msg import parse_state
from thermal_fusion.outputs import active_goal, state_line
from thermal_fusion.tracker import Spot


def test_active_goal_passes_when_fresh():
    assert active_goal((1.0, 2.0, 0.0), 0.2, 1.0) == (1.0, 2.0, 0.0)


def test_active_goal_none_when_odom_stale():
    assert active_goal((1.0, 2.0, 0.0), 1.5, 1.0) is None
    assert active_goal((1.0, 2.0, 0.0), None, 1.0) is None
    assert active_goal(None, 0.1, 1.0) is None


def test_state_line_pending():
    s = Spot(1, 7.65, -6.0, 85.2, 5.0, -6.0, 3, 0.0, state="PENDING", goal=(6.1, -6.0, 0.0))
    assert state_line(s, "C") == "PENDING | hotspot_1 (7.65, -6.00) | 85.2 C | goal (6.1, -6.0)"


def test_state_line_reason_and_idle():
    s = Spot(2, 0.0, 8.9, 250.0, 0.0, 0.0, 3, 0.0, state="UNREACHABLE", reason="no_space:wall_north/link")
    state, fields = parse_state(state_line(s, "lv"))
    assert state == "UNREACHABLE" and fields[-1] == "no_space:wall_north/link"
    assert state_line(None, "C") == "IDLE | no hotspot"
    assert state_line(s, "C", no_map=True) == "IDLE | no_map"


def test_state_line_no_image():
    s = Spot(1, 7.65, -6.0, 85.2, 5.0, -6.0, 3, 0.0, state="CONFIRMED")
    assert state_line(s, "C", no_image=True) == "IDLE | no image"
    assert state_line(s, "C", no_map=True, no_image=True) == "IDLE | no_map"
```

- [ ] **Step 2: 실패 확인**

Run: `PYTHONPATH=inspection_common:thermal_fusion $V -m pytest thermal_fusion/test/test_outputs.py -q`
Expected: FAIL, `ModuleNotFoundError: No module named 'thermal_fusion.outputs'`

- [ ] **Step 3: 구현** — `thermal_fusion/outputs.py`

```python
"""노드가 내보낼 goal·상태 문자열을 정한다 (ROS 없이 테스트)."""

from inspection_common.state_msg import format_state


def active_goal(goal, odom_age, odom_timeout: float):
    """로봇 위치가 최신일 때만 goal을 낸다. 아니면 None → 빈 frame_id로 발행."""
    if goal is None or odom_age is None or odom_age > odom_timeout:
        return None
    return goal


def state_line(spot, unit: str, no_map: bool = False, no_image: bool = False) -> str:
    """알람(thermal_alert)은 마지막 확정 상태를 유지하고, 상태 문자열만 원인을 보여 준다."""
    if no_map:
        return format_state("IDLE", "no_map")
    if no_image:
        return format_state("IDLE", "no image")
    if spot is None:
        return format_state("IDLE", "no hotspot")
    fields = [f"hotspot_{spot.id} ({spot.x:.2f}, {spot.y:.2f})", f"{spot.t_max:.1f} {unit}"]
    if spot.goal is not None and spot.state in ("PENDING", "INSPECTING"):
        fields.append(f"goal ({spot.goal[0]:.1f}, {spot.goal[1]:.1f})")
    if spot.reason:
        fields.append(spot.reason)
    return format_state(spot.state, *fields)
```

- [ ] **Step 4: 통과 확인**

Run: `PYTHONPATH=inspection_common:thermal_fusion $V -m pytest thermal_fusion/test -q`
Expected: 31 + 5 = 36 passed

- [ ] **Step 5: 노드 교체** — `thermal_fusion/thermal_fusion_node.py`

```python
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
from thermal_fusion.overlay import draw_blobs, fuse
from thermal_fusion.thermal import to_temperature
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
        self._bridge = CvBridge()
        self._pose = None                                  # (x, y, yaw) map
        self._speed = 0.0
        self._odom_t = None
        self._img_t = None                                 # 마지막 열화상 수신 시각
        self._rgb = None                                   # 겹쳐 그리기용 최신 RGB 메시지
        self._hits = []
        self._hotspots_key = None
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
        depth = max(min((h.depth for h in self._hits), default=1.5), 0.3)   # 실제 거리로 정렬, 0 나눗셈 방지
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
        if self._pose is not None and age is not None and age <= p("odom_timeout"):
            self._tracker.update(self._pose[0], self._pose[1], self._speed, self._now())
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
        text = state_line(self._tracker.headline(), unit, no_map=self._map is None, no_image=no_image)
        self.pub_state.publish(String(data=text))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ThermalFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 6: 런치 교체** — `launch/thermal_fusion.launch.py`

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    world = LaunchConfiguration("world")
    params_file = PathJoinSubstitution(
        [FindPackageShare("thermal_fusion"), "config", ["thermal_", world, ".yaml"]]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="corridor"),
            DeclareLaunchArgument("locomotion", default_value="velocity"),  # full_system이 넘겨 줌
            Node(
                package="thermal_fusion",
                executable="thermal_fusion_node",
                name="thermal_fusion",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "world": world,
                        "locomotion": LaunchConfiguration("locomotion"),
                        "use_sim_time": True,
                    },
                ],
            ),
        ]
    )
```

- [ ] **Step 7: 설정 파일** — `config/thermal_factory.yaml` (`thermal_corridor.yaml`은 첫 줄 주석의 world 이름만 corridor로)

```yaml
# thermal_fusion 파라미터 (factory). 지금 시뮬 열화상은 흑백 밝기 카메라라 brightness 모드.
# L16 열화상 센서로 바꾸면: thermal_mode l16, t_detect 60.0, t_confirm 60.0, overlay 20~90.
thermal_fusion:
  ros__parameters:
    thermal_mode: brightness    # l16 | l8_thermal | brightness
    t_detect: 240.0             # 블롭 검출 임계 (brightness는 0~255 밝기)
    t_confirm: 240.0            # 점검 위치에서 이 값 이상이면 확정
    min_area: 20                # 열화상 px
    merge_radius: 1.0           # m 이내 검출은 같은 지점
    confirm_frames: 3
    forget_s: 3.0
    dwell_s: 5.0
    arrive_tol: 0.4
    stop_speed: 0.05
    inspect_timeout: 60.0
    max_attempts: 2
    recheck_s: 0.0              # 0이면 확정된 지점을 다시 보지 않음
    approach_dist: 1.5
    robot_radius: 0.35
    max_range: 15.0
    odom_timeout: 1.0
    image_timeout: 1.0          # 열화상이 이만큼 안 오면 상태 IDLE | no image
    overlay_lo: 100.0
    overlay_hi: 255.0
    publish_overlay: true
```

- [ ] **Step 8: setup.py·package.xml**

`setup.py` `data_files`에 한 줄 추가:
```python
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
```

`package.xml`의 `<exec_depend>std_msgs</exec_depend>` 아래에 추가:
```xml
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>cv_bridge</exec_depend>
  <exec_depend>python3-opencv</exec_depend>
  <exec_depend>python3-numpy</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>inspection_common</exec_depend>
```

- [ ] **Step 9: README 토픽 표** — `thermal_fusion/README.md`에 토픽 표를 다음으로 둔다:

```markdown
| 토픽 | 방향 | 내용 |
|------|------|------|
| `/thermal/image` | 구독 (Image) | 열화상 (지금 시뮬은 흑백 밝기 → `thermal_mode: brightness`) |
| `/camera/image` | 구독 (Image) | 겹쳐 그리기용 RGB |
| `/odom` | 구독 (Odometry) | 로봇 위치(map)·속도 |
| `/inspection/thermal_alert` | 발행 (Bool, 0.5 s) | 점검 위치에서 확정된 과열이 있으면 True |
| `/inspection/thermal_goal` | 발행 (PoseStamped, 2 Hz) | 점검할 지점이 있을 때 접근 위치. 없으면 빈 frame_id |
| `/inspection/thermal_hotspots` | 발행 (PoseArray, latched) | 모든 지점 위치 |
| `/inspection/thermal_overlay` | 발행 (Image) | RGB + 열지도 + 박스 (구독자 있을 때만) |
| `/inspection/thermal_state` | 발행 (String, 0.5 s) | `상태 \| hotspot_N (x, y) \| 값 \| goal/사유` |
```

- [ ] **Step 10: 구문 확인 + 전체 테스트**

Run: `$V -m py_compile thermal_fusion/thermal_fusion/thermal_fusion_node.py thermal_fusion/launch/thermal_fusion.launch.py && PYTHONPATH=inspection_common:thermal_fusion $V -m pytest thermal_fusion/test -q`
Expected: 구문 오류 없음, 36 passed

- [ ] **Step 11: 기록 (커밋 안 함)**

---

### Task 11: 인터페이스 표 정리 + 최종 확인

**Files:**
- Modify: `module2_inspection/README.md`

**Interfaces:**
- Consumes: Task 6, 10의 토픽 목록

- [ ] **Step 1: README에 인터페이스 표 추가** — 기존 3줄 아래에:

````markdown
## 운학 파트 인터페이스 (게이지·열화상)

설계: [docs/inspection_interface_design.md](docs/inspection_interface_design.md)

| 토픽 | 타입 | 발행 | 쓰는 곳 (회의에서 확정) |
|------|------|------|------|
| `/inspection/gauge` | Float32 | gauge_ocr, 0.5 s | 관제 |
| `/inspection/gauge_state` | String | gauge_ocr, 0.5 s | 관제·통합 |
| `/inspection/gauge_viewpoints` | PoseArray (latched) | gauge_ocr | 순찰 웨이포인트 참고 |
| `/inspection/thermal_alert` | Bool | thermal_fusion, 0.5 s | 관제 |
| `/inspection/thermal_goal` | PoseStamped | thermal_fusion, 2 Hz | mpc_controller `goal_topics` (추가 시) |
| `/inspection/thermal_hotspots` | PoseArray (latched) | thermal_fusion | 관제 지도 |
| `/inspection/thermal_overlay` | Image | thermal_fusion | 디버그 |
| `/inspection/thermal_state` | String | thermal_fusion, 0.5 s | 관제·통합 |

상태 문자열은 `"상태 | 칸 | 칸"` 형식이고 `inspection_common.state_msg.parse_state()`로 읽는다.
좌표는 모두 `map`, goal 비활성은 빈 `frame_id`.

```bash
# 테스트 (ROS 없이)
cd module2_inspection
PYTHONPATH=inspection_common python3 -m pytest inspection_common/test -q
PYTHONPATH=inspection_common:gauge_ocr python3 -m pytest gauge_ocr/test -q
PYTHONPATH=inspection_common:thermal_fusion python3 -m pytest thermal_fusion/test -q
```
````

- [ ] **Step 2: 전체 테스트**

Run: 위 "테스트 실행 방법"의 세 줄
Expected: 39 passed / 80 passed / 36 passed

- [ ] **Step 3: 다른 담당 파일을 건드리지 않았는지 확인**

Run: `git -C .. status --short`
Expected: 바뀐 경로가 모두 `module2_inspection/inspection_common/`, `module2_inspection/gauge_ocr/`, `module2_inspection/thermal_fusion/`, `module2_inspection/README.md`, `module2_inspection/docs/` 아래

- [ ] **Step 4: 작업 보고 (커밋 안 함)** — 바뀐 파일 목록, 테스트 결과, ROS 실행 확인은 안 했다는 점, 회의 안건(설계 7절)을 운학 님께 알린다.
