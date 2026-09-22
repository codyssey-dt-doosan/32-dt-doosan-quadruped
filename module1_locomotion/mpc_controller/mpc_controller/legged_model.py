"""다리 구동 go2 변형(C안, 옵트인) — 런치 시 go2/model.sdf·월드를 구조 변환한다. ROS 의존 없음.

원본 구조가 기대와 다르면 ValueError로 시끄럽게 실패한다(조용히 깨진 모델로 시뮬이 뜨는 것 방지).
"""
import math
import os
import tempfile
import xml.etree.ElementTree as ET

LEGS = ("FL", "FR", "RL", "RR")
PARTS = ("hip", "thigh", "calf")
JOINTS = tuple(f"{leg}_{part}_joint" for leg in LEGS for part in PARTS)
VEL_CTRL = "gz-sim-velocity-control-system"
POS_CTRL = "gz-sim-joint-position-controller-system"
FORCE_CTRL = "gz-sim-apply-joint-force-system"
WRENCH_SYS = "gz-sim-apply-link-wrench-system"
RAMP_NAME = "balance_ramp"
RAMP_SIZE, RAMP_T = 3.0, 0.1
SPAWN_CLEARANCE = 0.4  # go2 기본 스폰 z와 같음
MODEL_CONFIG = (
    '<?xml version="1.0"?><model><name>go2_legged</name><version>1.0</version>'
    '<sdf version="1.10">model.sdf</sdf></model>'
)


def force_topic(joint: str) -> str:
    return f"/model/go2/joint/{joint}/cmd_force"  # ApplyJointForce 기본 토픽


def ros_force_topic(joint: str) -> str:
    return f"/legged_force/{joint}"


def ramp_spawn_z(deg: float) -> float:
    """판 최저 모서리가 z 0에 닿게 놓은 판의 중심 위 표면 + 스폰 여유."""
    th = math.radians(deg)
    return RAMP_SIZE / 2 * math.sin(th) + RAMP_T / 2 * math.cos(th) + RAMP_T / 2 + SPAWN_CLEARANCE


def _expect(step: str, expected, actual) -> None:
    if expected != actual:
        raise ValueError(f"legged 변환 실패[{step}]: 기대 {expected}, 실제 {actual} — 원본 SDF 구조가 바뀜")


def make_legged_model(
    sdf_text: str,
    mu: float = 0.8,
    p_gain: float = 120.0,
    d_gain: float = 2.0,
    *,
    force_ctrl: bool = False,
    foot_r: float = 0.0,
    damping: float | None = None,
) -> str:
    """VelocityControl 제거, 관절별 위치 컨트롤러 12개, 다리 마찰 복원, 관절 스프링 제거.

    damping은 None이면 원본(1.0) 유지. 힘 제어(balance)는 0.0 — gz-sim 8.6/DART가 정지 상태에도 감쇠 토크를
    먹어(calf ≈1 N·m) τ=−Jᵀf가 30% 부족해지고 무릎이 한계까지 접힘(2026-09-22 스파이크).
    """
    root = ET.fromstring(sdf_text)
    model = root.find("model")
    plugins = model.findall("plugin")
    vel = [p for p in plugins if p.get("filename") == VEL_CTRL]
    _expect("VelocityControl 플러그인 수", 1, len(vel))
    for p in vel + [p for p in plugins if p.get("filename") == POS_CTRL]:
        model.remove(p)

    joints = [j for j in model.findall("joint") if j.get("type") == "revolute"]
    _expect("revolute 관절", sorted(JOINTS), sorted(j.get("name") for j in joints))
    for j in joints:
        dyn = j.find("axis/dynamics")
        for tag in ("spring_stiffness", "spring_reference"):
            el = dyn.find(tag) if dyn is not None else None
            if el is not None:
                dyn.remove(el)
        if damping is not None:
            dyn.find("damping").text = str(damping)
        # 다중 <joint_name>은 첫 관절만 피드백하므로 관절마다 플러그인 하나
        ctrl = ET.SubElement(model, "plugin", filename=POS_CTRL, name="gz::sim::systems::JointPositionController")
        for tag, text in (
            ("joint_name", j.get("name")),
            ("topic", f"/model/go2/legged/{j.get('name')}"),
            ("p_gain", p_gain),
            ("i_gain", 0),
            ("d_gain", d_gain),
            ("cmd_max", 30),
            ("cmd_min", -30),
        ):
            ET.SubElement(ctrl, tag).text = str(text)

    if force_ctrl:  # 위치 컨트롤러 뒤에 있어야 힘이 합산됨(스파이크 확인)
        for j in joints:
            ctrl = ET.SubElement(model, "plugin", filename=FORCE_CTRL, name="gz::sim::systems::ApplyJointForce")
            ET.SubElement(ctrl, "joint_name").text = j.get("name")

    n_mu = 0
    for link in model.findall("link"):
        if link.get("name", "").endswith(("_thigh", "_calf")):
            for ode in link.findall("collision/surface/friction/ode"):
                ode.find("mu").text = ode.find("mu2").text = str(mu)
                n_mu += 1
        if foot_r > 0 and link.get("name", "").endswith("_calf"):
            col = ET.SubElement(link, "collision", name="foot")
            ET.SubElement(col, "pose").text = "0 0 -0.1 0 0 0"  # calf 축에서 L2 아래 = 구 중심
            ET.SubElement(ET.SubElement(ET.SubElement(col, "geometry"), "sphere"), "radius").text = str(foot_r)
            ode = ET.SubElement(ET.SubElement(ET.SubElement(col, "surface"), "friction"), "ode")
            ET.SubElement(ode, "mu").text = str(mu)
            ET.SubElement(ode, "mu2").text = str(mu)
    _expect("다리 충돌 마찰", 8, n_mu)
    return ET.tostring(root, encoding="unicode")


def make_legged_world(world_text: str, *, wrench: bool = False, ramp_deg: float = 0.0) -> str:
    """월드의 model://go2 include를 model://go2_legged로. include 이름(go2)은 유지 → 토픽명 불변."""
    root = ET.fromstring(world_text)
    uris = [u for u in root.iter("uri") if (u.text or "").strip() == "model://go2"]
    _expect("model://go2 include 수", 1, len(uris))
    uris[0].text = "model://go2_legged"

    world = root.find("world")
    if wrench:
        ET.SubElement(world, "plugin", filename=WRENCH_SYS, name="gz::sim::systems::ApplyLinkWrench")
    if ramp_deg:
        include = next(i for i in world.findall("include") if i.find("uri") is uris[0])
        x, y, _z, *rest = include.find("pose").text.split()
        include.find("pose").text = " ".join([x, y, f"{ramp_spawn_z(ramp_deg):.8f}", *rest])
        th = math.radians(ramp_deg)
        zc = RAMP_SIZE / 2 * math.sin(th) + RAMP_T / 2 * math.cos(th)
        world.append(
            ET.fromstring(
                f'<model name="{RAMP_NAME}"><static>true</static><pose>{x} {y} {zc:.4f} 0 {-th:.8f} 0</pose>'
                f'<link name="link"><collision name="c"><geometry><box><size>{RAMP_SIZE} {RAMP_SIZE} {RAMP_T}</size></box>'
                f'</geometry></collision><visual name="v"><geometry><box><size>{RAMP_SIZE} {RAMP_SIZE} {RAMP_T}</size>'
                "</box></geometry></visual></link></model>"
            )
        )
    return ET.tostring(root, encoding="unicode")


def bridge_entries(*, force: bool = False) -> list:
    """ros_gz_bridge yaml 항목: /legged/<joint>(Float64) → /model/go2/legged/<joint>."""
    entries = [
        {
            "ros_topic_name": f"/legged/{j}",
            "gz_topic_name": f"/model/go2/legged/{j}",
            "ros_type_name": "std_msgs/msg/Float64",
            "gz_type_name": "gz.msgs.Double",
            "direction": "ROS_TO_GZ",
        }
        for j in JOINTS
    ]
    if force:
        entries += [
            {
                "ros_topic_name": ros_force_topic(j),
                "gz_topic_name": force_topic(j),
                "ros_type_name": "std_msgs/msg/Float64",
                "gz_type_name": "gz.msgs.Double",
                "direction": "ROS_TO_GZ",
            }
            for j in JOINTS
        ]
    return entries


def write_legged_assets(
    sim_share: str,
    world: str,
    mu: float = 0.8,
    p_gain: float = 120.0,
    d_gain: float = 2.0,
    *,
    force_ctrl: bool = False,
    foot_r: float = 0.0,
    wrench: bool = False,
    ramp_deg: float = 0.0,
    damping: float | None = None,
) -> tuple:
    """변환한 모델·월드·브리지 yaml을 임시 dir에 쓰고 (world_file, bridge_yaml, models_dir)를 돌려준다.

    sim_share는 simulation 패키지 share(또는 소스) 경로. 변환을 먼저 해서 실패하면 임시 dir도 안 남는다.
    """
    import yaml  # 런치에서만 필요 — 모듈 import는 stdlib만으로 되게

    def read(*parts: str) -> str:
        with open(os.path.join(sim_share, *parts), encoding="utf-8") as f:
            return f.read()

    model_sdf = make_legged_model(
        read("models", "go2", "model.sdf"), mu, p_gain, d_gain, force_ctrl=force_ctrl, foot_r=foot_r, damping=damping
    )
    world_sdf = make_legged_world(read("worlds", f"{world}.sdf"), wrench=wrench, ramp_deg=ramp_deg)
    # ponytail: 실행마다 임시 dir 하나가 남는다(수십 KB). 거슬리면 런치 종료 핸들러에서 삭제
    out = tempfile.mkdtemp(prefix="go2_legged_")
    models_dir = os.path.join(out, "models")
    os.makedirs(os.path.join(models_dir, "go2_legged"))
    files = {
        os.path.join(models_dir, "go2_legged", "model.sdf"): model_sdf,
        os.path.join(models_dir, "go2_legged", "model.config"): MODEL_CONFIG,
        os.path.join(out, f"{world}.sdf"): world_sdf,
        os.path.join(out, "bridge.yaml"): yaml.safe_dump(bridge_entries(force=force_ctrl)),
    }
    for path, text in files.items():
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)
    return os.path.join(out, f"{world}.sdf"), os.path.join(out, "bridge.yaml"), models_dir
