"""다리 구동 go2 변형(C안, 옵트인) — 런치 시 go2/model.sdf·월드를 구조 변환한다. ROS 의존 없음.

원본 구조가 기대와 다르면 ValueError로 시끄럽게 실패한다(조용히 깨진 모델로 시뮬이 뜨는 것 방지).
"""
import os
import tempfile
import xml.etree.ElementTree as ET

LEGS = ("FL", "FR", "RL", "RR")
PARTS = ("hip", "thigh", "calf")
JOINTS = tuple(f"{leg}_{part}_joint" for leg in LEGS for part in PARTS)
VEL_CTRL = "gz-sim-velocity-control-system"
POS_CTRL = "gz-sim-joint-position-controller-system"
MODEL_CONFIG = (
    '<?xml version="1.0"?><model><name>go2_legged</name><version>1.0</version>'
    '<sdf version="1.10">model.sdf</sdf></model>'
)


def _expect(step: str, expected, actual) -> None:
    if expected != actual:
        raise ValueError(f"legged 변환 실패[{step}]: 기대 {expected}, 실제 {actual} — 원본 SDF 구조가 바뀜")


def make_legged_model(sdf_text: str, mu: float = 0.8, p_gain: float = 120.0, d_gain: float = 2.0) -> str:
    """VelocityControl 제거, 관절별 위치 컨트롤러 12개, 다리 마찰 복원, 관절 스프링 제거."""
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

    n_mu = 0
    for link in model.findall("link"):
        if link.get("name", "").endswith(("_thigh", "_calf")):
            for ode in link.findall("collision/surface/friction/ode"):
                ode.find("mu").text = ode.find("mu2").text = str(mu)
                n_mu += 1
    _expect("다리 충돌 마찰", 8, n_mu)
    return ET.tostring(root, encoding="unicode")


def make_legged_world(world_text: str) -> str:
    """월드의 model://go2 include를 model://go2_legged로. include 이름(go2)은 유지 → 토픽명 불변."""
    root = ET.fromstring(world_text)
    uris = [u for u in root.iter("uri") if (u.text or "").strip() == "model://go2"]
    _expect("model://go2 include 수", 1, len(uris))
    uris[0].text = "model://go2_legged"
    return ET.tostring(root, encoding="unicode")


def bridge_entries() -> list:
    """ros_gz_bridge yaml 항목: /legged/<joint>(Float64) → /model/go2/legged/<joint>."""
    return [
        {
            "ros_topic_name": f"/legged/{j}",
            "gz_topic_name": f"/model/go2/legged/{j}",
            "ros_type_name": "std_msgs/msg/Float64",
            "gz_type_name": "gz.msgs.Double",
            "direction": "ROS_TO_GZ",
        }
        for j in JOINTS
    ]


def write_legged_assets(
    sim_share: str, world: str, mu: float = 0.8, p_gain: float = 120.0, d_gain: float = 2.0
) -> tuple:
    """변환한 모델·월드·브리지 yaml을 임시 dir에 쓰고 (world_file, bridge_yaml, models_dir)를 돌려준다.

    sim_share는 simulation 패키지 share(또는 소스) 경로. 변환을 먼저 해서 실패하면 임시 dir도 안 남는다.
    """
    import yaml  # 런치에서만 필요 — 모듈 import는 stdlib만으로 되게

    def read(*parts: str) -> str:
        with open(os.path.join(sim_share, *parts), encoding="utf-8") as f:
            return f.read()

    model_sdf = make_legged_model(read("models", "go2", "model.sdf"), mu, p_gain, d_gain)
    world_sdf = make_legged_world(read("worlds", f"{world}.sdf"))
    # ponytail: 실행마다 임시 dir 하나가 남는다(수십 KB). 거슬리면 런치 종료 핸들러에서 삭제
    out = tempfile.mkdtemp(prefix="go2_legged_")
    models_dir = os.path.join(out, "models")
    os.makedirs(os.path.join(models_dir, "go2_legged"))
    files = {
        os.path.join(models_dir, "go2_legged", "model.sdf"): model_sdf,
        os.path.join(models_dir, "go2_legged", "model.config"): MODEL_CONFIG,
        os.path.join(out, f"{world}.sdf"): world_sdf,
        os.path.join(out, "bridge.yaml"): yaml.safe_dump(bridge_entries()),
    }
    for path, text in files.items():
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)
    return os.path.join(out, f"{world}.sdf"), os.path.join(out, "bridge.yaml"), models_dir
