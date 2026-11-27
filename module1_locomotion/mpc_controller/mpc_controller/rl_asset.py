"""학습용 URDF 생성기. Gazebo walk 변환본 SDF(make_legged_model)를 Genesis용 URDF로 옮긴다. ROS 의존 없음.

SDF 링크 pose는 모델 프레임 절대값, 관절 pose는 없음 → URDF 관절 origin = child − parent 링크 위치.
Genesis 1.5는 effort가 비거나 0이면 토크 무제한으로 보므로 모든 관절에 tau_max를 넣는다.
"""
import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from mpc_controller import sim2sim
from mpc_controller.legged_model import make_legged_model

VEL_MAX = 30.0  # URDF limit에 필수인 값, 토크 제한이 지배한다


def _pose(elem):
    """<pose> → xyz. 회전이 0이 아니면 거부(지금 모델에는 없다)."""
    p = elem.find("pose")
    v = [float(x) for x in p.text.split()] if p is not None else [0.0] * 6
    if any(v[3:]):
        raise ValueError(f"rotated pose not supported: {v}")
    return np.array(v[:3])


def _f(x):
    return repr(round(float(x), 9))  # 위치 뺄셈의 부동소수 잡음 제거


def _xyz(v):
    return " ".join(_f(x) for x in v)


def sdf_to_urdf(sdf_text: str, tau_max: float) -> str:
    model = ET.fromstring(sdf_text).find("model")
    pos, out = {}, ['<?xml version="1.0"?>', '<robot name="go2_box">']
    for link in model.findall("link"):
        pos[link.get("name")] = _pose(link)
    for link in model.findall("link"):
        name = link.get("name")
        inertia = link.find("inertial/inertia")
        out += [f'  <link name="{name}">', "    <inertial>", '      <origin xyz="0 0 0" rpy="0 0 0"/>',
                f'      <mass value="{_f(link.find("inertial/mass").text)}"/>',
                "      <inertia " + " ".join(f'{k}="{_f(inertia.find(k).text)}"'
                                              for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")) + "/>",
                "    </inertial>"]
        for col in link.findall("collision"):
            geo = col.find("geometry")
            if geo.find("box") is not None:
                shape = f'<box size="{geo.find("box/size").text}"/>'
            else:
                shape = f'<sphere radius="{_f(geo.find("sphere/radius").text)}"/>'
            out += ["    <collision>", f'      <origin xyz="{_xyz(_pose(col))}" rpy="0 0 0"/>',
                    f"      <geometry>{shape}</geometry>", "    </collision>"]
        out.append("  </link>")
    for j in model.findall("joint"):
        if j.get("type") != "revolute":
            raise ValueError(f"unsupported joint type: {j.get('type')}")
        parent, child = j.find("parent").text, j.find("child").text
        ax = j.find("axis")
        out += [f'  <joint name="{j.get("name")}" type="revolute">',
                f'    <parent link="{parent}"/>', f'    <child link="{child}"/>',
                f'    <origin xyz="{_xyz(pos[child] - pos[parent])}" rpy="0 0 0"/>',
                f'    <axis xyz="{ax.find("xyz").text}"/>',
                f'    <limit lower="{_f(ax.find("limit/lower").text)}" upper="{_f(ax.find("limit/upper").text)}"'
                f' effort="{_f(tau_max)}" velocity="{_f(VEL_MAX)}"/>',
                '    <dynamics damping="0" friction="0"/>', "  </joint>"]
    out.append("</robot>")
    return "\n".join(out) + "\n"


def make_go2_box_urdf(go2_sdf_text: str, cfg: dict) -> str:
    """Gazebo walk와 같은 입력(구 발·감쇠 0·스프링 제거)으로 변환."""
    sdf = make_legged_model(go2_sdf_text, cfg["nominal"]["mu"], 0.0, 0.0,
                            force_ctrl=True, foot_r=cfg["foot_r"], damping=0.0)
    return sdf_to_urdf(sdf, cfg["tau_max"])


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--sdf", required=True)
    ap.add_argument("--config", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    urdf = make_go2_box_urdf(Path(a.sdf).read_text(encoding="utf-8"), sim2sim.load(a.config))
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(urdf, encoding="utf-8")


if __name__ == "__main__":
    main()
