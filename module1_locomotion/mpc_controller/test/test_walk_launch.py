import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "walk.launch.py"


def load_launch(path):
    spec = importlib.util.spec_from_file_location("walk_launch_under_test", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def default_text(arg):
    return arg.default_value[0].text


def test_controller_default_is_mpc():
    ld = load_launch(LAUNCH).generate_launch_description()
    args = {e.name: e for e in ld.entities if isinstance(e, DeclareLaunchArgument)}
    assert default_text(args["controller"]) == "mpc" and default_text(args["policy"]) == "flat_v3.npz"


def test_controller_rejects_unknown_value():
    ctx = LaunchContext()
    ctx.launch_configurations["controller"] = "bogus"
    with pytest.raises(ValueError):
        load_launch(LAUNCH)._launch_setup(ctx)


def test_course_args_default_off():
    ld = load_launch(LAUNCH).generate_launch_description()
    args = {e.name: default_text(e) for e in ld.entities if isinstance(e, DeclareLaunchArgument)}
    want = {"stairs_h": "0.0", "stairs_n": "3", "pipes_d": "0.0", "pipes_n": "5", "plate_mu": "0.0"}
    assert {k: args.get(k) for k in want} == want
