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
    assert default_text(args["controller"]) == "mpc" and default_text(args["policy"]) == ""


def test_controller_rejects_unknown_value():
    ctx = LaunchContext()
    ctx.launch_configurations["controller"] = "bogus"
    with pytest.raises(ValueError):
        load_launch(LAUNCH)._launch_setup(ctx)
