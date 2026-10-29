import ast
from pathlib import Path
import runpy
import xml.etree.ElementTree as ET

import setuptools


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_hand_adapter_has_no_body_control_dependencies():
    source = (
        PACKAGE_ROOT / "g1_pico_teleop" / "hand" / "hand_ros_adapter_node.py"
    ).read_text(encoding="utf-8")
    tree = ast.parse(source)
    imported_modules = {
        node.module
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom) and node.module is not None
    }

    assert "body_mapper" not in imported_modules
    assert "full_body" not in imported_modules
    assert "gear_sonic_interfaces.msg" not in imported_modules
    assert "geometry_msgs.msg" not in imported_modules
    assert "SmplMotion" not in source
    assert "enable_smpl_stream" not in source
    assert "_publish_body" not in source


def test_recommended_launch_selects_hand_only_source_and_adapter():
    source = (PACKAGE_ROOT / "launch" / "pico_hands.launch.py").read_text(
        encoding="utf-8"
    )

    assert '"--hands-only"' in source
    assert 'executable="pico_hand_adapter"' in source
    assert 'executable="pico_ros_adapter"' not in source


def test_packaging_only_exports_hand_runtime(monkeypatch):
    captured = {}
    monkeypatch.setattr(setuptools, "setup", lambda **kwargs: captured.update(kwargs))
    monkeypatch.chdir(PACKAGE_ROOT)
    runpy.run_path(str(PACKAGE_ROOT / "setup.py"), run_name="__main__")

    assert captured["entry_points"]["console_scripts"] == [
        "pico_hand_adapter = g1_pico_teleop.hand.hand_ros_adapter_node:main",
    ]
    assert set(captured["packages"]) == {
        "g1_pico_teleop",
        "g1_pico_teleop.hand",
    }
    launch_files = dict(captured["data_files"])["share/g1_pico_teleop/launch"]
    assert launch_files == ["launch/pico_hands.launch.py"]
    for filename in ("ros_adapter_node.py", "body_mapper.py"):
        assert not (PACKAGE_ROOT / "g1_pico_teleop" / filename).exists()
    for source in (PACKAGE_ROOT / "g1_pico_teleop").rglob("*.py"):
        ast.parse(source.read_text(encoding="utf-8"))


def test_body_dependencies_are_not_runtime_dependencies():
    root = ET.parse(PACKAGE_ROOT / "package.xml").getroot()
    runtime = {element.text for element in root.findall("exec_depend")}
    assert not {"gear_sonic_interfaces", "geometry_msgs"} & runtime
    assert "python3-zmq" in runtime  # Robot optical-hand transport, not body control.
    assert {"rclpy", "sensor_msgs", "trajectory_msgs", "std_srvs"} <= runtime
    assert "python3-scipy" not in runtime
    assert "g1_hardware" not in runtime


def test_launches_separate_udp_bind_address_from_source_destination():
    hands = (PACKAGE_ROOT / "launch" / "pico_hands.launch.py").read_text()
    assert "source_destination_host" in hands
    assert '"--host",\n                    listen_address' not in hands


def test_no_local_body_bridge_is_shipped():
    body_package = PACKAGE_ROOT / "g1_pico_teleop" / "body"
    assert not (body_package / "__init__.py").exists()
    assert not (body_package / "body_ros_adapter_node.py").exists()
    assert not (body_package / "official_pose.py").exists()
    assert not (PACKAGE_ROOT / "launch" / "pico_body_hands.launch.py").exists()
    assert not (PACKAGE_ROOT / "test" / "test_official_pose.py").exists()
