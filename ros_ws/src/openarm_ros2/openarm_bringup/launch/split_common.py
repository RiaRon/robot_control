"""Pieces every split bringup (OpenArm + one hand family) shares.

Hand-specific launch files live in ``rh56f1/`` and ``leap_hand/``; each adds this
directory to ``sys.path`` and imports from here, so neither hand imports the other.
"""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

FAKE_PLUGIN = "mock_components/GenericSystem"


def default_manifest_for(canonical_urdf: Path) -> Path:
    """``<stem>_manifest.yaml`` next to the canonical URDF."""

    canonical_urdf = Path(canonical_urdf)
    return canonical_urdf.with_name(canonical_urdf.stem + "_manifest.yaml")


def controller_manager(namespace: str) -> str:
    return f"/{namespace}/controller_manager" if namespace else "/controller_manager"


def prime_fake_trajectory_controllers(spawner, manager: str, controllers: list[str]):
    """Launch action: once *spawner* exits, run fake_trajectory_controller_prime.py.

    joint_trajectory_controller 2.47.0 can start with its pending-goal flag
    uninitialised and then silently drop topic trajectories; one finished
    hold-at-measured action goal clears it (see the script). Fake managers only.
    """
    from launch.actions import RegisterEventHandler
    from launch.event_handlers import OnProcessExit
    from launch_ros.actions import Node

    return RegisterEventHandler(OnProcessExit(target_action=spawner, on_exit=[Node(
        package="openarm_bringup",
        executable="fake_trajectory_controller_prime.py",
        arguments=[*controllers, "--controller-manager", manager],
        output="both",
    )]))


def ros2_control_blocks(description: str) -> dict[str, dict]:
    """name -> {plugin, joints, command interfaces} of every ros2_control block."""
    blocks = {}
    for block in ET.fromstring(description).findall("ros2_control"):
        plugin = block.find("hardware/plugin")
        joints = block.findall("joint")
        blocks[block.get("name")] = {
            "plugin": None if plugin is None else (plugin.text or "").strip(),
            "joints": [joint.get("name") for joint in joints],
            "commands": [
                f"{joint.get('name')}/{command.get('name')}"
                for joint in joints for command in joint.findall("command_interface")
            ],
        }
    return blocks
