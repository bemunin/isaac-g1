"""Public API: open the G1 warehouse Scenario."""

from pathlib import Path

import carb.settings
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.app

__all__ = ["CONTROLLERS", "SCENARIO", "controller", "open_scenario", "ros"]

SCENARIO = Path(__file__).resolve().parents[3] / "usd" / "warehouse_g1_scene.usda"
CONTROLLERS = {"walk": 100, "wbc": 200}  # controller: physics steps per second it needs
CONTROLLER_SETTING = "/exts/oc.scenario.warehouse/controller"
ROS_SETTING = "/exts/oc.scenario.warehouse/ros"
# Isaac Sim extension that rescans the whole stage for gantry prims every 15 physics steps (~50 ms each here); the
# warehouse has no gantries.
GANTRY_EXTENSION = "isaacsim.robot_setup.virtual_gantry"
GRAPHS = "/World/G1/Graph"  # the G1's ROS 2 graphs: camera, lidar, TF, IMU, joint states and commands, odometry


def controller() -> str:
    """The G1's controller: "walk" (oc.control.g1_walk, default) or "wbc" (oc.control.g1_wbc_sonic)."""
    name = carb.settings.get_settings().get(CONTROLLER_SETTING) or "walk"
    if name not in CONTROLLERS:
        raise RuntimeError(f"{CONTROLLER_SETTING} must be one of {sorted(CONTROLLERS)}, got {name!r}")
    return name


def ros() -> bool:
    """Whether the G1 publishes its ROS 2 topics (off by default)."""
    return bool(carb.settings.get_settings().get(ROS_SETTING))


def open_scenario() -> None:
    """Open SCENARIO as the stage, replacing the current one. Its G1 is at /World/G1. Physics runs at the rate
    controller() needs (walk 100 Hz, wbc 200 Hz). Unless ros(), the G1's ROS 2 graphs are deactivated, so it publishes nothing and doesn't render its camera and
    lidar. Disables GANTRY_EXTENSION, whose stage scans would otherwise slow the simulation.

    Raises:
        RuntimeError: The Scenario could not be opened.
    """
    omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(GANTRY_EXTENSION, False)
    opened, stage = stage_utils.open_stage(str(SCENARIO))
    if not opened:
        raise RuntimeError(f"open_scenario: could not open {SCENARIO}")
    rate = CONTROLLERS[controller()]
    scene = stage.GetPrimAtPath("/PhysicsScene")
    for name in ("newton:timeStepsPerSecond", "physxScene:timeStepsPerSecond"):
        attribute = scene.GetAttribute(name)
        if attribute and attribute.Get() != rate:
            attribute.Set(rate)
    if not ros():
        stage.GetPrimAtPath(GRAPHS).SetActive(False)
