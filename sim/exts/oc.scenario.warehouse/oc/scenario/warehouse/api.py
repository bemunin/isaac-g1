"""Public API: open the G1 warehouse Scenario."""

from pathlib import Path

import carb.settings
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.app
from pxr import Usd

__all__ = ["OBSTACLES", "PHYSICS_RATE", "SCENARIO", "apply_obstacles", "obstacles", "open_scenario", "ros"]

SCENARIO = Path(__file__).resolve().parents[3] / "usd" / "warehouse_g1_scene.usda"
PHYSICS_RATE = 200  # physics steps per second: SONIC's training rate
ROS_SETTING = "/exts/oc.scenario.warehouse/ros"
OBSTACLES_SETTING = "/exts/oc.scenario.warehouse/obstacles"
# The SCENARIO marks these props `active = false`; apply_obstacles puts them back. Names are children of full_warehouse.
FULL_WAREHOUSE = "/Environment/full_warehouse"
_CLUTTER = (
    "SM_CardBoxA_02_2",
    "SM_CardBoxA_3",
    "SM_CardBoxD_04_6",
    "SM_CardBoxD_03_9",
    "SM_CardBoxD_4",
    "SM_CratePlasticNote_B_03_18",
    "SM_CratePlasticNote_B_5",
    "SM_PushcartA_02_22",
    "SM_BarelPlastic_C_01_25",
    "SM_BarelPlastic_C_2",
    "SM_CardBoxB_01_29",
    "SM_CardBoxB_02_32",
    "SM_BottlePlasticD_01_35",
    "SM_BottlePlasticD_2",
    "SM_BottlePlasticD_3",
    "SM_BottlePlasticD_4",
    "SM_BucketPlastic_B_41",
    "SM_CratePlastic_E_03_44",
    "SM_CratePlastic_D_01_47",
    "SM_PaperNote_Small_15",
    "SM_PaperNote_Small_06_54",
    "SM_PaperNote_Small_20",
    "SM_PaperNote_Small_16",
    "SM_PaperNote_Small_13",
    "SM_PaperNote_Small_17",
    "SM_PaperNote_Small_18",
    "SM_PaperNote_Small_09_72",
    "SM_PaperNote_Small_08_75",
    "SM_CardBoxB_3",
    "S_TrafficCone_2",
    "S_TrafficCone2",
    "S_TrafficCone3",
    "S_TrafficCone4",
    "S_TrafficCone5",
    "S_TrafficCone6",
    "S_WetFloorSign_2",
    "S_WetFloorSign2",
)
OBSTACLES = {
    "clutter": [f"{FULL_WAREHOUSE}/{name}" for name in _CLUTTER],
    "forklift": [f"{FULL_WAREHOUSE}/forklift"],
}
# Isaac Sim extension that rescans the whole stage for gantry prims every 15 physics steps (~50 ms each here); the
# warehouse has no gantries.
GANTRY_EXTENSION = "isaacsim.robot_setup.virtual_gantry"
GRAPHS = "/World/G1/Graph"  # the G1's ROS 2 graphs: camera, lidar, TF, IMU, joint states and commands, odometry


def ros() -> bool:
    """Whether the G1 publishes its ROS 2 topics (off by default)."""
    return bool(carb.settings.get_settings().get(ROS_SETTING))


def obstacles() -> list[str]:
    """The obstacles to show, from a comma-separated setting: "clutter" and/or "forklift" (none by default)."""
    value = carb.settings.get_settings().get(OBSTACLES_SETTING) or ""
    kinds = [kind.strip() for kind in value.split(",") if kind.strip() and kind.strip() != "none"]
    unknown = sorted(set(kinds) - OBSTACLES.keys())
    if unknown:
        raise RuntimeError(f"{OBSTACLES_SETTING} must list only {sorted(OBSTACLES)}, got {unknown}")
    return kinds


def apply_obstacles(stage: Usd.Stage, kinds: list[str]) -> None:
    """Activate the OBSTACLES of each kind in `kinds` on `stage`, in its session layer so SCENARIO is never changed.
    Prims missing from the stage are logged and skipped."""
    missing = []
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for kind in kinds:
            for path in OBSTACLES[kind]:
                prim = stage.GetPrimAtPath(path)
                if prim:
                    prim.SetActive(True)
                else:
                    missing.append(path)
    if missing:
        carb.log_warn(f"apply_obstacles: not on the stage, skipped: {missing}")


def open_scenario() -> None:
    """Open SCENARIO as the stage, replacing the current one. Its G1 is at /World/G1. Physics runs at PHYSICS_RATE
    (200 Hz). Unless ros(), the G1's ROS 2 graphs are deactivated, so it publishes nothing and doesn't render its
    camera and lidar. obstacles() are shown. Disables GANTRY_EXTENSION, whose stage scans would otherwise slow the simulation.

    Raises:
        RuntimeError: The Scenario could not be opened.
    """
    omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(GANTRY_EXTENSION, False)
    opened, stage = stage_utils.open_stage(str(SCENARIO))
    if not opened:
        raise RuntimeError(f"open_scenario: could not open {SCENARIO}")
    rate = PHYSICS_RATE
    scene = stage.GetPrimAtPath("/PhysicsScene")
    for name in ("newton:timeStepsPerSecond", "physxScene:timeStepsPerSecond"):
        attribute = scene.GetAttribute(name)
        if attribute and attribute.Get() != rate:
            attribute.Set(rate)
    if not ros():
        stage.GetPrimAtPath(GRAPHS).SetActive(False)
    apply_obstacles(stage, obstacles())
