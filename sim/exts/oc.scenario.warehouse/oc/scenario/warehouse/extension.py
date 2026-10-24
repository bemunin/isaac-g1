import asyncio
import gc

import carb
import carb.settings
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.ext
import omni.kit.app
from oc.robot.g1 import G1
from pxr import Usd

from .objects import ObjectManager
from .ui import WarehouseWindow

__all__ = ["Extension", "ObjectManager"]

_EXTENSION = "oc.scenario.warehouse"
_SETTINGS = f"/exts/{_EXTENSION}"  # carb settings: autostart, autoplay, ros, obstacles
_G1_PATH = "/World/G1"
# G1 spawn pose from the old warehouse_g1_scene.usda.
# Orientation is roll, pitch, yaw in degrees (extrinsic XYZ).
_G1_TRANSLATE = (-8.0, 9.5, 0.8)
_G1_ORIENT = (0.0, 0.0, 90.0)
_CLOCK_GRAPH = "/World/Graph/ROS2_Clock"  # publishes /clock; follows the ros setting

# Set on both PhysX and Newton, not through SimulationManager.setup_simulation, which sets only PhysX when the scene
# has PhysxSceneAPI (warehouse.usda applies PhysX, Newton and MuJoCo APIs).
_PHYSICS_SCENE = "/PhysicsScene"
_PHYSICS_RATE = 200  # physics steps per second: SONIC's training rate
_TIME_STEP_ATTRIBUTES = ("newton:timeStepsPerSecond", "physxScene:timeStepsPerSecond")
# Isaac Sim extension that rescans the whole stage for gantry prims every 15 physics steps; the warehouse has none.
_GANTRY_EXTENSION = "isaacsim.robot_setup.virtual_gantry"


def _warehouse_usd_path() -> str:
    return f"{app_utils.get_extension_path(_EXTENSION)}/usd/warehouse.usda"


def _autoplay_setting() -> bool:
    return bool(carb.settings.get_settings().get(f"{_SETTINGS}/autoplay"))


def _ros_setting() -> bool:
    return bool(carb.settings.get_settings().get(f"{_SETTINGS}/ros"))


def _obstacles_setting() -> list[str]:
    """Obstacle kinds from the comma-separated setting; empty or "none" means none."""
    value = carb.settings.get_settings().get(f"{_SETTINGS}/obstacles") or ""
    kinds = (kind.strip() for kind in value.split(","))
    return list(dict.fromkeys(kind for kind in kinds if kind and kind != "none"))


class Extension(omni.ext.IExt):
    """Kit instantiates this class when oc.scenario.warehouse is enabled; with autostart, it opens the Scenario and,
    with autoplay, plays it."""

    def on_startup(self, ext_id: str):
        carb.log_info(f"on_startup {ext_id}")
        self._window = WarehouseWindow()
        self._setup_task = None
        self._objects = None
        if carb.settings.get_settings().get(f"{_SETTINGS}/autostart"):
            self._setup_task = asyncio.ensure_future(self.setup_scene())

    async def setup_scene(self):
        """Open the warehouse stage, spawn the G1, show the obstacles, set the physics rate and, with autoplay, play."""
        app = omni.kit.app.get_app()
        # Mutating the stage before the app is ready races the renderer.
        while not app.is_app_ready():
            await app_utils.update_app_async()

        path = _warehouse_usd_path()
        success, stage = await stage_utils.open_stage_async(path)
        if not success or stage is None:
            carb.log_error(f"setup_scene: could not open {path}")
            return
        await app_utils.update_app_async()

        app_utils.enable_extension(_GANTRY_EXTENSION, enabled=False)
        ros = _ros_setting()
        try:
            # The session layer takes the edits, so saving never changes warehouse.usda.
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                G1.spawn(_G1_PATH, _G1_TRANSLATE, _G1_ORIENT, ros=ros)
                clock = stage.GetPrimAtPath(_CLOCK_GRAPH)
                if clock:
                    clock.SetActive(ros)
                else:
                    carb.log_warn(f"setup_scene: no clock graph at {_CLOCK_GRAPH}")
                scene = stage.GetPrimAtPath(_PHYSICS_SCENE)
                for name in _TIME_STEP_ATTRIBUTES:
                    scene.GetAttribute(name).Set(_PHYSICS_RATE)
            self._objects = ObjectManager(stage)
            self._objects.show(_obstacles_setting())
        except Exception as e:
            carb.log_error(f"setup_scene: failed to prepare the scenario: {e}")
            return
        if _autoplay_setting():
            app_utils.play()

    def on_shutdown(self):
        carb.log_info("on_shutdown")
        if self._setup_task is not None:
            self._setup_task.cancel()  # no-op if already done
            self._setup_task = None
        self._objects = None
        self._window.destroy()
        self._window = None
        gc.collect()
