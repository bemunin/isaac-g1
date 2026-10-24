import asyncio
import gc

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.core.experimental.utils.transform as transform_utils
import omni.ext
import omni.kit.app
from isaacsim.core.experimental.prims import XformPrim
from oc.robot.g1 import G1_USD
from pxr import Usd

from .ui import WarehouseWindow

_G1_PATH = "/World/G1"
# G1 spawn pose from the old warehouse_g1_scene.usda.
# Orientation is roll, pitch, yaw in degrees (extrinsic XYZ).
_G1_TRANSLATE = (-8.0, 9.5, 0.8)
_G1_ORIENT = (0.0, 0.0, 90.0)


def _add_g1(stage: Usd.Stage) -> None:
    """Reference the G1 at its spawn pose on the session layer."""
    orientation = transform_utils.euler_angles_to_quaternion(_G1_ORIENT, degrees=True)
    # Session layer so saving never writes the robot into warehouse.usda.
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        stage_utils.add_reference_to_stage(str(G1_USD), _G1_PATH, prim_type="Xform")
        XformPrim(
            _G1_PATH,
            translations=_G1_TRANSLATE,
            orientations=orientation,
            reset_xform_op_properties=True,
        )


class Extension(omni.ext.IExt):
    """Kit instantiates this class when oc.scenario.warehouse is enabled."""

    def on_startup(self, ext_id: str):
        carb.log_info(f"on_startup {ext_id}")
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        self._scene_path = f"{ext_manager.get_extension_path(ext_id)}/usd/warehouse.usda"
        self._window = WarehouseWindow()
        self._setup_task = asyncio.ensure_future(self.setup_scene())

    async def setup_scene(self):
        """Open the warehouse stage and reference the G1 robot at its spawn pose."""
        app = omni.kit.app.get_app()
        # Mutating the stage before the app is ready races the renderer.
        while not app.is_app_ready():
            await app_utils.update_app_async()

        success, stage = await stage_utils.open_stage_async(self._scene_path)
        if not success or stage is None:
            carb.log_error(f"setup_scene: could not open {self._scene_path}")
            return
        await app_utils.update_app_async()

        try:
            _add_g1(stage)
        except Exception as e:
            carb.log_error(f"setup_scene: failed to add {_G1_PATH}: {e}")
            return
        carb.log_info(f"setup_scene: added {_G1_PATH} from {G1_USD}")

    def on_shutdown(self):
        carb.log_info("on_shutdown")
        self._setup_task.cancel()  # no-op if already done
        self._setup_task = None
        self._window.destroy()
        self._window = None
        gc.collect()
