import carb
import omni.ext

from .ui import G1Window


class Extension(omni.ext.IExt):
    """Kit instantiates this class when oc.robot.g1 is enabled."""

    def on_startup(self, ext_id: str):
        carb.log_info(f"[{ext_id}] startup")
        self._window = G1Window()

    def on_shutdown(self):
        self._window.destroy()
        self._window = None
        carb.log_info("[oc.robot.g1] shutdown")
