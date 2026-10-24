import carb
import omni.ext


class Extension(omni.ext.IExt):
    """Kit instantiates this class when oc.robot.g1 is enabled."""

    def on_startup(self, ext_id: str):
        carb.log_info(f"[{ext_id}] startup")

    def on_shutdown(self):
        carb.log_info("[oc.robot.g1] shutdown")
