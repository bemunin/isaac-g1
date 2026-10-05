import carb
import omni.ext


class Extension(omni.ext.IExt):
    """Kit instantiates this class when oc.control.g1_walk is enabled.

    Provides G1Walker; enabling the extension does not make any robot walk.
    """

    def on_startup(self, ext_id: str):
        carb.log_info(f"[{ext_id}] startup")

    def on_shutdown(self):
        carb.log_info("[oc.control.g1_walk] shutdown")
