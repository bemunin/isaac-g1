import carb
import carb.settings
import omni.ext
import omni.kit.app
import omni.timeline
from carb.eventdispatcher import get_eventdispatcher
from oc.control.g1_walk import G1Walker

from .api import open_scenario
from .simple_walk_sequence import SIMPLE_WALK_SEQUENCE
from .ui import RosDdsWindow


class Extension(omni.ext.IExt):
    """Kit instantiates this class when oc.benchmark.ros_dds is enabled.

    With the autostart setting on, opens the Scenario and plays once the app is ready.
    Makes /World/G1 walk SIMPLE_WALK_SEQUENCE whenever the simulation plays.
    """

    def on_startup(self, ext_id: str):
        carb.log_info(f"[{ext_id}] startup")
        self._window = RosDdsWindow()
        self._walker = G1Walker("/World/G1", SIMPLE_WALK_SEQUENCE)
        self._walker.start()
        self._app_ready_sub = None
        if carb.settings.get_settings().get("/exts/oc.benchmark.ros_dds/autostart"):
            if omni.kit.app.get_app().is_app_ready():
                open_and_play()
            else:
                self._app_ready_sub = get_eventdispatcher().observe_event(
                    event_name=omni.kit.app.GLOBAL_EVENT_APP_READY,
                    on_event=lambda _: open_and_play(),
                    observer_name="oc.benchmark.ros_dds",
                )

    def on_shutdown(self):
        self._app_ready_sub = None
        self._walker.stop()
        self._walker = None
        self._window.destroy()
        self._window = None
        carb.log_info("[oc.benchmark.ros_dds] shutdown")


def open_and_play() -> None:
    try:
        open_scenario()
    except RuntimeError as error:
        carb.log_error(str(error))
        return
    omni.timeline.get_timeline_interface().play()
