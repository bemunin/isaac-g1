import carb
import carb.settings
import omni.ext
import omni.kit.app
import omni.timeline
from carb.eventdispatcher import get_eventdispatcher

from .api import open_scenario
from .g1_drive import G1Drive


class Extension(omni.ext.IExt):
    """Kit instantiates this class when oc.scenario.warehouse is enabled.

    With the autostart setting on, opens the Scenario and plays once the app is ready.
    Whenever the simulation plays, /World/G1 is set up for SONIC's joint targets (G1Drive); the oc_sonic ROS 2 nodes
    drive it over /joint_states and /joint_commands.
    """

    def on_startup(self, ext_id: str):
        carb.log_info(f"[{ext_id}] startup")
        self._drive = G1Drive("/World/G1")
        self._drive.start()
        self._app_ready_sub = None
        if carb.settings.get_settings().get("/exts/oc.scenario.warehouse/autostart"):
            if omni.kit.app.get_app().is_app_ready():
                open_and_play()
            else:
                self._app_ready_sub = get_eventdispatcher().observe_event(
                    event_name=omni.kit.app.GLOBAL_EVENT_APP_READY,
                    on_event=lambda _: open_and_play(),
                    observer_name="oc.scenario.warehouse",
                )

    def on_shutdown(self):
        self._app_ready_sub = None
        self._drive.stop()
        self._drive = None
        carb.log_info("[oc.scenario.warehouse] shutdown")


def open_and_play() -> None:
    try:
        open_scenario()
    except RuntimeError as error:
        carb.log_error(str(error))
        return
    omni.timeline.get_timeline_interface().play()
