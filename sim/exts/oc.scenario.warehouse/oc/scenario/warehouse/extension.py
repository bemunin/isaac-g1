import carb
import carb.settings
import omni.ext
import omni.kit.app
import omni.timeline
from carb.eventdispatcher import get_eventdispatcher
from oc.control.g1_sonic import G1Wbc

from .api import open_scenario
from .cmd_vel import CmdVelPoller
from .warehouse_sequence import WAREHOUSE_SEQUENCE


class Extension(omni.ext.IExt):
    """Kit instantiates this class when oc.scenario.warehouse is enabled.

    With the autostart setting on, opens the Scenario and plays once the app is ready.
    Whenever the simulation plays, /World/G1 follows WAREHOUSE_SEQUENCE with SONIC whole-body control. With ros(), a
    /cmd_vel Twist takes over from the sequence (Real-time control) until the next Play.
    """

    def on_startup(self, ext_id: str):
        carb.log_info(f"[{ext_id}] startup")
        self._walker = G1Wbc("/World/G1")
        self._walker.execute(WAREHOUSE_SEQUENCE)
        self._walker.start()
        self._cmd_vel = CmdVelPoller(self._walker)
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
        self._cmd_vel.stop()
        self._cmd_vel = None
        self._walker.stop()
        self._walker = None
        carb.log_info("[oc.scenario.warehouse] shutdown")


def open_and_play() -> None:
    try:
        open_scenario()
    except RuntimeError as error:
        carb.log_error(str(error))
        return
    omni.timeline.get_timeline_interface().play()
