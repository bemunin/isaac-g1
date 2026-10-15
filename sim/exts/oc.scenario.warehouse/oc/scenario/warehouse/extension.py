import carb
import carb.settings
import omni.ext
import omni.kit.app
import omni.timeline
from carb.eventdispatcher import get_eventdispatcher
from oc.control.g1_sonic import G1Sonic

from .api import open_scenario, ros
from .cmd_vel import CmdVelBridge


class Extension(omni.ext.IExt):
    """Kit instantiates this class when oc.scenario.warehouse is enabled.

    With the autostart setting on, opens the Scenario and plays once the app is ready.
    Whenever the simulation plays, /World/G1 walks with SONIC whole-body control. With ROS (default), it follows
    /cmd_vel, e.g. from Nav2 or teleop; without, it stands.
    """

    def on_startup(self, ext_id: str):
        carb.log_info(f"[{ext_id}] startup")
        self._walker = G1Sonic("/World/G1")
        self._walker.start()
        self._cmd_vel = CmdVelBridge(self._walker.command) if ros() else None
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
        if self._cmd_vel:
            self._cmd_vel.destroy()
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
