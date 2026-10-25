from oc.control.g1_sonic import SonicController

from . import extension

__all__ = ["get_sonic_controller"]


def get_sonic_controller() -> SonicController | None:
    """The running warehouse's SONIC controller; None when SONIC is off or not created.

    controller = get_sonic_controller()
    if controller is not None:
        controller.command(SonicMode.WALK, vx=0.5)
    """
    return None if extension._instance is None else extension._instance._sonic
