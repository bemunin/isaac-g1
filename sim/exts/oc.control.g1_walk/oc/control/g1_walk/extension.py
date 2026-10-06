import carb
import omni.ext
import omni.kit.actions.core
from isaacsim.gui.components.menu import make_menu_item_description
from omni.kit.menu.utils import add_menu_items, remove_menu_items

from .ui import TITLE, G1WalkWindow


class Extension(omni.ext.IExt):
    """Kit instantiates this class when oc.control.g1_walk is enabled.

    Provides G1Walker and the G1 Walk window (Window > G1 Walk); enabling the extension does not
    make any robot walk.
    """

    def on_startup(self, ext_id: str):
        carb.log_info(f"[{ext_id}] startup")
        self._ext_id = ext_id
        self._window = G1WalkWindow()
        self._menu_items = [make_menu_item_description(ext_id, TITLE, self._toggle_window)]
        add_menu_items(self._menu_items, "Window")

    def on_shutdown(self):
        remove_menu_items(self._menu_items, "Window")
        omni.kit.actions.core.get_action_registry().deregister_all_actions_for_extension(self._ext_id)
        self._window.destroy()
        self._window = None
        carb.log_info("[oc.control.g1_walk] shutdown")

    def _toggle_window(self) -> None:
        self._window.visible = not self._window.visible
