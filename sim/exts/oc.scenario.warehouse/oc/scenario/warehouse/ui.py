from omni import ui


class WarehouseWindow(ui.Window):
    """Example window with a click counter."""

    def __init__(self):
        super().__init__("Warehouse", width=300, height=120)
        # Tab beside Property in the right panel, with this tab selected.
        self.deferred_dock_in("Property", ui.DockPolicy.CURRENT_WINDOW_IS_ACTIVE)
