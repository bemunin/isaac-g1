from omni import ui


class RosDdsWindow(ui.Window):
    """Empty window, docked as a tab next to Property."""

    def __init__(self):
        super().__init__("ROS DDS Benchmark", width=300, height=120)
        self.deferred_dock_in("Property", ui.DockPolicy.DO_NOTHING)
