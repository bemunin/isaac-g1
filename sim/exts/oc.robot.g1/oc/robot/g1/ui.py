from omni import ui


class G1Window(ui.Window):
    """Example window with a click counter."""

    def __init__(self):
        super().__init__("G1 Robot", width=300, height=120)
