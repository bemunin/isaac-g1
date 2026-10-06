from isaacsim.gui.components import CheckBox, CollapsableFrame, ScrollingWindow
from omni import ui

from .visualizer import WalkTrajectoryVisualizer

TITLE = "G1 Walk"


class G1WalkWindow(ScrollingWindow):
    """G1 Walk window, docked as a tab next to Property: Visualizer > Show Walk Trajectory."""

    def __init__(self):
        super().__init__(title=TITLE, width=300, height=120)
        self.deferred_dock_in("Property", ui.DockPolicy.DO_NOTHING)
        self._visualizer = WalkTrajectoryVisualizer()
        with self.frame, ui.VStack(height=0, spacing=4):
            self._visualizer_frame = CollapsableFrame("Visualizer", collapsed=False)
            with self._visualizer_frame.frame:
                self._show_trajectory = CheckBox(
                    "Show Walk Trajectory",
                    default_value=False,
                    tooltip="Draw each started G1Walker's Walk trajectory on the ground",
                    on_click_fn=self._visualizer.show,
                )

    def destroy(self):
        self._visualizer.destroy()
        super().destroy()
