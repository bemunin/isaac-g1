"""Walk trajectory visualizer: draws the started G1Walkers' Walk trajectories on the ground with debug_draw."""

import math

import omni.kit.app
from isaacsim.util.debug_draw import _debug_draw

from .api import walkers

Z = 0.02  # m: draw just above flat ground
HEADING_LENGTH = 0.2  # m
LINE_COLOR = (0.1, 0.8, 1.0, 1.0)
GOAL_COLOR = (1.0, 0.6, 0.1, 1.0)
LINE_WIDTH = 2.0
POINT_SIZE = 10.0


class WalkTrajectoryVisualizer:
    """While shown, redraws every started G1Walker's Walk trajectory whenever one changes."""

    def __init__(self) -> None:
        self._draw = _debug_draw.acquire_debug_draw_interface()
        self._update_subscription = None
        self._drawn = None

    def show(self, visible: bool) -> None:
        if visible and self._update_subscription is None:
            stream = omni.kit.app.get_app().get_update_event_stream()
            self._update_subscription = stream.create_subscription_to_pop(self._on_update, name="oc.control.g1_walk.visualizer")
        elif not visible and self._update_subscription is not None:
            self._update_subscription = None
            self._drawn = None
            self._draw.clear_lines()
            self._draw.clear_points()

    def destroy(self) -> None:
        self.show(False)

    def _on_update(self, event) -> None:
        paths = tuple((walker.world_path(), walker.trajectory.loop if walker.trajectory else False) for walker in walkers())
        if paths == self._drawn:
            return
        self._drawn = paths
        self._draw.clear_lines()
        self._draw.clear_points()
        starts, ends, goals = [], [], []
        for path, loop in paths:
            if not path:
                continue
            points = [(x, y, Z) for x, y, _ in path]
            starts += points[:-1]
            ends += points[1:]
            if loop and len(points) > 2:  # back from the last goal to the first
                starts.append(points[-1])
                ends.append(points[1])
            for x, y, yaw in path[1:]:  # heading ticks
                starts.append((x, y, Z))
                ends.append((x + HEADING_LENGTH * math.cos(yaw), y + HEADING_LENGTH * math.sin(yaw), Z))
            goals += points[1:]
        if starts:
            self._draw.draw_lines(starts, ends, [LINE_COLOR] * len(starts), [LINE_WIDTH] * len(starts))
        if goals:
            self._draw.draw_points(goals, [GOAL_COLOR] * len(goals), [POINT_SIZE] * len(goals))
