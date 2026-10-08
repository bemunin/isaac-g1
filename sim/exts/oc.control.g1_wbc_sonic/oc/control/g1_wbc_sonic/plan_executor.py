"""Plan executor: steps through a Movement plan's Segments from the robot's pose, deciding when each ends, timing them
out, looping and halting."""

import enum
import math
from dataclasses import dataclass, replace

from .movement_plan import FORWARD_KINDS, MovementPlan, Pose, Segment
from .path_tracker import wrap

POSITION_TOLERANCE = 0.15  # m: a forward or side move ends this far before its target, along the segment
YAW_TOLERANCE = math.radians(5.0)  # rad
TURN_RATE = 0.5  # rad/s: nominal turn rate, for timeouts
MIN_NOMINAL_SPEED = 0.2  # m/s: slowest speed assumed for timeouts
TIMEOUT_FACTOR = 2.0  # a segment fails after TIMEOUT_FACTOR x its nominal duration + TIMEOUT_MARGIN
TIMEOUT_MARGIN = 3.0  # s


class MovementStatus(enum.Enum):
    IDLE = "idle"  # standing, no command
    RUNNING = "running"  # following a Movement plan or a real-time command
    DONE = "done"  # reached the end of a Movement plan
    FAILED = "failed"  # a segment failed; see failure
    CANCELLED = "cancelled"  # replaced by another command or halted


@dataclass(frozen=True)
class MovementFailure:
    """Why a Movement plan failed ("timeout") and at which segment index."""

    reason: str
    index: int


def to_world(pose: Pose, anchor: Pose) -> Pose:
    """A pose given in the frame of the anchor pose, in the world."""
    ax, ay, ayaw = anchor
    c, s = math.cos(ayaw), math.sin(ayaw)
    return ax + c * pose[0] - s * pose[1], ay + s * pose[0] + c * pose[1], ayaw + pose[2]


class PlanExecutor:
    """Runs one Movement plan at a time, anchored at the robot's pose on the first update after execute. on_done(status)
    is called when the plan ends: DONE, FAILED or CANCELLED."""

    def __init__(self, on_done=None) -> None:
        self.on_done = on_done
        self.status = MovementStatus.IDLE
        self.failure = None
        self._plan = None
        self._segments = ()
        self._pending = False
        self._anchor = (0.0, 0.0, 0.0)
        self._index = 0
        self._segment = None  # the active segment, world frame
        self._deadline = math.inf
        self._arrived_at = None

    @property
    def plan(self) -> MovementPlan | None:
        return self._plan

    @property
    def index(self) -> int:
        """Index of the active segment."""
        return self._index

    def execute(self, plan: MovementPlan) -> None:
        """Run plan from its first segment, starting at the next update; cancels the running plan."""
        segments = plan.segments  # raises on an empty plan
        self.cancel()
        self._plan, self._segments = plan, segments
        self._pending = True

    def restart(self) -> None:
        """Run the current plan again from its first segment, anchored at the next update."""
        if self._plan is not None:
            self._pending = True

    def cancel(self) -> None:
        """Drop the plan; a running one ends CANCELLED."""
        if self.status == MovementStatus.RUNNING and self._plan is not None:
            self._finish(MovementStatus.CANCELLED)
        self._plan, self._segments = None, ()
        self._pending = False

    def update(self, pose: Pose, t: float) -> Segment | None:
        """The active segment (world frame) for the robot at pose at time t [s], moving on when it ends; None when no
        plan is running."""
        if self._pending:
            self._pending = False
            self._anchor = pose
            self.status = MovementStatus.RUNNING
            self.failure = None
            self._start(0, pose, t)
        if self.status != MovementStatus.RUNNING or self._plan is None:
            return None
        if self._reached(self._segment, pose, t):
            index = self._index + 1
            if index == len(self._segments):
                if not self._plan.repeats:
                    self._finish(MovementStatus.DONE)
                    return None
                index = 0
            self._start(index, pose, t)
        elif t > self._deadline:
            self.failure = MovementFailure("timeout", self._index)
            self._finish(MovementStatus.FAILED)
            return None
        return self._segment

    def _start(self, index: int, pose: Pose, t: float) -> None:
        self._index = index
        self._arrived_at = None
        segment = self._segments[index]
        self._segment = replace(segment, start=to_world(segment.start, self._anchor), target=to_world(segment.target, self._anchor))
        gx, gy, gyaw = self._segment.target
        length = math.hypot(gx - pose[0], gy - pose[1])
        speed = segment.speed if segment.kind in FORWARD_KINDS or segment.kind == "side" else 1.0
        nominal = length / max(speed, MIN_NOMINAL_SPEED) + abs(wrap(gyaw - pose[2])) / TURN_RATE + segment.hold
        self._deadline = math.inf if segment.forever else t + TIMEOUT_FACTOR * nominal + TIMEOUT_MARGIN

    def _reached(self, segment: Segment, pose: Pose, t: float) -> bool:
        gx, gy, gyaw = segment.target
        if segment.kind in FORWARD_KINDS or segment.kind == "side":
            sx, sy, _ = segment.start
            length = math.hypot(gx - sx, gy - sy)
            remaining = length - ((pose[0] - sx) * (gx - sx) + (pose[1] - sy) * (gy - sy)) / max(length, 1e-6)
            return remaining < POSITION_TOLERANCE
        if segment.kind == "turn":
            return abs(wrap(gyaw - pose[2])) < YAW_TOLERANCE
        if segment.forever:  # stop(): the plan ends here and the robot stands
            return True
        if self._arrived_at is None:  # stop(seconds)
            self._arrived_at = t
        return t - self._arrived_at >= segment.hold

    def _finish(self, status: MovementStatus) -> None:
        """End the plan with status."""
        self.status = status
        if self.on_done:
            self.on_done(status)
