"""Sequence executor: steps through a Behavior sequence's Segments from the robot's pose, steering along each one,
deciding when it ends, timing it out, looping and halting. Like a Nav2 behavior, each segment both commands the
velocity and judges its own completion."""

import enum
import math
from dataclasses import dataclass, replace

from .behavior_sequence import MAX_SIDE_SPEED, BehaviorSequence, Pose, Segment
from .locomotion_command import LocomotionCommand

POSITION_TOLERANCE = 0.15  # m: a drive or side step ends this far before its target, along the segment
YAW_TOLERANCE = math.radians(5.0)  # rad
TURN_RATE = 0.5  # rad/s: nominal turn rate, for timeouts
MIN_NOMINAL_SPEED = 0.2  # m/s: slowest speed assumed for timeouts
TIMEOUT_FACTOR = 2.0  # a segment fails after TIMEOUT_FACTOR x its nominal duration + TIMEOUT_MARGIN
TIMEOUT_MARGIN = 3.0  # s
K_YAW = 2.0  # 1/s: heading error -> turn rate
MAX_TURN_RATE = 1.5  # rad/s
KP_CROSS_TRACK = 1.0  # 1/m: sideways distance from the segment -> direction correction
MAX_CROSS_TRACK = 0.5  # cap of that correction (0.5 ~ 27 deg)
MOVES = ("drive", "side_step")  # segments that travel along a line


class MovementStatus(enum.Enum):
    """Status of the controller, with Nav2's TaskResult names for how a command ends."""

    IDLE = "idle"  # standing, no command
    RUNNING = "running"  # following a Behavior sequence or a real-time command
    SUCCEEDED = "succeeded"  # reached the end of a Behavior sequence
    ABORTED = "aborted"  # a segment failed; see failure
    CANCELED = "canceled"  # replaced by another command or halted


@dataclass(frozen=True)
class MovementFailure:
    """Why a Behavior sequence was aborted ("timeout") and at which segment index."""

    reason: str
    index: int


def wrap(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


def to_world(pose: Pose, anchor: Pose) -> Pose:
    """A pose given in the frame of the anchor pose, in the world."""
    ax, ay, ayaw = anchor
    c, s = math.cos(ayaw), math.sin(ayaw)
    return ax + c * pose[0] - s * pose[1], ay + s * pose[0] + c * pose[1], ayaw + pose[2]


def profile_speed(segment: Segment, remaining: float) -> float:
    """Speed [m/s] of a drive remaining meters before its target: cruise, slowing to exit_speed at the target at the
    segment's deceleration."""
    return min(segment.speed, math.sqrt(segment.exit_speed**2 + 2.0 * segment.decel * max(remaining, 0.0)))


def _line(segment: Segment, pose: Pose) -> tuple[float, float, float, float, float]:
    """(ux, uy, length, done, lateral): the direction and length of the line from segment's start to its target, and
    how far pose is along it and left of it."""
    sx, sy, _ = segment.start
    gx, gy, gyaw = segment.target
    length = math.hypot(gx - sx, gy - sy)
    ux, uy = ((gx - sx) / length, (gy - sy) / length) if length > 1e-6 else (math.cos(gyaw), math.sin(gyaw))
    done = (pose[0] - sx) * ux + (pose[1] - sy) * uy
    lateral = -(pose[0] - sx) * uy + (pose[1] - sy) * ux
    return ux, uy, length, done, lateral


def _steer(segment: Segment, pose: Pose) -> LocomotionCommand:
    """The command for the robot at pose on segment: along the line from its start to its target, steering back onto
    it, at the speed profile; holding or turning to the target heading."""
    yaw = pose[2]
    wz = clamp(K_YAW * wrap(segment.target[2] - yaw), -MAX_TURN_RATE, MAX_TURN_RATE)
    if segment.kind not in MOVES:
        return LocomotionCommand(wz=wz, gait=segment.gait)
    ux, uy, length, done, lateral = _line(segment, pose)
    correction = clamp(-KP_CROSS_TRACK * lateral, -MAX_CROSS_TRACK, MAX_CROSS_TRACK)
    movement = math.atan2(uy + correction * ux, ux - correction * uy)
    if segment.kind == "side_step":
        speed = min(segment.speed, MAX_SIDE_SPEED)
    else:
        speed = profile_speed(segment, length - done)
    return LocomotionCommand(speed * math.cos(movement - yaw), speed * math.sin(movement - yaw), wz, segment.gait)


class SequenceExecutor:
    """Runs one Behavior sequence at a time, anchored at the robot's pose on the first update after execute.
    on_done(status) is called when the sequence ends: SUCCEEDED, ABORTED or CANCELED."""

    def __init__(self, on_done=None) -> None:
        self.on_done = on_done
        self.status = MovementStatus.IDLE
        self.failure = None
        self._sequence = None
        self._segments = ()
        self._pending = False
        self._anchor = (0.0, 0.0, 0.0)
        self._index = 0
        self._segment = None  # the active segment, world frame
        self._deadline = math.inf
        self._arrived_at = None

    @property
    def sequence(self) -> BehaviorSequence | None:
        return self._sequence

    @property
    def index(self) -> int:
        """Index of the active segment."""
        return self._index

    def execute(self, sequence: BehaviorSequence) -> None:
        """Run sequence from its first segment, starting at the next update; cancels the running sequence."""
        segments = sequence.segments  # raises on an empty sequence
        self.cancel()
        self._sequence, self._segments = sequence, segments
        self._pending = True

    def restart(self) -> None:
        """Run the current sequence again from its first segment, anchored at the next update."""
        if self._sequence is not None:
            self._pending = True

    def cancel(self) -> None:
        """Drop the sequence; a running one ends CANCELED."""
        if self.status == MovementStatus.RUNNING and self._sequence is not None:
            self._finish(MovementStatus.CANCELED)
        self._sequence, self._segments = None, ()
        self._pending = False

    def update(self, pose: Pose, t: float) -> LocomotionCommand | None:
        """The Locomotion command, with the active segment's gait, for the robot at pose at time t [s], moving on to
        the next segment when one ends; None when no sequence is running."""
        if self._pending:
            self._pending = False
            self._anchor = pose
            self.status = MovementStatus.RUNNING
            self.failure = None
            self._start(0, pose, t)
        if self.status != MovementStatus.RUNNING or self._sequence is None:
            return None
        if self._reached(self._segment, pose, t):
            index = self._index + 1
            if index == len(self._segments):
                if not self._sequence.repeats:
                    self._finish(MovementStatus.SUCCEEDED)
                    return None
                index = 0
            self._start(index, pose, t)
        elif t > self._deadline:
            self.failure = MovementFailure("timeout", self._index)
            self._finish(MovementStatus.ABORTED)
            return None
        return _steer(self._segment, pose)

    def _start(self, index: int, pose: Pose, t: float) -> None:
        self._index = index
        self._arrived_at = None
        segment = self._segments[index]
        self._segment = replace(segment, start=to_world(segment.start, self._anchor), target=to_world(segment.target, self._anchor))
        gx, gy, gyaw = self._segment.target
        length = math.hypot(gx - pose[0], gy - pose[1])
        speed = segment.speed if segment.kind in MOVES else 1.0
        nominal = length / max(speed, MIN_NOMINAL_SPEED) + abs(wrap(gyaw - pose[2])) / TURN_RATE + segment.hold
        self._deadline = math.inf if segment.forever else t + TIMEOUT_FACTOR * nominal + TIMEOUT_MARGIN

    def _reached(self, segment: Segment, pose: Pose, t: float) -> bool:
        if segment.kind in MOVES:
            _, _, length, done, _ = _line(segment, pose)
            return length - done < POSITION_TOLERANCE
        if segment.kind == "spin":
            return abs(wrap(segment.target[2] - pose[2])) < YAW_TOLERANCE
        if segment.forever:  # wait(): the sequence ends here and the robot stands
            return True
        if self._arrived_at is None:  # wait(seconds)
            self._arrived_at = t
        return t - self._arrived_at >= segment.hold

    def _finish(self, status: MovementStatus) -> None:
        """End the sequence with status."""
        self.status = status
        if self.on_done:
            self.on_done(status)
