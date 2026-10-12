"""Behavior sequence: an ordered, unchangeable list of Segments, built in code from behaviors named after Nav2's
(DriveOnHeading, Spin, Wait), each forward move with a gait.

Each behavior method returns a new sequence. Consecutive forward moves carry their speed into the next one (the lower
of the two cruise speeds) and otherwise slow down to a stop; the Sequence executor does the slowing down.
"""

import math
from dataclasses import dataclass, field, replace

from .locomotion_command import Gait

# Speeds of SONIC's planner modes (docs/source/tutorials/keyboard.md, gamepad.md): slow walk 0.2-0.8 m/s,
# walk 0.8-1.5 m/s, run 1.5-3.0 m/s; side steps at up to ~0.4 m/s, or the feet collide. Below 0.4 m/s a slow walk
# steps in place.
SLOW_WALK_SPEED, MIN_SLOW_WALK_SPEED, MAX_SLOW_WALK_SPEED = 0.6, 0.4, 0.8  # m/s
WALK_SPEED, MIN_WALK_SPEED, MAX_WALK_SPEED = 1.0, 0.8, 1.5  # m/s
RUN_SPEED, MIN_RUN_SPEED, MAX_RUN_SPEED = 2.0, 1.5, 3.0  # m/s
SIDE_SPEED, MAX_SIDE_SPEED = 0.3, 0.4  # m/s
WALK_DECEL, RUN_DECEL = 1.0, 1.5  # m/s^2
SPEED_RANGES = {
    Gait.SLOW_WALK: (MIN_SLOW_WALK_SPEED, MAX_SLOW_WALK_SPEED),
    Gait.WALK: (MIN_WALK_SPEED, MAX_WALK_SPEED),
    Gait.RUN: (MIN_RUN_SPEED, MAX_RUN_SPEED),
}

Pose = tuple[float, float, float]  # x [m], y [m], yaw [rad]


def gait_mode(speed: float) -> Gait:
    """SONIC planner mode for a speed [m/s]."""
    return Gait.RUN if speed >= MIN_RUN_SPEED else Gait.WALK if speed > MAX_SLOW_WALK_SPEED else Gait.SLOW_WALK


@dataclass(frozen=True)
class Segment:
    """One behavior: from the start pose to the target pose (relative to the sequence's start, or in the world once a
    Sequence executor anchors it).

    kind: "drive" (forward, at gait), "side_step", "spin" (in place) or "wait" (stand for hold seconds, or until the
    next command if forever). gait is the fastest planner mode the segment may use. Drives and side steps travel at
    speed [m/s]; drives slow down to exit_speed at the target.
    """

    kind: str
    start: Pose
    target: Pose
    gait: Gait = Gait.IDLE
    speed: float = 0.0
    exit_speed: float = 0.0
    hold: float = 0.0
    forever: bool = False

    @property
    def decel(self) -> float:
        return RUN_DECEL if self.gait == Gait.RUN else WALK_DECEL


@dataclass(frozen=True)
class BehaviorSequence:
    """Behaviors in order; each method returns a new sequence, so calls chain.

    Behaviors are relative to where the previous one left the robot. Drives go forward only; spins and side steps
    follow REP-103 (positive: counterclockwise, left). wait(seconds) stands for seconds; wait() stands until the next
    command and ends the sequence, so nothing can follow it and a looping sequence doesn't repeat. A looping sequence
    without wait() runs until G1Wbc.halt() or another command.
    """

    loop: bool = False
    _behaviors: tuple[Segment, ...] = field(default=(), repr=False)

    @property
    def repeats(self) -> bool:
        """Whether the sequence starts over after its last segment."""
        return self.loop and not self._stopped

    @property
    def segments(self) -> tuple[Segment, ...]:
        """The segments, each drive with its exit speed."""
        if not self._behaviors:
            raise ValueError("BehaviorSequence: the sequence is empty")
        segments = self._behaviors
        count = len(segments)
        result = []
        for i, segment in enumerate(segments):
            following = segments[(i + 1) % count] if self.repeats or i + 1 < count else None
            carries_on = segment.kind == "drive" and following is not None and following.kind == "drive"
            result.append(replace(segment, exit_speed=min(segment.speed, following.speed) if carries_on else 0.0))
        return tuple(result)

    def drive_on_heading(self, meters: float, speed: float, gait: Gait | None = None) -> "BehaviorSequence":
        """Move forward meters at speed [m/s] (Nav2 DriveOnHeading), at gait, or the gait for speed if None."""
        if gait is None:
            _check_range("speed", speed, MIN_SLOW_WALK_SPEED, MAX_RUN_SPEED)
            gait = gait_mode(speed)
        elif gait not in SPEED_RANGES:
            raise ValueError(f"BehaviorSequence: gait must be SLOW_WALK, WALK or RUN, got {gait!r}")
        else:
            _check_range("speed", speed, *SPEED_RANGES[gait])
        return self._move("drive", meters, 0.0, speed, gait)

    def slow_walk(self, meters: float, speed: float = SLOW_WALK_SPEED) -> "BehaviorSequence":
        """Slow-walk forward meters at speed [m/s]."""
        return self.drive_on_heading(meters, speed, Gait.SLOW_WALK)

    def walk(self, meters: float, speed: float = WALK_SPEED) -> "BehaviorSequence":
        """Walk forward meters at speed [m/s]."""
        return self.drive_on_heading(meters, speed, Gait.WALK)

    def run(self, meters: float, speed: float = RUN_SPEED) -> "BehaviorSequence":
        """Run forward meters at speed [m/s]."""
        return self.drive_on_heading(meters, speed, Gait.RUN)

    def side_step(self, meters: float, speed: float = SIDE_SPEED) -> "BehaviorSequence":
        """Step sideways meters (positive: left) at speed [m/s], keeping the heading."""
        _check_nonzero("meters", meters)
        _check_range("speed", speed, 0.0, MAX_SIDE_SPEED, low_open=True)
        return self._move("side_step", abs(meters), math.copysign(math.pi / 2, meters), speed, Gait.SLOW_WALK)

    def spin(self, degrees: float) -> "BehaviorSequence":
        """Turn in place by degrees (Nav2 Spin; positive: counterclockwise, left)."""
        _check_nonzero("degrees", degrees)
        x, y, yaw = start = self._end
        return self._add(Segment("spin", start, (x, y, yaw + math.radians(degrees))))

    def wait(self, seconds: float | None = None) -> "BehaviorSequence":
        """Slow down and stand for seconds (Nav2 Wait), or with no seconds until the next command, ending the
        sequence."""
        if seconds is not None:
            check_positive("seconds", seconds)
        pose = self._end
        return self._add(Segment("wait", pose, pose, hold=seconds or 0.0, forever=seconds is None))

    @property
    def _end(self) -> Pose:
        return self._behaviors[-1].target if self._behaviors else (0.0, 0.0, 0.0)

    @property
    def _stopped(self) -> bool:
        return bool(self._behaviors) and self._behaviors[-1].forever

    def _move(self, kind: str, meters: float, direction: float, speed: float, gait: Gait) -> "BehaviorSequence":
        """Move meters along the robot's heading rotated by direction, keeping the heading."""
        check_positive("meters", meters)
        x, y, yaw = start = self._end
        target = (x + meters * math.cos(yaw + direction), y + meters * math.sin(yaw + direction), yaw)
        return self._add(Segment(kind, start, target, gait=gait, speed=speed))

    def _add(self, segment: Segment) -> "BehaviorSequence":
        if self._stopped:
            raise ValueError("BehaviorSequence: nothing can follow wait() without seconds")
        return replace(self, _behaviors=self._behaviors + (segment,))


def check_positive(name: str, value: float) -> None:
    if not value > 0:
        raise ValueError(f"BehaviorSequence: {name} must be positive, got {value}")


def _check_nonzero(name: str, value: float) -> None:
    if value == 0 or not math.isfinite(value):
        raise ValueError(f"BehaviorSequence: {name} must be nonzero, got {value}")


def _check_range(name: str, value: float, low: float, high: float, low_open: bool = False) -> None:
    if not (low < value if low_open else low <= value) or not value <= high:
        raise ValueError(f"BehaviorSequence: {name} must be in {'(' if low_open else '['}{low}, {high}], got {value}")
