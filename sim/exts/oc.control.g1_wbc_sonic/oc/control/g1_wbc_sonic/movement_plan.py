"""Movement plan: an ordered, unchangeable list of Segments, built from Movement primitives fixed in code.

Each primitive method returns a new plan. Slow walks, walks and runs carry their speed into a following one (the
lower of the two cruise speeds) and otherwise slow down to a stop; the Path tracker does the slowing down.
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
FORWARD_KINDS = ("slow_walk", "walk", "run")
_GAITS = {
    "slow_walk": Gait.SLOW_WALK,
    "walk": Gait.WALK,
    "run": Gait.RUN,
    "side": Gait.SLOW_WALK,
    "turn": Gait.IDLE,
    "stop": Gait.IDLE,
}

Pose = tuple[float, float, float]  # x [m], y [m], yaw [rad]


@dataclass(frozen=True)
class Segment:
    """One Movement primitive: from the start pose to the target pose (relative to the plan's start, or in the world
    once a Plan executor anchors it).

    kind: "slow_walk", "walk" or "run" (forward), "side" (side move), "turn" (in place) or "stop" (stand for hold
    seconds, or until the next command if forever). Forward moves and side moves travel at speed [m/s]; forward moves
    slow down to exit_speed at the target.
    """

    kind: str
    start: Pose
    target: Pose
    speed: float = 0.0
    exit_speed: float = 0.0
    hold: float = 0.0
    forever: bool = False

    @property
    def gait(self) -> Gait:
        """The fastest planner mode this segment may use."""
        return _GAITS[self.kind]

    @property
    def decel(self) -> float:
        return RUN_DECEL if self.kind == "run" else WALK_DECEL


@dataclass(frozen=True)
class MovementPlan:
    """Movement primitives in order; each method returns a new plan, so calls chain.

    Primitives are relative to where the previous one left the robot. Forward moves go forward only. stop(seconds)
    stands for seconds; stop() stands until the next command and ends the plan, so nothing can follow it and a
    looping plan doesn't repeat. A looping plan without stop() runs until G1Wbc.halt() or another command.
    """

    loop: bool = False
    _primitives: tuple[Segment, ...] = field(default=(), repr=False)

    @property
    def repeats(self) -> bool:
        """Whether the plan starts over after its last segment."""
        return self.loop and not self._stopped

    @property
    def segments(self) -> tuple[Segment, ...]:
        """The segments, each forward move with its exit speed."""
        if not self._primitives:
            raise ValueError("MovementPlan: the plan is empty")
        segments = self._primitives
        count = len(segments)
        result = []
        for i, segment in enumerate(segments):
            following = segments[(i + 1) % count] if self.repeats or i + 1 < count else None
            carries_on = segment.kind in FORWARD_KINDS and following is not None and following.kind in FORWARD_KINDS
            result.append(replace(segment, exit_speed=min(segment.speed, following.speed) if carries_on else 0.0))
        return tuple(result)

    def slow_walk(self, meters: float, speed: float = SLOW_WALK_SPEED) -> "MovementPlan":
        """Slow-walk forward meters at speed [m/s]."""
        _check_range("speed", speed, MIN_SLOW_WALK_SPEED, MAX_SLOW_WALK_SPEED)
        return self._move("slow_walk", meters, 0.0, speed)

    def walk(self, meters: float, speed: float = WALK_SPEED) -> "MovementPlan":
        """Walk forward meters at speed [m/s]."""
        _check_range("speed", speed, MIN_WALK_SPEED, MAX_WALK_SPEED)
        return self._move("walk", meters, 0.0, speed)

    def run(self, meters: float, speed: float = RUN_SPEED) -> "MovementPlan":
        """Run forward meters at speed [m/s]."""
        _check_range("speed", speed, MIN_RUN_SPEED, MAX_RUN_SPEED)
        return self._move("run", meters, 0.0, speed)

    def side_move_left(self, meters: float, speed: float = SIDE_SPEED) -> "MovementPlan":
        _check_range("speed", speed, 0.0, MAX_SIDE_SPEED, low_open=True)
        return self._move("side", meters, math.pi / 2, speed)

    def side_move_right(self, meters: float, speed: float = SIDE_SPEED) -> "MovementPlan":
        _check_range("speed", speed, 0.0, MAX_SIDE_SPEED, low_open=True)
        return self._move("side", meters, -math.pi / 2, speed)

    def turn_left(self, degrees: float) -> "MovementPlan":
        """Turn in place counterclockwise by degrees."""
        return self._turn(degrees, 1.0)

    def turn_right(self, degrees: float) -> "MovementPlan":
        """Turn in place clockwise by degrees."""
        return self._turn(degrees, -1.0)

    def stop(self, seconds: float | None = None) -> "MovementPlan":
        """Slow down and stand for seconds, or with no seconds until the next command, ending the plan."""
        if seconds is not None:
            _check_positive("seconds", seconds)
        pose = self._end
        return self._add(Segment("stop", pose, pose, hold=seconds or 0.0, forever=seconds is None))

    @property
    def _end(self) -> Pose:
        return self._primitives[-1].target if self._primitives else (0.0, 0.0, 0.0)

    @property
    def _stopped(self) -> bool:
        return bool(self._primitives) and self._primitives[-1].forever

    def _move(self, kind: str, meters: float, direction: float, speed: float) -> "MovementPlan":
        """Move meters along the robot's heading rotated by direction, keeping the heading."""
        _check_positive("meters", meters)
        x, y, yaw = start = self._end
        target = (x + meters * math.cos(yaw + direction), y + meters * math.sin(yaw + direction), yaw)
        return self._add(Segment(kind, start, target, speed=speed))

    def _turn(self, degrees: float, sign: float) -> "MovementPlan":
        _check_positive("degrees", degrees)
        x, y, yaw = start = self._end
        return self._add(Segment("turn", start, (x, y, yaw + sign * math.radians(degrees))))

    def _add(self, segment: Segment) -> "MovementPlan":
        if self._stopped:
            raise ValueError("MovementPlan: nothing can follow stop() without seconds")
        return replace(self, _primitives=self._primitives + (segment,))


def _check_positive(name: str, value: float) -> None:
    if not value > 0:
        raise ValueError(f"MovementPlan: {name} must be positive, got {value}")


def _check_range(name: str, value: float, low: float, high: float, low_open: bool = False) -> None:
    if not (low < value if low_open else low <= value) or not value <= high:
        raise ValueError(f"MovementPlan: {name} must be in {'(' if low_open else '['}{low}, {high}], got {value}")
