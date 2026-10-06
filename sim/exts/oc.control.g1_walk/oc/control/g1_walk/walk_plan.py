"""Walk plan: Walk primitives, each at its own speed, that generate a Walk trajectory, the Walk goals G1WalkController follows."""

import math
from dataclasses import dataclass

# Default walking speeds, and the policy's trained command range (|vx|, |vy| <= 1 m/s, |wz| <= 1 rad/s).
SPEED = 0.5  # m/s
TURN_SPEED = math.radians(45.0)  # rad/s
MAX_SPEED = 1.0  # m/s
MAX_TURN_SPEED = 1.0  # rad/s


@dataclass(frozen=True)
class WalkGoal:
    """A pose to reach in the trajectory frame at speed and turn_speed, then hold for hold seconds."""

    x: float  # m
    y: float  # m
    yaw: float  # rad
    hold: float = 0.0  # s
    speed: float = SPEED  # m/s
    turn_speed: float = TURN_SPEED  # rad/s


@dataclass(frozen=True)
class WalkTrajectory:
    """Walk goals in order. frame "start": relative to the robot's pose when it starts; "world": absolute.

    A looping trajectory goes back to its first goal after the last one, keeping the same start pose.
    """

    goals: tuple[WalkGoal, ...]
    frame: str = "start"
    loop: bool = False

    def __post_init__(self) -> None:
        if not self.goals:
            raise ValueError("WalkTrajectory: the trajectory is empty")
        if self.frame not in ("start", "world"):
            raise ValueError(f"WalkTrajectory: frame must be 'start' or 'world', got {self.frame!r}")


class WalkPlan:
    """Builds a Walk trajectory from Walk primitives; each method returns the plan, so calls chain.

    Moves, turns and waits are relative to where the previous primitive left the robot. walk_to goes
    to a world pose, e.g. from a planner, and can't be mixed with them. Moves and walk_to take a speed
    in m/s, turns in deg/s. With loop=True the trajectory repeats, so a looping plan should end where
    it started; stop() ends the plan, so it doesn't repeat. to_trajectory() returns the Walk trajectory.
    """

    def __init__(self, loop: bool = False) -> None:
        self.loop = loop
        self._goals = []
        self._frame = None
        self._stopped = False
        self._x = self._y = self._yaw = 0.0

    @property
    def goals(self) -> tuple[WalkGoal, ...]:
        return tuple(self._goals)

    @property
    def frame(self) -> str | None:
        """"start" or "world"; None while the plan is empty."""
        return self._frame

    def move_forward(self, meters: float, speed: float = SPEED) -> "WalkPlan":
        return self._move(meters, 0.0, speed)

    def move_backward(self, meters: float, speed: float = SPEED) -> "WalkPlan":
        return self._move(meters, math.pi, speed)

    def step_left(self, meters: float, speed: float = SPEED) -> "WalkPlan":
        return self._move(meters, math.pi / 2, speed)

    def step_right(self, meters: float, speed: float = SPEED) -> "WalkPlan":
        return self._move(meters, -math.pi / 2, speed)

    def turn_left(self, degrees: float, speed: float = math.degrees(TURN_SPEED)) -> "WalkPlan":
        return self._turn(degrees, 1.0, speed)

    def turn_right(self, degrees: float, speed: float = math.degrees(TURN_SPEED)) -> "WalkPlan":
        return self._turn(degrees, -1.0, speed)

    def wait(self, seconds: float) -> "WalkPlan":
        """Stand still for seconds."""
        _check_positive("seconds", seconds)
        return self._add(WalkGoal(self._x, self._y, self._yaw, hold=seconds), "start")

    def walk_to(self, x: float, y: float, yaw: float, speed: float = SPEED) -> "WalkPlan":
        """Walk to the world pose (x [m], y [m], yaw [rad]) at speed [m/s]."""
        _check_speed("speed", speed, MAX_SPEED)
        return self._add(WalkGoal(x, y, yaw, speed=speed), "world")

    def stop(self) -> "WalkPlan":
        """End the plan: the robot stands after the last goal and a looping plan doesn't repeat."""
        self._check_not_stopped()
        self._stopped = True
        return self

    def to_trajectory(self) -> WalkTrajectory:
        """Return the Walk trajectory."""
        if not self._goals:
            raise ValueError("WalkPlan: the plan is empty")
        return WalkTrajectory(tuple(self._goals), self._frame, self.loop and not self._stopped)

    def _move(self, meters: float, direction: float, speed: float) -> "WalkPlan":
        """Move meters along the robot's heading rotated by direction, keeping the heading."""
        _check_positive("meters", meters)
        _check_speed("speed", speed, MAX_SPEED)
        self._x += meters * math.cos(self._yaw + direction)
        self._y += meters * math.sin(self._yaw + direction)
        return self._add(WalkGoal(self._x, self._y, self._yaw, speed=speed), "start")

    def _turn(self, degrees: float, sign: float, speed: float) -> "WalkPlan":
        _check_positive("degrees", degrees)
        _check_speed("speed", speed, math.degrees(MAX_TURN_SPEED))
        self._yaw += sign * math.radians(degrees)
        return self._add(WalkGoal(self._x, self._y, self._yaw, turn_speed=math.radians(speed)), "start")

    def _add(self, goal: WalkGoal, frame: str) -> "WalkPlan":
        self._check_not_stopped()
        if self._frame not in (None, frame):
            raise ValueError("WalkPlan: walk_to can't be mixed with relative primitives")
        self._frame = frame
        self._goals.append(goal)
        return self

    def _check_not_stopped(self) -> None:
        if self._stopped:
            raise ValueError("WalkPlan: the plan is stopped")


def _check_positive(name: str, value: float) -> None:
    if not value > 0:
        raise ValueError(f"WalkPlan: {name} must be positive, got {value}")


def _check_speed(name: str, value: float, maximum: float) -> None:
    if not 0 < value <= maximum:
        raise ValueError(f"WalkPlan: {name} must be in (0, {maximum:.3g}], got {value}")
