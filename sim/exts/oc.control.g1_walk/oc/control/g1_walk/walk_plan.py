"""Walk plan: Walk primitives in the robot's frame, converted to a Walk sequence."""

import math

DEFAULT_SPEED = 0.5  # m/s
DEFAULT_TURN_SPEED = 45.0  # deg/s
# The policy's trained command range: |vx|, |vy| <= 1 m/s, |wz| <= 1 rad/s.
MAX_SPEED = 1.0  # m/s
MAX_TURN_SPEED = math.degrees(1.0)  # deg/s


class WalkPlan:
    """Builds a Walk sequence from Walk primitives; each method returns the plan, so calls chain.

    The sequence repeats forever unless the plan ends with abort(), after which the G1 stands.
    """

    def __init__(self) -> None:
        self._steps = []
        self._aborted = False

    def move_forward(self, meters: float, speed: float = DEFAULT_SPEED) -> "WalkPlan":
        return self._move(meters, speed, speed, 0.0)

    def move_backward(self, meters: float, speed: float = DEFAULT_SPEED) -> "WalkPlan":
        return self._move(meters, speed, -speed, 0.0)

    def step_left(self, meters: float, speed: float = DEFAULT_SPEED) -> "WalkPlan":
        return self._move(meters, speed, 0.0, speed)

    def step_right(self, meters: float, speed: float = DEFAULT_SPEED) -> "WalkPlan":
        return self._move(meters, speed, 0.0, -speed)

    def turn_left(self, degrees: float, speed: float = DEFAULT_TURN_SPEED) -> "WalkPlan":
        return self._turn(degrees, speed, 1.0)

    def turn_right(self, degrees: float, speed: float = DEFAULT_TURN_SPEED) -> "WalkPlan":
        return self._turn(degrees, speed, -1.0)

    def stop(self, seconds: float) -> "WalkPlan":
        """Stand still for seconds."""
        _check_positive("seconds", seconds)
        return self._add((seconds, 0.0, 0.0, 0.0))

    def abort(self) -> "WalkPlan":
        """End the plan: the G1 stands forever and the sequence does not repeat."""
        self._add((math.inf, 0.0, 0.0, 0.0))
        self._aborted = True
        return self

    def to_walk_sequence(self) -> tuple[tuple[float, float, float, float], ...]:
        """Return ((seconds, vx [m/s], vy [m/s], wz [rad/s]), ...) for G1Walker."""
        if not self._steps:
            raise ValueError("WalkPlan: the plan is empty")
        return tuple(self._steps)

    def _move(self, meters: float, speed: float, vx: float, vy: float) -> "WalkPlan":
        _check_positive("meters", meters)
        _check_speed(speed, MAX_SPEED, "m/s")
        return self._add((meters / speed, vx, vy, 0.0))

    def _turn(self, degrees: float, speed: float, sign: float) -> "WalkPlan":
        _check_positive("degrees", degrees)
        _check_speed(speed, MAX_TURN_SPEED, "deg/s")
        return self._add((degrees / speed, 0.0, 0.0, sign * math.radians(speed)))

    def _add(self, step: tuple[float, float, float, float]) -> "WalkPlan":
        if self._aborted:
            raise ValueError("WalkPlan: abort() must be the last primitive")
        self._steps.append(step)
        return self


def _check_positive(name: str, value: float) -> None:
    if not value > 0:
        raise ValueError(f"WalkPlan: {name} must be positive, got {value}")


def _check_speed(speed: float, limit: float, unit: str) -> None:
    if not 0 < speed <= limit:
        raise ValueError(f"WalkPlan: speed must be in (0, {limit:.1f}] {unit}, got {speed}")
