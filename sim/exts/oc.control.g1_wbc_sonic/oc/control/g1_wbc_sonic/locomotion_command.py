"""Locomotion command: the Whole-body controller's one input, from a Movement plan (through the Path tracker), from
Real-time control or, later, from Nav2."""

import enum
from dataclasses import dataclass


class Gait(enum.IntEnum):
    """SONIC's planner modes from slowest to fastest, with planner.py's mode numbers. A Locomotion command's gait is
    the fastest mode it may use."""

    IDLE = 0
    SLOW_WALK = 1
    WALK = 2
    RUN = 3


@dataclass(frozen=True)
class LocomotionCommand:
    """Body-frame velocity: vx [m/s] forward, vy [m/s] left, wz [rad/s] counterclockwise. gait caps the planner mode;
    None picks it from the speed."""

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    gait: Gait | None = None


STAND = LocomotionCommand(gait=Gait.IDLE)
