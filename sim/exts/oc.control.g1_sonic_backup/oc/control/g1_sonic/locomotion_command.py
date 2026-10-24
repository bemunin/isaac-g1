"""Locomotion command: the Whole-body controller's one input, from Real-time control or, later, from Nav2."""

import enum
import math
from dataclasses import dataclass

# Speeds of SONIC's planner modes (docs/source/tutorials/keyboard.md, gamepad.md): slow walk 0.2-0.8 m/s,
# walk 0.8-1.5 m/s, run 1.5-3.0 m/s; side steps at up to ~0.4 m/s, or the feet collide. Below 0.4 m/s a slow walk
# steps in place.
MAX_SLOW_WALK_SPEED = 0.8  # m/s
MAX_WALK_SPEED = 1.5  # m/s
MIN_RUN_SPEED, MAX_RUN_SPEED = 1.5, 3.0  # m/s
MAX_SIDE_SPEED = 0.4  # m/s
MAX_TURN_RATE = 1.5  # rad/s


class Gait(enum.IntEnum):
    """SONIC's planner modes from slowest to fastest, with kinematic_planner.py's mode numbers. A Locomotion command's
    gait is the fastest mode it may use."""

    IDLE = 0
    SLOW_WALK = 1
    WALK = 2
    RUN = 3


class Posture(enum.IntEnum):
    """SONIC's static ground postures (kinematic_planner.py's mode numbers). Unlike a Gait, a posture is held in
    place and needs a target pelvis height, not a velocity."""

    SQUAT = 4  # height 0.4-0.8 m
    KNEEL_TWO_LEGS = 5  # height 0.2-0.4 m
    KNEEL_ONE_LEG = 6  # height 0.2-0.4 m


# Allowed pelvis-height range [m] of each posture (SONIC's keyframes; see references/planner_onnx).
POSTURE_HEIGHT_RANGE = {
    Posture.SQUAT: (0.4, 0.8),
    Posture.KNEEL_TWO_LEGS: (0.2, 0.4),
    Posture.KNEEL_ONE_LEG: (0.2, 0.4),
}


class Crawl(enum.IntEnum):
    """SONIC's crawling modes (kinematic_planner.py's mode numbers): low locomotion modes driven by a velocity,
    unlike a static Posture. The body stays low and pitched forward; the robot holds the pose at zero speed."""

    HANDS = 8  # hand crawling, 0.4-1.0 m/s, pelvis ~0.4 m
    ELBOWS = 14  # elbow crawling, 0.7-1.0 m/s, pelvis ~0.25 m (overheats on the real robot)


# Speed range [m/s] of each crawl mode (tutorials/keyboard; a command below the low bound is raised to it).
CRAWL_SPEED_RANGE = {
    Crawl.HANDS: (0.4, 1.0),
    Crawl.ELBOWS: (0.7, 1.0),
}


@dataclass(frozen=True)
class LocomotionCommand:
    """Body-frame velocity: vx [m/s] forward, vy [m/s] left, wz [rad/s] counterclockwise. gait caps the planner mode;
    None picks it from the speed."""

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    gait: Gait | None = None


STAND = LocomotionCommand(gait=Gait.IDLE)


def gait_mode(speed: float) -> Gait:
    """SONIC planner mode for a speed [m/s]."""
    return Gait.RUN if speed >= MIN_RUN_SPEED else Gait.WALK if speed > MAX_SLOW_WALK_SPEED else Gait.SLOW_WALK


def check_positive(name: str, value: float) -> None:
    if not value > 0:
        raise ValueError(f"G1Wbc: {name} must be positive, got {value}")


def wrap(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)
