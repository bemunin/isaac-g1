"""Locomotion command: the Whole-body controller's one input, from a Behavior sequence (through the Sequence executor),
from Real-time control or, later, from Nav2."""

import enum
from dataclasses import dataclass


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
