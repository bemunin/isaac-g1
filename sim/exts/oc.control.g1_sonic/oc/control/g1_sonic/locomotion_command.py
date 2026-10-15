"""Locomotion command: the Whole-body controller's one input, a body-frame velocity, e.g. from Nav2's cmd_vel."""

import enum
from dataclasses import dataclass

# SONIC's walk mode (planner mode 2, docs/source/tutorials/keyboard.md): forward or backward at 0.8-1.5 m/s here
# (backward up to 0.8), strafing at up to ~0.4 m/s, or the feet collide.
MIN_WALK_SPEED, MAX_WALK_SPEED = 0.8, 1.5  # m/s
MAX_BACKWARD_SPEED = 0.8  # m/s
MAX_SIDE_SPEED = 0.4  # m/s


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


class Posture(enum.IntEnum):
    """SONIC's static ground postures (kinematic_planner.py's mode numbers). A posture is held in place and needs a
    target pelvis height, not a velocity."""

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
    """Body-frame velocity: vx [m/s] forward, vy [m/s] left, wz [rad/s] counterclockwise."""

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


STAND = LocomotionCommand()
