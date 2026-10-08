"""Path tracker: turns the robot's pose and its active Segment (world frame) into a Locomotion command.

A tracker only steers; the Plan executor decides when a segment ends and stamps the gait.
"""

import math
from typing import Protocol

from .locomotion_command import LocomotionCommand
from .movement_plan import MAX_SIDE_SPEED, Pose, Segment

K_YAW = 2.0  # 1/s: heading error -> turn rate
MAX_TURN_RATE = 1.5  # rad/s
KP_CROSS_TRACK = 1.0  # 1/m: sideways distance from the segment -> direction correction
MAX_CROSS_TRACK = 0.5  # cap of that correction (0.5 ~ 27 deg)


def wrap(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


class PathTracker(Protocol):
    def reset(self, segment: Segment) -> None:
        """Start tracking segment."""

    def compute(self, pose: Pose, segment: Segment) -> LocomotionCommand:
        """The command for the robot at pose (x, y, yaw) on segment."""


def profile_speed(segment: Segment, remaining: float) -> float:
    """Speed [m/s] of a forward move remaining meters before its target: cruise, slowing to exit_speed at the target
    at the segment's deceleration."""
    return min(segment.speed, math.sqrt(segment.exit_speed**2 + 2.0 * segment.decel * max(remaining, 0.0)))


class GoalLineTracker:
    """Follows the straight line from a segment's start to its target, steering back onto it, and holds or turns to
    the target heading."""

    def reset(self, segment: Segment) -> None:
        pass

    def compute(self, pose: Pose, segment: Segment) -> LocomotionCommand:
        x, y, yaw = pose
        gx, gy, gyaw = segment.target
        wz = clamp(K_YAW * wrap(gyaw - yaw), -MAX_TURN_RATE, MAX_TURN_RATE)
        if segment.kind in ("turn", "stop"):
            return LocomotionCommand(wz=wz)
        sx, sy, _ = segment.start
        length = math.hypot(gx - sx, gy - sy)
        ux, uy = ((gx - sx) / length, (gy - sy) / length) if length > 1e-6 else (math.cos(gyaw), math.sin(gyaw))
        done = (x - sx) * ux + (y - sy) * uy
        lateral = -(x - sx) * uy + (y - sy) * ux  # positive: left of the segment
        correction = clamp(-KP_CROSS_TRACK * lateral, -MAX_CROSS_TRACK, MAX_CROSS_TRACK)
        movement = math.atan2(uy + correction * ux, ux - correction * uy)
        if segment.kind == "side":
            speed = min(segment.speed, MAX_SIDE_SPEED)
        else:
            speed = profile_speed(segment, length - done)
        return LocomotionCommand(speed * math.cos(movement - yaw), speed * math.sin(movement - yaw), wz)

