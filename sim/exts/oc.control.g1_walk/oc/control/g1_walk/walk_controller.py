"""Walk controller: follows a Walk trajectory with pose feedback, walking with Unitree's G1 policy."""

import enum
import math

import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np

from .policy import POLICY_PATH, _Policy
from .walk_plan import WalkGoal, WalkTrajectory

KP_POSITION = 1.5  # 1/s: position error [m] -> speed [m/s]
KP_YAW = 2.0  # 1/s: yaw error [rad] -> turn speed [rad/s]
POSITION_TOLERANCE = 0.10  # m
YAW_TOLERANCE = math.radians(3.0)  # rad
TIMEOUT_FACTOR = 2.0  # a goal fails after TIMEOUT_FACTOR x its nominal duration + TIMEOUT_MARGIN
TIMEOUT_MARGIN = 2.0  # s


class WalkStatus(enum.Enum):
    IDLE = "idle"  # no trajectory
    RUNNING = "running"
    DONE = "done"  # reached the last goal of a non-looping trajectory
    FAILED = "failed"  # a goal timed out


def yaw_of(wxyz) -> float:
    """Heading [rad] of a [w, x, y, z] orientation."""
    w, x, y, z = wxyz
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def to_world(goal: WalkGoal, anchor: tuple[float, float, float]) -> tuple[float, float, float]:
    """Pose (x, y, yaw) of a goal given in the frame of the anchor pose."""
    ax, ay, ayaw = anchor
    c, s = math.cos(ayaw), math.sin(ayaw)
    return ax + c * goal.x - s * goal.y, ay + s * goal.x + c * goal.y, ayaw + goal.yaw


def velocity_command(pose, goal: WalkGoal, anchor) -> tuple[float, float, float]:
    """Body-frame [vx, vy, wz] that drives the robot at pose (x, y, yaw) toward the goal.

    Position and heading are corrected independently, so a goal beside the robot is reached by
    stepping sideways and a goal at the same position by turning in place.
    """
    x, y, yaw = pose
    gx, gy, gyaw = to_world(goal, anchor)
    c, s = math.cos(yaw), math.sin(yaw)
    ex, ey = gx - x, gy - y
    vx, vy = KP_POSITION * (c * ex + s * ey), KP_POSITION * (-s * ex + c * ey)
    norm = math.hypot(vx, vy)
    if norm > goal.speed:
        vx, vy = vx * goal.speed / norm, vy * goal.speed / norm
    wz = max(-goal.turn_speed, min(goal.turn_speed, KP_YAW * wrap(gyaw - yaw)))
    return vx, vy, wz


class G1WalkController(mg.BaseController):
    """Turns the estimated G1 state and a Walk trajectory into joint position targets. Call forward at 50 Hz.

    The estimated state needs joint positions and velocities, root position and orientation (world
    frame) and root angular velocity in the body frame (IMU gyro). Each forward steers toward the
    current Walk goal with a [vx, vy, wz] command and runs the walking policy on it.
    """

    def __init__(self, policy_path=POLICY_PATH) -> None:
        self._policy = _Policy(policy_path)
        self._trajectory = None
        self._trajectory_pending = False
        self._anchor = (0.0, 0.0, 0.0)
        self._index = 0
        self._goal_deadline = math.inf
        self._arrived_at = None
        self.status = WalkStatus.IDLE

    def set_trajectory(self, trajectory: WalkTrajectory | None) -> None:
        """Follow trajectory from its first goal, starting at the next forward; the gait carries on."""
        self._trajectory = trajectory
        self._trajectory_pending = True

    def reset(self, estimated_state: mg.RobotState, setpoint_state: mg.RobotState | None, t: float, **kwargs) -> bool:
        """Reset the policy and restart the trajectory from its first goal at the current pose."""
        self._policy.reset()
        self._start_trajectory(self.pose(estimated_state), t)
        return True

    def forward(
        self, estimated_state: mg.RobotState, setpoint_state: mg.RobotState | None, t: float, **kwargs
    ) -> mg.RobotState:
        pose = self.pose(estimated_state)
        if self._trajectory_pending:
            self._start_trajectory(pose, t)
        return self._policy.act(estimated_state, self.step(pose, t), t)

    def step(self, pose: tuple[float, float, float], t: float) -> tuple[float, float, float]:
        """Advance through the trajectory and return the [vx, vy, wz] command for the robot at pose (x, y, yaw)."""
        if self.status != WalkStatus.RUNNING:
            return 0.0, 0.0, 0.0
        goal = self._trajectory.goals[self._index]
        gx, gy, gyaw = to_world(goal, self._anchor)
        reached = math.hypot(gx - pose[0], gy - pose[1]) < POSITION_TOLERANCE and abs(wrap(gyaw - pose[2])) < YAW_TOLERANCE
        if reached and self._arrived_at is None:
            self._arrived_at = t
        if self._arrived_at is not None and t - self._arrived_at >= goal.hold:
            index = self._index + 1
            if index == len(self._trajectory.goals):
                if not self._trajectory.loop:
                    self.status = WalkStatus.DONE
                    return 0.0, 0.0, 0.0
                index = 0
            self._start_goal(index, pose, t)
            goal = self._trajectory.goals[index]
        elif t > self._goal_deadline:
            self.status = WalkStatus.FAILED
            return 0.0, 0.0, 0.0
        return velocity_command(pose, goal, self._anchor)

    @staticmethod
    def pose(estimated_state: mg.RobotState) -> tuple[float, float, float]:
        """World (x, y, yaw) from the estimated state's root position and orientation."""
        root = estimated_state.root
        position = root.position.numpy().reshape(-1)
        return float(position[0]), float(position[1]), yaw_of(np.asarray(root.orientation.numpy(), dtype=float).reshape(-1))

    def _start_trajectory(self, pose, t: float) -> None:
        self._trajectory_pending = False
        if self._trajectory is None:
            self.status = WalkStatus.IDLE
            return
        self._anchor = pose if self._trajectory.frame == "start" else (0.0, 0.0, 0.0)
        self.status = WalkStatus.RUNNING
        self._start_goal(0, pose, t)

    def _start_goal(self, index: int, pose, t: float) -> None:
        self._index = index
        self._arrived_at = None
        goal = self._trajectory.goals[index]
        gx, gy, gyaw = to_world(goal, self._anchor)
        nominal = (
            math.hypot(gx - pose[0], gy - pose[1]) / goal.speed
            + abs(wrap(gyaw - pose[2])) / goal.turn_speed
            + goal.hold
        )
        self._goal_deadline = t + TIMEOUT_FACTOR * nominal + TIMEOUT_MARGIN
