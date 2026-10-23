"""Whole-body controller: turns Locomotion commands from Real-time control into joint targets, moving the G1 with
NVIDIA SONIC (kinematic planner + tracking policy)."""

import math
from dataclasses import dataclass, replace

import carb
import numpy as np

from ..locomotion_command import (
    CRAWL_SPEED_RANGE,
    MAX_RUN_SPEED,
    MAX_SIDE_SPEED,
    MAX_SLOW_WALK_SPEED,
    MAX_TURN_RATE,
    MAX_WALK_SPEED,
    STAND,
    Crawl,
    Gait,
    LocomotionCommand,
    check_positive,
    clamp,
    gait_mode,
    wrap,
)
from .kinematic_planner import KinematicPlanner
from .sonic import CONTROL_DT, SONIC_DIR, SonicPolicy, yaw_of

DEADBAND = 0.1  # m/s: slower commands stand
MIN_MOVING_SPEED = 0.4  # m/s: slower, SONIC's slow walk steps in place (its planner eases off from rest by itself)
MAX_FACING_LEAD = math.radians(30.0)  # rad: the facing the planner is told stays this close to the measured yaw
COMMAND_TIMEOUT = 0.5  # s: a real-time command without a new one stops
TOP_SPEED = {Gait.IDLE: 0.0, Gait.SLOW_WALK: MAX_SLOW_WALK_SPEED, Gait.WALK: MAX_WALK_SPEED, Gait.RUN: MAX_RUN_SPEED}


def limit(command: LocomotionCommand) -> LocomotionCommand:
    """command within what the robot can do: forward only, up to its gait's top speed (RUN's for None), |vy| <= 0.4,
    |wz| <= 1.5; an IDLE command only turns."""
    top = TOP_SPEED[Gait.RUN if command.gait is None else command.gait]
    side = MAX_SIDE_SPEED if top > 0.0 else 0.0
    return replace(
        command,
        vx=clamp(float(command.vx), 0.0, top),
        vy=clamp(float(command.vy), -side, side),
        wz=clamp(float(command.wz), -MAX_TURN_RATE, MAX_TURN_RATE),
    )


@dataclass(frozen=True)
class G1State:
    """Estimated G1 state, joints in MUJOCO_JOINTS order. The root position is not needed."""

    q: np.ndarray  # joint positions [rad]
    dq: np.ndarray  # joint velocities [rad/s]
    orientation: np.ndarray  # root quaternion (w, x, y, z), world frame
    gyro: np.ndarray  # root angular velocity, body frame [rad/s] (IMU)


class G1WbcController:
    """Turns the estimated G1 state (a G1State) and a real-time Locomotion command into joint position targets for
    all 29 body joints, in MUJOCO_JOINTS order. Call forward at 50 Hz.
    """

    def __init__(self, directory=SONIC_DIR) -> None:
        self._policy = SonicPolicy(directory)
        self._planner = KinematicPlanner(directory)
        self._command = None  # the real-time command
        self._command_time = None  # t of its arrival; None until the next step stamps it
        self._speed_limit = None  # m/s: cap on real-time commands' linear speed, or None
        self._facing = 0.0  # world yaw the planner is told to face
        self._heading_offset = 0.0  # world yaw - planner yaw
        self._clamp_warned = False
        self._posture = None  # (planner mode, pelvis height [m]) while holding a static ground posture, else None
        self._locomotion_mode = None  # a Crawl mode while crawling (cmd_vel drives it), else None (normal gaits)

    def command(self, command: LocomotionCommand) -> None:
        """Real-time Locomotion command, limited (see limit) and to the speed limit; stands COMMAND_TIMEOUT s after the
        last call. Cancels a posture."""
        self._posture = None
        limited = limit(command)
        if limited != command and not self._clamp_warned:
            carb.log_warn(f"G1WbcController: {command} limited to {limited}")
            self._clamp_warned = True
        speed = math.hypot(limited.vx, limited.vy)
        if self._speed_limit is not None and speed > self._speed_limit:
            scale = self._speed_limit / speed
            limited = replace(limited, vx=limited.vx * scale, vy=limited.vy * scale)
        self._command = limited
        self._command_time = None

    def set_speed_limit(self, speed_limit: float | None) -> None:
        """Cap the linear speed [m/s] of real-time commands from their next call, like Nav2's SpeedLimit (absolute);
        None removes the cap. The cap also caps the gait: up to 0.8 m/s slow walk, up to 1.5 m/s walk."""
        if speed_limit is not None:
            check_positive("speed_limit", speed_limit)
        self._speed_limit = speed_limit

    def halt(self) -> None:
        """Slow down and stand, cancelling what is running."""
        self._cancel()

    def posture(self, mode: int, height: float) -> None:
        """Hold a static ground posture (SONIC planner mode 4-6) at pelvis height [m], cancelling what is running.
        Held until another command; halt() or any drive stands back up."""
        self._cancel()
        self._posture = (int(mode), float(height))

    def crawl(self, mode: int) -> None:
        """Enter a crawl mode (SONIC planner mode 8 hands / 14 elbows): a low moving mode that real-time commands
        drive, cancelling a posture. The robot holds the low pose when no command arrives;
        halt() stands back up."""
        self._cancel()
        self._locomotion_mode = Crawl(int(mode))

    def reset(self, state: G1State) -> None:
        """Restart the planner standing still, align it with the robot's heading."""
        base_quat = np.asarray(state.orientation, dtype=float).reshape(-1)
        self._planner.reset()
        self._policy.reset(base_quat, self._planner.root_quat())
        self._heading_offset = wrap(yaw_of(base_quat) - yaw_of(self._planner.root_quat()))
        self._facing = yaw_of(base_quat)
        self._command = None
        self._posture = None
        self._locomotion_mode = None

    def forward(self, state: G1State, t: float) -> np.ndarray:
        """Joint position targets (29, float32, MUJOCO_JOINTS order) for the estimated state at time t [s]."""
        base_quat = np.asarray(state.orientation, dtype=float).reshape(-1)
        offset = self._heading_offset
        if self._posture is not None:  # hold a static ground posture at its height, facing where it stood
            mode, height = self._posture
            self._planner.command(mode, None, wrap(self._facing - offset), -1.0, height)
        else:
            mode, movement, speed = self.planner_inputs(self.step(t), yaw_of(base_quat))
            self._planner.command(int(mode), None if movement is None else wrap(movement - offset), wrap(self._facing - offset), speed)

        targets = self._policy.act(state.q, state.dq, base_quat, state.gyro, self._planner.reference())
        self._planner.step()
        return targets.astype(np.float32)

    def step(self, t: float) -> LocomotionCommand:
        """The real-time Locomotion command at time t [s], or STAND once it has timed out or when there is none."""
        if self._command is None:
            return STAND
        if self._command_time is None:
            self._command_time = t
        elif t - self._command_time > COMMAND_TIMEOUT:
            self._command = None
            return STAND
        return self._command

    def planner_inputs(self, command: LocomotionCommand, yaw: float) -> tuple[int, float | None, float]:
        """(planner mode, world movement yaw or None to stand, speed [m/s]) for command with the robot at world yaw.
        Also turns the facing the planner is told by wz, keeping it within MAX_FACING_LEAD of yaw."""
        command = limit(command)
        lead = clamp(wrap(self._facing + command.wz * CONTROL_DT - yaw), -MAX_FACING_LEAD, MAX_FACING_LEAD)
        self._facing = wrap(yaw + lead)
        if self._locomotion_mode is not None:  # crawl: cmd_vel drives a low mode; hold the low pose when stopped
            speed = math.hypot(command.vx, command.vy)
            if speed < DEADBAND:
                return self._locomotion_mode, None, 0.0
            low, high = CRAWL_SPEED_RANGE[self._locomotion_mode]
            return self._locomotion_mode, wrap(yaw + math.atan2(command.vy, command.vx)), clamp(speed, low, high)
        ceiling = Gait.RUN if command.gait is None else command.gait
        speed = math.hypot(command.vx, command.vy)
        if ceiling == Gait.IDLE or speed < DEADBAND:
            return Gait.IDLE, None, 0.0
        if command.vx >= abs(command.vy):  # mostly forward; side steps are slow by design
            speed = max(speed, MIN_MOVING_SPEED)
        return min(ceiling, gait_mode(speed)), wrap(yaw + math.atan2(command.vy, command.vx)), speed

    def _cancel(self) -> None:
        """Cancel the real-time command, posture or crawl mode; the robot stands."""
        self._command = None
        self._posture = None
        self._locomotion_mode = None
