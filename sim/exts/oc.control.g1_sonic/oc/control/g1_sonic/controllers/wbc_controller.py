"""Whole-body controller: turns Locomotion commands, from a Behavior sequence (Sequence executor) or from Real-time
control, into joint targets, moving the G1 with NVIDIA SONIC (kinematic planner + tracking policy)."""

import math
from dataclasses import replace

import carb
import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import warp as wp

from ..behavior_sequence import (
    MAX_RUN_SPEED,
    MAX_SIDE_SPEED,
    MAX_SLOW_WALK_SPEED,
    MAX_WALK_SPEED,
    BehaviorSequence,
    check_positive,
    gait_mode,
)
from ..locomotion_command import STAND, Gait, LocomotionCommand
from ..sequence_executor import (
    MAX_TURN_RATE,
    MovementFailure,
    MovementStatus,
    SequenceExecutor,
    clamp,
    wrap,
)
from .kinematic_planner import KinematicPlanner
from .sonic import CONTROL_DT, MUJOCO_JOINTS, SONIC_DIR, SonicPolicy, yaw_of

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


class G1WbcController(mg.BaseController):
    """Turns the estimated G1 state and a Behavior sequence or real-time Locomotion command into joint position
    targets for all 29 body joints. Call forward at 50 Hz.

    The estimated state needs joint positions and velocities, root position and orientation (world frame) and root
    angular velocity in the body frame (IMU gyro). on_done(status) is called when a Behavior sequence ends:
    SUCCEEDED, ABORTED or CANCELED.
    """

    def __init__(self, directory=SONIC_DIR, on_done=None) -> None:
        self._policy = SonicPolicy(directory)
        self._planner = KinematicPlanner(directory)
        self._executor = SequenceExecutor(on_done)
        self._command = None  # the real-time command
        self._command_time = None  # t of its arrival; None until the next step stamps it
        self._realtime_status = None  # status of real-time control, or None to report the executor's
        self._speed_limit = None  # m/s: cap on real-time commands' linear speed, or None
        self._facing = 0.0  # world yaw the planner is told to face
        self._heading_offset = 0.0  # world yaw - planner yaw
        self._clamp_warned = False

    @property
    def on_done(self):
        return self._executor.on_done

    @on_done.setter
    def on_done(self, callback) -> None:
        self._executor.on_done = callback

    @property
    def status(self) -> MovementStatus:
        return self._executor.status if self._realtime_status is None else self._realtime_status

    @property
    def failure(self) -> MovementFailure | None:
        """Why the last Behavior sequence was aborted, if it was."""
        return self._executor.failure

    @property
    def sequence(self) -> BehaviorSequence | None:
        return self._executor.sequence

    def execute(self, sequence: BehaviorSequence) -> None:
        """Follow sequence from its first segment, starting at the next forward; cancels what is running."""
        self._cancel()
        self._realtime_status = None
        self._executor.execute(sequence)

    def command(self, command: LocomotionCommand) -> None:
        """Real-time Locomotion command, limited (see limit) and to the speed limit; stands COMMAND_TIMEOUT s after the
        last call. Cancels a running Behavior sequence."""
        limited = limit(command)
        if limited != command and not self._clamp_warned:
            carb.log_warn(f"G1WbcController: {command} limited to {limited}")
            self._clamp_warned = True
        speed = math.hypot(limited.vx, limited.vy)
        if self._speed_limit is not None and speed > self._speed_limit:
            scale = self._speed_limit / speed
            limited = replace(limited, vx=limited.vx * scale, vy=limited.vy * scale)
        if self._command is None:
            self._executor.cancel()
        self._command = limited
        self._command_time = None
        self._realtime_status = MovementStatus.RUNNING

    def set_speed_limit(self, speed_limit: float | None) -> None:
        """Cap the linear speed [m/s] of real-time commands from their next call, like Nav2's SpeedLimit (absolute);
        None removes the cap. The cap also caps the gait: up to 0.8 m/s slow walk, up to 1.5 m/s walk."""
        if speed_limit is not None:
            check_positive("speed_limit", speed_limit)
        self._speed_limit = speed_limit

    def halt(self) -> None:
        """Slow down and stand, cancelling what is running."""
        self._cancel()

    def reset(self, estimated_state: mg.RobotState, setpoint_state: mg.RobotState | None, t: float, **kwargs) -> bool:
        """Restart the planner standing still, align it with the robot's heading and restart the Behavior sequence."""
        base_quat = self._orientation(estimated_state)
        self._planner.reset()
        self._policy.reset(base_quat, self._planner.root_quat())
        self._heading_offset = wrap(yaw_of(base_quat) - yaw_of(self._planner.root_quat()))
        self._facing = yaw_of(base_quat)
        self._command = None
        self._realtime_status = None
        self._executor.restart()
        return True

    def forward(self, estimated_state: mg.RobotState, setpoint_state: mg.RobotState | None, t: float, **kwargs) -> mg.RobotState:
        pose = self.pose(estimated_state)
        mode, movement, speed = self.planner_inputs(self.step(pose, t), pose[2])
        offset = self._heading_offset
        self._planner.command(int(mode), None if movement is None else wrap(movement - offset), wrap(self._facing - offset), speed)

        joints = estimated_state.joints
        names = list(joints.robot_joint_space)
        indices = [names.index(name) for name in MUJOCO_JOINTS]
        q = joints.positions.numpy().reshape(-1)[indices]
        dq = joints.velocities.numpy().reshape(-1)[indices]
        gyro = estimated_state.root.angular_velocity.numpy().reshape(-1)
        targets = self._policy.act(q, dq, self._orientation(estimated_state), gyro, self._planner.reference())
        self._planner.step()
        return mg.RobotState(
            joints=mg.JointState.from_name(
                joints.robot_joint_space,
                positions=(list(MUJOCO_JOINTS), wp.array(targets.astype(np.float32), dtype=wp.float32, device="cpu")),
            )
        )

    def step(self, pose: tuple[float, float, float], t: float) -> LocomotionCommand:
        """The Locomotion command for the robot at pose (x, y, yaw) at time t [s]: the real-time one, or the Sequence
        executor's, with its segment's gait."""
        if self._command is not None:
            if self._command_time is None:
                self._command_time = t
            elif t - self._command_time > COMMAND_TIMEOUT:
                self._command = None
                self._realtime_status = MovementStatus.IDLE
                return STAND
            return self._command
        return self._executor.update(pose, t) or STAND

    def planner_inputs(self, command: LocomotionCommand, yaw: float) -> tuple[Gait, float | None, float]:
        """(planner mode, world movement yaw or None to stand, speed [m/s]) for command with the robot at world yaw.
        Also turns the facing the planner is told by wz, keeping it within MAX_FACING_LEAD of yaw."""
        command = limit(command)
        lead = clamp(wrap(self._facing + command.wz * CONTROL_DT - yaw), -MAX_FACING_LEAD, MAX_FACING_LEAD)
        self._facing = wrap(yaw + lead)
        ceiling = Gait.RUN if command.gait is None else command.gait
        speed = math.hypot(command.vx, command.vy)
        if ceiling == Gait.IDLE or speed < DEADBAND:
            return Gait.IDLE, None, 0.0
        if command.vx >= abs(command.vy):  # mostly forward; side steps are slow by design
            speed = max(speed, MIN_MOVING_SPEED)
        return min(ceiling, gait_mode(speed)), wrap(yaw + math.atan2(command.vy, command.vx)), speed

    def _cancel(self) -> None:
        """Cancel a running Behavior sequence or real-time command; the robot stands facing where it was told to."""
        if self._command is not None:
            self._command = None
            self._realtime_status = MovementStatus.CANCELED
        self._executor.cancel()

    @staticmethod
    def _orientation(estimated_state: mg.RobotState) -> np.ndarray:
        return np.asarray(estimated_state.root.orientation.numpy(), dtype=float).reshape(-1)

    @staticmethod
    def pose(estimated_state: mg.RobotState) -> tuple[float, float, float]:
        """World (x, y, yaw) from the estimated state's root position and orientation."""
        position = estimated_state.root.position.numpy().reshape(-1)
        return float(position[0]), float(position[1]), yaw_of(G1WbcController._orientation(estimated_state))
