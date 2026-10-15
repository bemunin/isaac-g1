"""Whole-body controller: turns Real-time Locomotion commands into joint targets, moving the G1 with NVIDIA SONIC
(kinematic planner + tracking policy) in its walk mode."""

import enum
import math

import carb
import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import warp as wp

from ..locomotion_command import (
    CRAWL_SPEED_RANGE,
    MAX_BACKWARD_SPEED,
    MAX_SIDE_SPEED,
    MAX_WALK_SPEED,
    MIN_WALK_SPEED,
    STAND,
    Crawl,
    LocomotionCommand,
)
from .kinematic_planner import IDLE, WALK, KinematicPlanner
from .sonic import CONTROL_DT, MUJOCO_JOINTS, SONIC_DIR, SonicPolicy, yaw_of

DEADBAND = 0.1  # m/s: slower commands stand
MAX_FACING_LEAD = math.radians(30.0)  # rad: the facing the planner is told stays this close to the measured yaw
COMMAND_TIMEOUT = 0.5  # s: a real-time command without a new one stops
MAX_TURN_RATE = 1.5  # rad/s


class MovementStatus(enum.Enum):
    """Status of the controller."""

    IDLE = "idle"  # standing, no command
    RUNNING = "running"  # following a real-time command


def wrap(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


def limit(command: LocomotionCommand) -> LocomotionCommand:
    """command within what the robot walks: -0.8 <= vx <= 1.5, |vy| <= 0.4, |wz| <= 1.5."""
    return LocomotionCommand(
        vx=clamp(float(command.vx), -MAX_BACKWARD_SPEED, MAX_WALK_SPEED),
        vy=clamp(float(command.vy), -MAX_SIDE_SPEED, MAX_SIDE_SPEED),
        wz=clamp(float(command.wz), -MAX_TURN_RATE, MAX_TURN_RATE),
    )


class G1SonicController(mg.BaseController):
    """Turns the estimated G1 state and a real-time Locomotion command into joint position targets for all 29 body
    joints, walking in SONIC's walk mode and standing in its idle mode. Call forward at 50 Hz.

    The estimated state needs joint positions and velocities, root position and orientation (world frame) and root
    angular velocity in the body frame (IMU gyro).
    """

    def __init__(self, directory=SONIC_DIR) -> None:
        self._policy = SonicPolicy(directory)
        self._planner = KinematicPlanner(directory)
        self._command = None  # the real-time command
        self._command_time = None  # t of its arrival; None until the next step stamps it
        self._facing = 0.0  # world yaw the planner is told to face
        self._heading_offset = 0.0  # world yaw - planner yaw
        self._clamp_warned = False
        self._posture = None  # (planner mode, pelvis height [m]) while holding a static ground posture, else None
        self._crawl = None  # a Crawl mode while crawling (cmd_vel drives it), else None (walk mode)

    @property
    def status(self) -> MovementStatus:
        busy = self._command is not None or self._posture is not None or self._crawl is not None
        return MovementStatus.RUNNING if busy else MovementStatus.IDLE

    def command(self, command: LocomotionCommand) -> None:
        """Real-time Locomotion command, limited (see limit); stands COMMAND_TIMEOUT s after the last call
        (a crawl mode holds its low pose instead). Cancels a held posture."""
        self._posture = None
        limited = limit(command)
        if limited != command and not self._clamp_warned:
            carb.log_warn(f"G1SonicController: {command} limited to {limited}")
            self._clamp_warned = True
        self._command = limited
        self._command_time = None

    def halt(self) -> None:
        """Slow down and stand, dropping the real-time command, posture or crawl mode."""
        self._command = None
        self._posture = None
        self._crawl = None

    def posture(self, mode: int, height: float) -> None:
        """Hold a static ground posture (SONIC planner mode 4-6) at pelvis height [m], dropping the real-time command.
        Held until another command; halt() or any drive stands back up."""
        self.halt()
        self._posture = (int(mode), float(height))

    def crawl(self, mode: int) -> None:
        """Enter a crawl mode (SONIC planner mode 8 hands / 14 elbows): a low moving mode that real-time commands
        drive. The robot holds the low pose when no command arrives; halt() stands back up."""
        self.halt()
        self._crawl = Crawl(int(mode))

    def reset(self, estimated_state: mg.RobotState, setpoint_state: mg.RobotState | None, t: float, **kwargs) -> bool:
        """Restart the planner standing still and align it with the robot's heading."""
        base_quat = self._orientation(estimated_state)
        self._planner.reset()
        self._policy.reset(base_quat, self._planner.root_quat())
        self._heading_offset = wrap(yaw_of(base_quat) - yaw_of(self._planner.root_quat()))
        self._facing = yaw_of(base_quat)
        self._command = None
        self._posture = None
        self._crawl = None
        return True

    def forward(self, estimated_state: mg.RobotState, setpoint_state: mg.RobotState | None, t: float, **kwargs) -> mg.RobotState:
        pose = self.pose(estimated_state)
        offset = self._heading_offset
        if self._posture is not None:  # hold a static ground posture at its height, facing where it stood
            mode, height = self._posture
            self._planner.command(mode, None, wrap(self._facing - offset), -1.0, height)
        else:
            mode, movement, speed = self.planner_inputs(self.step(pose, t), pose[2])
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
        """The Locomotion command for the robot at pose (x, y, yaw) at time t [s]: the real-time one, or STAND once
        none arrived for COMMAND_TIMEOUT s."""
        if self._command is None:
            return STAND
        if self._command_time is None:
            self._command_time = t
        elif t - self._command_time > COMMAND_TIMEOUT:
            self._command = None
            return STAND
        return self._command

    def planner_inputs(self, command: LocomotionCommand, yaw: float) -> tuple[int, float | None, float]:
        """(planner mode: WALK, IDLE or the Crawl mode, world movement yaw or None to stand, speed [m/s]) for command with the robot at
        world yaw. A mostly forward or backward command walks at 0.8-1.5 m/s; a mostly sideways one keeps its speed
        (up to 0.4 m/s). Also turns the facing the planner is told by wz, keeping it within MAX_FACING_LEAD of yaw."""
        command = limit(command)
        lead = clamp(wrap(self._facing + command.wz * CONTROL_DT - yaw), -MAX_FACING_LEAD, MAX_FACING_LEAD)
        self._facing = wrap(yaw + lead)
        speed = math.hypot(command.vx, command.vy)
        if self._crawl is not None:  # crawl: cmd_vel drives a low mode, forward only; hold the low pose when stopped
            vx = max(command.vx, 0.0)
            speed = math.hypot(vx, command.vy)
            if speed < DEADBAND:
                return self._crawl, None, 0.0
            low, high = CRAWL_SPEED_RANGE[self._crawl]
            return self._crawl, wrap(yaw + math.atan2(command.vy, vx)), clamp(speed, low, high)
        if speed < DEADBAND:
            return IDLE, None, 0.0
        if abs(command.vx) >= abs(command.vy):  # side steps stay slow, or the feet collide
            speed = clamp(speed, MIN_WALK_SPEED, MAX_WALK_SPEED)
        return WALK, wrap(yaw + math.atan2(command.vy, command.vx)), speed

    @staticmethod
    def _orientation(estimated_state: mg.RobotState) -> np.ndarray:
        return np.asarray(estimated_state.root.orientation.numpy(), dtype=float).reshape(-1)

    @staticmethod
    def pose(estimated_state: mg.RobotState) -> tuple[float, float, float]:
        """World (x, y, yaw) from the estimated state's root position and orientation."""
        position = estimated_state.root.position.numpy().reshape(-1)
        return float(position[0]), float(position[1]), yaw_of(G1SonicController._orientation(estimated_state))
