"""SonicController: the G1 SONIC whole-body controller, stepped from the caller's physics callback."""

import logging
from enum import IntEnum
from pathlib import Path

import numpy as np
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager

from . import sonic_policy
from .kinematic_planner import KinematicPlanner

_log = logging.getLogger(__name__)


class SonicMode(IntEnum):
    """The 27 planner modes; the ids are the planner's (.agents/skills/oc-sonic-cmd/SKILL.md, localmotion_kplanner.hpp:78-115)."""

    IDLE = 0
    SLOW_WALK = 1
    WALK = 2
    RUN = 3
    SQUAT = 4
    KNEEL_TWO_LEGS = 5
    KNEEL_ONE_LEG = 6
    LYING_FACE_DOWN = 7
    HAND_CRAWL = 8
    BOXING_IDLE = 9
    BOXING_WALK = 10
    BOXING_LEFT_JAB = 11
    BOXING_RIGHT_JAB = 12
    BOXING_RANDOM_PUNCHES = 13
    ELBOW_CRAWL = 14
    BOXING_LEFT_HOOK = 15
    BOXING_RIGHT_HOOK = 16
    HAPPY_WALK = 17
    STEALTH_WALK = 18
    INJURED_WALK = 19
    CAREFUL_WALK = 20
    OBJECT_CARRYING = 21
    CROUCH_WALK = 22
    HAPPY_DANCE_WALK = 23
    ZOMBIE_WALK = 24
    POINT_WALK = 25
    SCARED_WALK = 26


class SonicController:
    """The G1 SONIC whole-body controller: loads the policy and the kinematic planner and drives the G1 at 50 Hz.

    controller = SonicController("/World/G1")  # ros=False: no ROS 2 subscriptions
    controller.command(SonicMode.WALK, vx=0.5)  # holds until the next command()
    controller.forward(dt)  # every physics step; sets up on the first call after construction or reset()
    controller.reset()  # on Stop: the next forward() starts again from the measured pose
    controller.close()  # on shutdown
    """

    def __init__(
        self, prim_path: str, *, ros: bool = True, directory: Path = sonic_policy.SONIC_DIR, providers: list[str] | None = None
    ) -> None:
        self._prim_path = prim_path
        self._ros = ros  # subscribe to /cmd_vel, /sonic/mode and /sonic/pelvis_height of the robot
        self._encoder = sonic_policy._session(directory / "model_encoder.onnx", providers)
        self._decoder = sonic_policy._session(directory / "model_decoder.onnx", providers)
        self._kplanner = KinematicPlanner(providers=providers)
        self._articulation: Articulation | None = None
        self._dof_indices: list[int] = []  # the 29 body DOFs in IsaacLab order
        self._histories = sonic_policy.DecoderHistories()
        self._last_action = np.zeros(sonic_policy.NUM_JOINTS, dtype=np.float32)
        self._heading: np.ndarray | None = None  # delta_heading, captured on the first policy tick after a start
        self._decimation = 1
        self._tick = 0
        self._started = False
        self._closed = False

    def command(
        self, mode: SonicMode | int, vx: float = 0.0, vy: float = 0.0, wz: float = 0.0, pelvis_height: float = -1.0
    ) -> None:
        """Sets the Locomotion command: `mode` (SonicMode or an id 0-26), `vx`, `vy` [m/s] and `wz` [rad/s] in the
        robot frame, `pelvis_height` [m] (-1 lets the mode decide). It holds until replaced, with no timeout. With
        ros=True the latest command wins, from ROS 2 or from here. Raises ValueError on a bad mode, leaving the
        previous command intact."""
        if self._closed:
            raise RuntimeError("SonicController.command() called after close()")
        self._kplanner.set_mode(int(mode))  # validates first: a bad mode raises before anything changes
        self._kplanner.set_cmd_vel(vx, vy, wz)
        self._kplanner.set_pelvis_height(pelvis_height)

    def forward(self, dt: float) -> None:
        """One physics step: on every decimation-th call, read the robot, run the policy and write the targets."""
        if self._closed:
            raise RuntimeError("SonicController.forward() called after close()")
        if not self._started:
            self._start(dt)
        if self._tick % self._decimation == 0:
            self._write_targets(self._infer(*self._read_state()))
        self._tick += 1

    def reset(self) -> None:
        """Clears the command to idle and makes the next forward() set up again, from the robot's measured pose."""
        if not self._closed:
            self.command(SonicMode.IDLE)
        self._started = False

    def close(self) -> None:
        """Closes the planner and drops the policy sessions and the articulation."""
        self._kplanner.close()
        self._encoder = self._decoder = None
        self._articulation = None
        self._started = False
        self._closed = True

    def _start(self, dt: float) -> None:
        """Builds the articulation, sets the drives and starts the planner and the policy from the measured joints."""
        self._articulation = Articulation(self._prim_path)
        dof_names = list(self._articulation.dof_names)
        self._dof_indices = [dof_names.index(name) for name in sonic_policy.ISAACLAB_JOINTS]
        self._configure_drives()
        self._decimation = max(1, round(sonic_policy.CONTROL_DT / dt))
        if abs(self._decimation * dt - sonic_policy.CONTROL_DT) > 1e-6:
            _log.warning(
                "SonicController: physics dt %s does not divide the %s s control period; running every %d steps",
                dt, sonic_policy.CONTROL_DT, self._decimation,
            )  # fmt: skip
        q = self._read_state()[0]
        self._kplanner.initialize(q[sonic_policy.ISAACLAB_OF_MUJOCO], self._prim_path if self._ros else None)
        self._histories.reset()
        self._last_action[:] = 0.0
        self._heading = None
        self._tick = 0
        self._started = True

    def _read_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Body joint positions and velocities (IsaacLab order), root orientation (wxyz) and world angular velocity."""
        q = self._articulation.get_dof_positions(dof_indices=self._dof_indices).numpy()[0]
        dq = self._articulation.get_dof_velocities(dof_indices=self._dof_indices).numpy()[0]
        q_root = self._articulation.get_world_poses()[1].numpy()[0]
        w_world = self._articulation.get_velocities()[1].numpy()[0]
        return q.astype(float), dq.astype(float), q_root.astype(float), w_world.astype(float)

    def _infer(self, q: np.ndarray, dq: np.ndarray, q_root: np.ndarray, w_world: np.ndarray) -> np.ndarray:
        """One tick: observations from the planner's current reference, the policy, then advance the planner."""
        ref_joints, ref_vels, ref_quats = self._kplanner.reference()
        if self._heading is None:
            self._heading = sonic_policy.delta_heading(q_root, ref_quats[0])

        token = self._encoder.run(None, {"obs_dict": sonic_policy.encoder_obs(ref_joints, ref_vels, ref_quats, q_root, self._heading)[None]})[0][0]
        self._histories.push(
            sonic_policy.ang_vel_body(q_root, w_world), q - sonic_policy.DEFAULT_POSE_IL, dq, self._last_action, sonic_policy.gravity_dir(q_root)
        )
        action = np.asarray(self._decoder.run(None, {"obs_dict": sonic_policy.decoder_obs(token, self._histories)[None]})[0]).reshape(-1)
        if action.shape != (sonic_policy.NUM_JOINTS,) or not np.isfinite(action).all():
            raise ValueError(f"SonicController: decoder produced a bad action (shape {action.shape}): {action}")
        self._last_action[:] = action
        self._kplanner.step(sonic_policy.CONTROL_DT)
        return sonic_policy.action_to_target(action).astype(np.float32)

    def _write_targets(self, target: np.ndarray) -> None:
        """Writes the joint position targets (IsaacLab order) to the body DOFs; other DOFs are untouched."""
        self._articulation.set_dof_position_targets(target[None], dof_indices=self._dof_indices)

    def _configure_drives(self) -> None:
        """PD gains, effort and velocity limits and armature of the 29 body DOFs, in rad units (TRN:238-357)."""
        indices = self._dof_indices
        self._articulation.set_dof_gains(sonic_policy.STIFFNESS_IL, sonic_policy.DAMPING_IL, dof_indices=indices)
        self._articulation.set_dof_armatures(sonic_policy.ARMATURE_IL, dof_indices=indices)
        if SimulationManager.get_active_physics_engine() != "newton":  # Newton has no per-DOF limits to set here
            self._articulation.set_dof_max_efforts(sonic_policy.EFFORT_LIMIT_IL, dof_indices=indices)
            self._articulation.set_dof_max_velocities(sonic_policy.VELOCITY_LIMIT_IL, dof_indices=indices)
