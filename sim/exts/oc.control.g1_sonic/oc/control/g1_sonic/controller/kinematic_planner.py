"""SONIC's kinematic planner: builds the planner inputs from ROS 2 commands and its own previous plan, and runs
planner_sonic.onnx. Stepped by its owner and registers no physics callback; pure numpy and onnxruntime.

Asynchronous like NVIDIA's deploy: every plan is resampled to a 50 Hz stored motion that the owner reads at its tick,
and a replan runs on one worker thread and is cross-faded into the stored motion when it lands, so the tick never waits
(gear_sonic_deploy localmotion_kplanner.hpp:395-519 resample, :591-680 context; g1_deploy_onnx_ref.cpp:3184-3398 blend).

Design: docs/adr/0003-kinematic-planner-inputs.md.
"""

import math
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .sonic_policy import (
    DEFAULT_HEIGHT,
    FUTURE_FRAMES,
    ISAACLAB_OF_MUJOCO,
    MUJOCO_OF_ISAACLAB,
    SONIC_DIR,
    _session,
    quat_slerp,
)

NUM_MODES = 27  # planner modes 0-26
IDLE = 0
PLANNER_PERIOD = 0.1  # s between plans (10 Hz)
PLANNER_RATE = 30.0  # Hz: the planner's frame rate
MOTION_RATE = 50.0  # Hz: the stored motion's frame rate, the policy tick
LOOK_AHEAD = 2  # 50 Hz frames between the playback frame and where a replan starts (motion_look_ahead_steps)
BLEND_FRAMES = 8  # 50 Hz frames of the cross-fade when a plan is spliced in; about the planner's context length
REFERENCE_STEP = 5  # 50 Hz frames between the FUTURE_FRAMES reference frames (0.1 s)
MIN_SPEED = 0.01  # m/s: a slower command stands still
CMD_TIMEOUT = 1.0  # s: an older /cmd_vel counts as zero (ros2_input_handler.hpp:305-318)
MIN_TURN_RATE = 0.01  # rad/s: a slower /cmd_vel wz is ignored (ros2_input_handler.hpp:740)
SPEED_RANGES = {1: (0.2, 0.8), 2: (0.8, 1.5), 3: (1.5, 3.0)}  # m/s for slow walk, walk, run (gamepad.hpp:349-367)
DEFAULT_SPEED_MODES = frozenset({4, 5, 6})  # squat, kneel two legs, kneel: target_vel -1 (gamepad.hpp:369-381)
# Modes that go to IDLE when stopped (localmotion_kplanner.hpp:117-139 is_standing_motion_mode).
STANDING_MODES = frozenset({0, 1, 2, 3, 9, 10, 11, 12, 13, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26})
_DIRECTION_BIN = math.pi / 4  # 8 movement directions, 45 degrees apart
_CONTEXT_FRAMES = 4
_QPOS_DIM = 36  # root xyz, quaternion wxyz, 29 joints (MuJoCo order)
_NUM_JOINTS = 29
_ONNX_FILE = "planner_sonic.onnx"

# ONNX inputs that never change.
FIXED_INPUTS = {
    "random_seed": np.zeros(1, np.int64),
    "has_specific_target": np.zeros((1, 1), np.int64),
    "specific_target_positions": np.zeros((1, 4, 3), np.float32),
    "specific_target_headings": np.zeros((1, 4), np.float32),
    "allowed_pred_num_tokens": np.array([[1] * 6 + [0] * 5], np.int64),
}


@dataclass(frozen=True)
class KinematicPlannerInputs:
    """One plan's inputs; names follow the ADR."""

    mode: int
    movement_direction: np.ndarray  # [3] unit [cos, sin, 0], or zeros to stand
    speed: float  # m/s, clamped per mode; -1: the mode's default; 0: stopped in a ground mode
    facing_direction: np.ndarray  # [3] unit [cos, sin, 0]
    pelvis_height: float  # m; -1: the mode decides
    context_mujoco_qpos: np.ndarray  # [4, 36]

    def __post_init__(self):
        if not 0 <= self.mode < NUM_MODES:
            raise ValueError(f"mode must be in [0, {NUM_MODES}), got {self.mode}")
        for name, shape in (("movement_direction", (3,)), ("facing_direction", (3,)), ("context_mujoco_qpos", (_CONTEXT_FRAMES, _QPOS_DIM))):
            if np.shape(getattr(self, name)) != shape:
                raise ValueError(f"{name} must have shape {shape}, got {np.shape(getattr(self, name))}")


def _bin_direction(angle: float) -> float:
    """`angle` snapped to the nearest of 8 directions 45 degrees apart, in [-pi, pi] (ros2_input_handler.hpp:1138-1150)."""
    angle = math.atan2(math.sin(angle), math.cos(angle))  # normalise to [-pi, pi]
    return round(angle / _DIRECTION_BIN) * _DIRECTION_BIN


def model_feed(inputs: KinematicPlannerInputs) -> dict[str, np.ndarray]:
    """The ONNX inputs of `inputs`, with the batch dimension added."""
    return {
        "context_mujoco_qpos": np.asarray(inputs.context_mujoco_qpos, np.float32)[None],
        "target_vel": np.array([inputs.speed], np.float32),
        "mode": np.array([inputs.mode], np.int64),
        "movement_direction": np.asarray(inputs.movement_direction, np.float32)[None],
        "facing_direction": np.asarray(inputs.facing_direction, np.float32)[None],
        "height": np.array([inputs.pelvis_height], np.float32),
        **FIXED_INPUTS,
    }


def infer(session, inputs: KinematicPlannerInputs) -> np.ndarray:
    """Run the planner; returns the predicted `mujoco_qpos` frames, [n, 36] at 30 Hz."""
    qpos, num_frames = session.run(["mujoco_qpos", "num_pred_frames"], model_feed(inputs))
    n = int(num_frames[0])
    plan = qpos[0, :n]
    if n < 2 or not np.isfinite(plan).all():
        raise RuntimeError(f"kinematic planner returned an unusable plan (num_pred_frames={n})")
    return plan


@dataclass(frozen=True)
class Motion:
    """A plan resampled to 50 Hz: root position, root quaternion (wxyz), joints and joint velocities (IsaacLab order)."""

    positions: np.ndarray  # [N, 3]
    quats: np.ndarray  # [N, 4]
    joints: np.ndarray  # [N, 29]
    joint_velocities: np.ndarray  # [N, 29]

    def __len__(self) -> int:
        return len(self.positions)

    def qpos_at(self, frames: np.ndarray) -> np.ndarray:
        """The 30 Hz-style qpos rows [n, 36] (MuJoCo order) at fractional 50 Hz `frames`, clamped to the last frame."""
        f0, f1, frac = _bracket(frames, len(self))
        qpos = np.empty((len(f0), _QPOS_DIM))
        qpos[:, :3] = _lerp(self.positions, f0, f1, frac)
        qpos[:, 3:7] = quat_slerp(self.quats[f0], self.quats[f1], frac)
        qpos[:, 7:] = _lerp(self.joints, f0, f1, frac)[:, ISAACLAB_OF_MUJOCO]
        return qpos


def _bracket(frames: np.ndarray, count: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Frame indices below and above fractional `frames` (the last frame holds), and the fraction between them."""
    f0 = np.minimum(np.floor(frames).astype(int), count - 1)
    return f0, np.minimum(f0 + 1, count - 1), frames - f0


def _lerp(values: np.ndarray, f0: np.ndarray, f1: np.ndarray, frac: np.ndarray) -> np.ndarray:
    return values[f0] + frac[:, None] * (values[f1] - values[f0])


def resample_50hz(plan: np.ndarray) -> Motion:
    """A 30 Hz plan [n, 36] as a 50 Hz Motion of floor(n / 30 * 50) frames; velocities are the forward difference."""
    frames = np.arange(len(plan) * 50 // 30) * 30 / 50  # 50 Hz frame i is at 30 Hz frame i * 0.6
    f0, f1, frac = _bracket(frames, len(plan))
    joints = _lerp(plan[:, 7:], f0, f1, frac)[:, MUJOCO_OF_ISAACLAB]
    velocities = np.empty_like(joints)
    velocities[:-1] = np.diff(joints, axis=0) * MOTION_RATE
    velocities[-1] = velocities[-2]
    return Motion(_lerp(plan[:, :3], f0, f1, frac), quat_slerp(plan[f0, 3:7], plan[f1, 3:7], frac), joints, velocities)


def _splice(old: Motion, new: Motion, base: int, start: int) -> Motion | None:
    """`old` from frame `base` on, cross-faded into `new`, whose frame 0 sits at `old` frame `start`; frame 0 of the
    result is `old` frame `base`. New frames before `base` are skipped; None if none of `new` is left to play."""
    length = start - base + len(new)
    if length <= 0:
        return None
    f = np.arange(length)
    f_old = np.clip(f + base, 0, len(old) - 1)
    f_new = np.clip(f + base - start, 0, len(new) - 1)
    w = np.clip((f - max(0, start - base)) / BLEND_FRAMES, 0.0, 1.0)  # weight of the new plan, rising linearly
    w2 = w[:, None]
    return Motion(
        (1 - w2) * old.positions[f_old] + w2 * new.positions[f_new],
        quat_slerp(old.quats[f_old], new.quats[f_new], w),
        (1 - w2) * old.joints[f_old] + w2 * new.joints[f_new],
        (1 - w2) * old.joint_velocities[f_old] + w2 * new.joint_velocities[f_new],
    )


class KinematicPlanner:
    """Plans at 10 Hz of the sim time `step(dt)` is given; /cmd_vel, mode and pelvis height arrive through setters.

    planner.initialize(joints)  # blocks for the first plan
    ref = planner.reference()  # every tick: (joints [10, 29], joint velocities [10, 29], quats [10, 4]), now to +0.9 s
    planner.step(0.02)  # every tick, after reference(); replans run on a worker thread and never block
    planner.close()
    """

    def __init__(self, directory: Path = SONIC_DIR, providers: list[str] | None = None, synchronous: bool = False):
        """`synchronous` runs each replan inline and splices it at once: deterministic, for tests."""
        self._session = _session(directory / _ONNX_FILE, providers)
        self._executor = None if synchronous else ThreadPoolExecutor(max_workers=1, thread_name_prefix="kplanner")
        self._cmd = (0.0, 0.0)  # latest /cmd_vel vx, vy
        self._wz = 0.0
        self._cmd_age = math.inf  # s of sim time since the last /cmd_vel
        self._mode = IDLE
        self._pelvis_height = -1.0
        self._initialized = False
        self._elapsed = 0.0
        self._ros_subscriptions = []  # /cmd_vel and /sonic subscriptions while connected
        self._motion: Motion | None = None  # the stored plan; frame 0 is the playback frame at the last splice
        self._cursor = 0.0  # playback position in `_motion`, 50 Hz frames
        self._pending: tuple[Future, int] | None = None  # the replan in flight and its start frame in `_motion`
        self.facing_yaw = 0.0
        self.last_inputs: KinematicPlannerInputs | None = None
        self.mujoco_qpos: np.ndarray | None = None  # latest raw plan, [n, 36]
        self.plans = 0  # inferences launched, the first included
        self.late_plans = 0  # plans spliced after their start frame had played
        self.skipped_replans = 0  # 10 Hz ticks skipped because the previous plan was still in flight

    def set_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        self._cmd = (vx, vy)
        self._wz = wz
        self._cmd_age = 0.0

    def set_mode(self, mode: int) -> None:
        if not 0 <= mode < NUM_MODES:
            raise ValueError(f"mode must be in [0, {NUM_MODES}), got {mode}")
        self._mode = int(mode)

    def set_pelvis_height(self, height: float) -> None:
        self._pelvis_height = float(height)

    def initialize(self, joint_positions: np.ndarray, robot_prim_path: str | None = None) -> None:
        """Plan the first time from `joint_positions` ([29], MuJoCo order), blocking; with `robot_prim_path`, also
        subscribe to /cmd_vel, /sonic/mode and /sonic/pelvis_height of that robot."""
        qpos = np.zeros(_QPOS_DIM)
        qpos[2], qpos[3] = DEFAULT_HEIGHT, 1.0
        qpos[7:] = np.asarray(joint_positions, np.float64).reshape(_NUM_JOINTS)
        self._pending = None  # a plan still running for an earlier start is dropped when it lands
        self.facing_yaw = 0.0
        self._elapsed = 0.0
        self.plans = self.late_plans = self.skipped_replans = 0
        inputs = self._build_inputs(np.tile(qpos, (_CONTEXT_FRAMES, 1)))
        self.mujoco_qpos = infer(self._session, inputs)
        self.plans = 1
        self._motion = resample_50hz(self.mujoco_qpos)
        self._cursor = 0.0
        self._initialized = True
        if robot_prim_path is not None and not self._ros_subscriptions:
            from oc.robot.g1 import G1  # Kit only

            from .ros import subscribe_commands

            self._ros_subscriptions = [
                G1(robot_prim_path).subscribe_cmd_vel(self.set_cmd_vel),
                *subscribe_commands(robot_prim_path, self.set_mode, self.set_pelvis_height),
            ]

    def reference(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Joints, joint velocities (IsaacLab order) and root quaternions (wxyz) of FUTURE_FRAMES frames 0.1 s apart,
        frame 0 being now."""
        if not self._initialized:
            raise RuntimeError("KinematicPlanner.initialize() must run before reference()")
        self._splice_finished()
        frames = np.minimum(math.floor(self._cursor + 1e-9) + REFERENCE_STEP * np.arange(FUTURE_FRAMES), len(self._motion) - 1)
        return self._motion.joints[frames], self._motion.joint_velocities[frames], self._motion.quats[frames]

    def step(self, dt: float) -> None:
        """Integrate the facing yaw, advance the playback and, every PLANNER_PERIOD of sim time, launch a replan."""
        if not self._initialized:
            raise RuntimeError("KinematicPlanner.initialize() must run before step()")
        self._splice_finished()
        if not self._stale() and abs(self._wz) > MIN_TURN_RATE:
            self.facing_yaw += self._wz * dt  # true rad/s; the reference adds wz * 0.02 per 100 Hz tick, about 2x wz
        self._cmd_age += dt
        self._cursor = min(self._cursor + dt * MOTION_RATE, len(self._motion) - 1)
        self._elapsed += dt
        if self._elapsed >= PLANNER_PERIOD - 1e-9:  # float sums of dt fall just short of the period
            self._elapsed -= PLANNER_PERIOD
            if self._pending is not None:
                self.skipped_replans += 1
            else:
                self._launch()
                if self._executor is None:
                    self._splice_finished()

    def close(self) -> None:
        """Stop the ROS 2 subscriptions and the worker thread."""
        for subscription in self._ros_subscriptions:
            subscription.stop()
        self._ros_subscriptions = []
        self._pending = None
        if self._executor is not None:
            self._executor.shutdown(wait=True, cancel_futures=True)

    def _launch(self) -> None:
        """Start a plan from the stored motion `LOOK_AHEAD` frames past the playback frame."""
        start = math.floor(self._cursor + 1e-9) + LOOK_AHEAD
        context = self._motion.qpos_at(start + np.arange(_CONTEXT_FRAMES) * MOTION_RATE / PLANNER_RATE)
        inputs = self._build_inputs(context)  # copies: the worker only sees `inputs`
        if self._executor is None:
            future = Future()
            future.set_result(infer(self._session, inputs))
        else:
            future = self._executor.submit(infer, self._session, inputs)
        self._pending = (future, start)
        self.plans += 1

    def _splice_finished(self) -> None:
        """Cross-fade the plan in flight into the stored motion if it has finished; a worker error is raised here."""
        if self._pending is None or not self._pending[0].done():
            return
        (future, start), self._pending = self._pending, None
        plan = future.result()
        base = math.floor(self._cursor + 1e-9)
        if base > start:
            self.late_plans += 1
        self.mujoco_qpos = plan
        motion = _splice(self._motion, resample_50hz(plan), base, start)
        if motion is not None:
            self._motion = motion
            self._cursor -= base  # frame 0 is now the playback frame; the fraction stays

    def _stale(self) -> bool:
        """True when the last /cmd_vel is older than CMD_TIMEOUT; its vx, vy and wz then count as zero."""
        return self._cmd_age > CMD_TIMEOUT

    def _build_inputs(self, context: np.ndarray) -> KinematicPlannerInputs:
        vx, vy = (0.0, 0.0) if self._stale() else self._cmd
        speed = math.hypot(vx, vy)
        mode, height = self._mode, self._pelvis_height
        movement = np.zeros(3)
        if mode == IDLE:
            speed = -1.0
        elif speed < MIN_SPEED:  # stopped; self._mode keeps the user's mode, so moving again resumes it
            if mode in STANDING_MODES:  # gamepad.hpp:697-702
                mode, speed, height = IDLE, -1.0, -1.0
            else:  # gamepad.hpp:703-707
                speed = 0.0
        else:
            yaw = _bin_direction(math.atan2(vy, vx) + self.facing_yaw)  # /cmd_vel is body-relative (:760-766)
            movement = np.array([math.cos(yaw), math.sin(yaw), 0.0])
            if mode in SPEED_RANGES:
                speed = min(max(speed, SPEED_RANGES[mode][0]), SPEED_RANGES[mode][1])
            elif mode in DEFAULT_SPEED_MODES:
                speed = -1.0
        facing = np.array([math.cos(self.facing_yaw), math.sin(self.facing_yaw), 0.0])
        self.last_inputs = KinematicPlannerInputs(mode, movement, speed, facing, height, context)
        return self.last_inputs
