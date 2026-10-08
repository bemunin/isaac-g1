"""SONIC's kinematic planner (planner_sonic.onnx): turns a gait mode, a movement direction, a facing direction and a
speed into the whole-body reference motion that SonicPolicy tracks.

Ports the C++ runtime's planner handling (NVlabs/GR00T-WholeBodyControl gear_sonic_deploy: localmotion_kplanner.hpp,
g1_deploy_onnx_ref.cpp CurrentFrameAdvancement and the planner thread; docs/source/references/planner_onnx.md):
replan at most at 10 Hz, 30 -> 50 Hz resampling, an 8-frame cross-fade into the current motion.
"""

import math
from pathlib import Path

import numpy as np

from .sonic import (
    DEFAULT_HEIGHT,
    DEFAULT_POSE,
    FUTURE_FRAMES,
    FUTURE_STEP,
    ISAACLAB_OF_MUJOCO,
    MUJOCO_OF_ISAACLAB,
    SONIC_DIR,
    _session,
    quat_slerp,
)

IDLE, SLOW_WALK, WALK, RUN = 0, 1, 2, 3  # planner modes used here (planner_onnx.md lists 27)
PLANNER_PERIOD = 5  # control steps: the planner runs at 10 Hz
LOOK_AHEAD = 2  # control steps: a new plan starts this far ahead of the current frame
BLEND_FRAMES = 8
REPLAN_INTERVAL = {RUN: 0.1}  # s: periodic replan while moving; others 1.0 s
DEFAULT_REPLAN_INTERVAL = 1.0
FACING_CHANGE = math.radians(1.0)  # rad: a smaller facing change doesn't replan
DIRECTION_CHANGE = math.radians(5.0)  # rad
SPEED_CHANGE = 0.05  # m/s
CONTROL_RATE, PLANNER_RATE = 50.0, 30.0  # Hz


def _angle_between(a: float, b: float) -> float:
    return abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)


class Motion:
    """A whole-body reference motion at 50 Hz: root position [N, 3], root quaternion wxyz [N, 4] and joint positions and
    velocities [N, 29] in IsaacLab order, all in the planner's frame."""

    def __init__(self, positions, quats, joints, joint_velocities) -> None:
        self.positions, self.quats, self.joints, self.joint_velocities = positions, quats, joints, joint_velocities

    def __len__(self) -> int:
        return len(self.joints)

    def sample(self, frame: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """(root position, root quaternion, joints) at a fractional frame, clamped to the motion."""
        frame = min(max(frame, 0.0), len(self) - 1.0)
        f0 = math.floor(frame)
        f1 = min(f0 + 1, len(self) - 1)
        w = frame - f0
        return (
            (1 - w) * self.positions[f0] + w * self.positions[f1],
            quat_slerp(self.quats[f0], self.quats[f1], w),
            (1 - w) * self.joints[f0] + w * self.joints[f1],
        )


class Planner:
    """Keeps the reference motion SonicPolicy tracks, replanning it from the latest command.

    Call reset() once, then each control step: command() (any time), reference() for the policy, then step().
    Directions and facing are yaw angles in the planner's frame, which starts at the origin facing +x.
    """

    def __init__(self, directory: Path = SONIC_DIR) -> None:
        self._session = _session(directory / "planner_sonic.onnx")
        self.motion = None
        self.frame = 0  # current frame of motion
        self._tick = 0
        self._command = (IDLE, None, 0.0, 0.0)  # mode, movement yaw (None: standing), facing yaw, speed [m/s]
        self._planned = None
        self._since_replan = 0.0

    @property
    def mode(self) -> int:
        return self._command[0]

    def reset(self) -> None:
        """Start standing at the origin facing +x."""
        context = np.zeros((4, 36), np.float32)
        context[:, 2] = DEFAULT_HEIGHT
        context[:, 3] = 1.0
        context[:, 7:] = DEFAULT_POSE
        self._command = (IDLE, None, 0.0, 0.0)
        self.motion = self._plan(context, self._command)
        self.frame = 0
        self._tick = 0
        self._planned = self._command
        self._since_replan = 0.0

    def command(self, mode: int, movement_yaw: float | None, facing_yaw: float, speed: float) -> None:
        """Move in mode toward movement_yaw (None: stand) at speed [m/s, <= 0: the mode's default], facing facing_yaw."""
        self._command = (mode, None if mode == IDLE else movement_yaw, facing_yaw, speed if mode != IDLE else 0.0)

    def reference(self):
        """The policy's reference: joint positions [10, 29], joint velocities [10, 29] and root quats [10, 4] of
        the 10 frames 0.1 s apart from the current one, joints in IsaacLab order."""
        frames = np.minimum(self.frame + FUTURE_STEP * np.arange(FUTURE_FRAMES), len(self.motion) - 1)
        return self.motion.joints[frames], self.motion.joint_velocities[frames], self.motion.quats[frames]

    @property
    def settled(self) -> bool:
        """True once the current motion has played out to its last frame."""
        return self.frame >= len(self.motion) - 1

    def root_quat(self) -> np.ndarray:
        return self.motion.quats[self.frame]

    def step(self) -> None:
        """Advance one control step, replanning first on a planner tick if the command needs it."""
        if self._tick % PLANNER_PERIOD == 0:
            self._since_replan += PLANNER_PERIOD / CONTROL_RATE
            if self._needs_replan():
                self._replan()
        self._tick += 1
        self.frame = min(self.frame + 1, len(self.motion) - 1)

    def _needs_replan(self) -> bool:
        mode, movement, facing, speed = self._command
        old_mode, old_movement, old_facing, old_speed = self._planned
        if mode != old_mode or _angle_between(facing, old_facing) > FACING_CHANGE:
            return True
        if mode == IDLE:
            return False
        if abs(speed - old_speed) > SPEED_CHANGE:
            return True
        if old_movement is None or _angle_between(movement, old_movement) > DIRECTION_CHANGE:
            return True
        return self._since_replan >= REPLAN_INTERVAL.get(mode, DEFAULT_REPLAN_INTERVAL)

    def _replan(self) -> None:
        start = self.frame + LOOK_AHEAD
        context = np.zeros((4, 36), np.float32)
        for n in range(4):  # 4 frames at 30 Hz from the start frame
            position, quat, joints = self.motion.sample(start + n * CONTROL_RATE / PLANNER_RATE)
            context[n, :3], context[n, 3:7], context[n, 7:] = position, quat, joints[ISAACLAB_OF_MUJOCO]
        new = self._plan(context, self._command)
        self.motion = self._blend(new, start)
        self.frame = 0
        self._planned = self._command
        self._since_replan = 0.0

    def _plan(self, context: np.ndarray, command) -> Motion:
        mode, movement, facing, speed = command
        moving = np.zeros(3, np.float32) if movement is None else np.array([math.cos(movement), math.sin(movement), 0.0], np.float32)
        allowed_tokens = np.zeros((1, 11), np.int64)
        allowed_tokens[0, :6] = 1  # localmotion_kplanner_onnx.hpp
        qpos, count = self._session.run(
            None,
            {
                "context_mujoco_qpos": context[None],
                "target_vel": np.array([speed if speed > 0 else -1.0], np.float32),
                "mode": np.array([mode], np.int64),
                "movement_direction": moving[None],
                "facing_direction": np.array([[math.cos(facing), math.sin(facing), 0.0]], np.float32),
                "random_seed": np.array([0], np.int64),
                "has_specific_target": np.zeros((1, 1), np.int64),
                "specific_target_positions": np.zeros((1, 4, 3), np.float32),
                "specific_target_headings": np.zeros((1, 4), np.float32),
                "allowed_pred_num_tokens": allowed_tokens,
                "height": np.array([-1.0], np.float32),
            },
        )
        qpos = qpos[0, : int(count[0])].astype(float)
        if len(qpos) < 2 or not np.isfinite(qpos).all():
            raise ValueError("Planner: the planner returned an invalid motion")
        return _resample(qpos)

    def _blend(self, new: Motion, start: int) -> Motion:
        """The current motion from the current frame on, cross-faded into new, which starts at frame start."""
        old = self.motion
        length = start - self.frame + len(new)
        blend_start = max(0, start - self.frame)
        f = np.arange(length)
        f_old = np.clip(f + self.frame, 0, len(old) - 1)
        f_new = np.clip(f + self.frame - start, 0, len(new) - 1)
        w = np.clip((f - blend_start) / BLEND_FRAMES, 0.0, 1.0)[:, None]
        quats = np.array([quat_slerp(old.quats[a], new.quats[b], float(t)) for a, b, t in zip(f_old, f_new, w[:, 0])])
        return Motion(
            (1 - w) * old.positions[f_old] + w * new.positions[f_new],
            quats,
            (1 - w) * old.joints[f_old] + w * new.joints[f_new],
            (1 - w) * old.joint_velocities[f_old] + w * new.joint_velocities[f_new],
        )


def _resample(qpos: np.ndarray) -> Motion:
    """A 50 Hz Motion from the planner's 30 Hz MuJoCo qpos [N, 36] (root xyz, wxyz, 29 joints)."""
    length = math.floor(len(qpos) / PLANNER_RATE * CONTROL_RATE)
    t = np.arange(length) * PLANNER_RATE / CONTROL_RATE
    f0 = np.floor(t).astype(int)
    f1 = np.minimum(f0 + 1, len(qpos) - 1)
    w = (t - f0)[:, None]
    positions = (1 - w) * qpos[f0, :3] + w * qpos[f1, :3]
    quats = np.array([quat_slerp(qpos[a, 3:7], qpos[b, 3:7], float(c)) for a, b, c in zip(f0, f1, w[:, 0])])
    joints = ((1 - w) * qpos[f0, 7:] + w * qpos[f1, 7:])[:, MUJOCO_OF_ISAACLAB]
    velocities = np.empty_like(joints)
    velocities[:-1] = (joints[1:] - joints[:-1]) * CONTROL_RATE
    velocities[-1] = velocities[-2]
    return Motion(positions, quats, joints, velocities)
