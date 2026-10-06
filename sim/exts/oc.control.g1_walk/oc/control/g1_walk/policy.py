"""Unitree's pretrained G1 walking policy (unitree_rl_gym motion.pt).

Constants come from unitree_rl_gym deploy/deploy_real/configs/g1.yaml and legged_gym/envs/g1/g1_config.py.
"""

from pathlib import Path

import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import torch
import warp as wp

POLICY_PATH = Path(__file__).parents[3] / "data" / "motion.pt"

LEG_JOINTS = tuple(
    f"{side}_{joint}_joint"
    for side in ("left", "right")
    for joint in ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")
)
LEG_DEFAULT = np.array([-0.1, 0.0, 0.0, 0.3, -0.2, 0.0] * 2, dtype=np.float32)
LEG_KP = (100, 100, 100, 150, 40, 40) * 2
LEG_KD = (2, 2, 2, 4, 2, 2) * 2

# Waist and arms are held stiffly at zero: the policy was trained on g1_12dof, whose upper body is rigid.
# Unitree's real-robot gains (kp 20-300) let the upper body sway, and the policy then runs away and falls.
UPPER_JOINTS = ("waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint") + tuple(
    f"{side}_{joint}_joint"
    for side in ("left", "right")
    for joint in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")
)
UPPER_KP = (5000,) * len(UPPER_JOINTS)
UPPER_KD = (200,) * len(UPPER_JOINTS)

JOINTS = LEG_JOINTS + UPPER_JOINTS
DEFAULT_POSE = np.concatenate([LEG_DEFAULT, np.zeros(len(UPPER_JOINTS), np.float32)])
KP = LEG_KP + UPPER_KP
KD = LEG_KD + UPPER_KD

ANG_VEL_SCALE = 0.25
DOF_VEL_SCALE = 0.05
ACTION_SCALE = 0.25
CMD_SCALE = np.array([2.0, 2.0, 0.25], dtype=np.float32)
GAIT_PERIOD = 0.8  # seconds


def to_body_frame(wxyz: np.ndarray, vector: np.ndarray) -> np.ndarray:
    """Rotate a world-frame vector into the frame of the body with orientation wxyz."""
    w, u = wxyz[0], -wxyz[1:]
    t = 2.0 * np.cross(u, vector)
    return vector + w * t + np.cross(u, t)


class _Policy:
    """Turns the estimated G1 state and a [vx, vy, wz] command into joint position targets. Call act at 50 Hz.

    The estimated state's root orientation is world-frame wxyz; its angular velocity is body-frame (IMU gyro).
    """

    def __init__(self, policy_path: Path = POLICY_PATH) -> None:
        self._policy = torch.jit.load(str(policy_path), map_location="cpu").eval()
        self._last_action = np.zeros(len(LEG_JOINTS), dtype=np.float32)

    def reset(self) -> None:
        self._last_action[:] = 0.0
        self._policy.hidden_state.zero_()
        self._policy.cell_state.zero_()

    def act(self, estimated_state: mg.RobotState, command, t: float) -> mg.RobotState:
        observation = self.observation(estimated_state, command, t)
        with torch.inference_mode():
            action = self._policy(torch.from_numpy(observation).unsqueeze(0)).numpy().reshape(-1)
        if action.shape != self._last_action.shape or not np.isfinite(action).all():
            raise ValueError(f"G1WalkController: policy returned an invalid action {action}")
        self._last_action[:] = action

        targets = DEFAULT_POSE.copy()
        targets[: len(LEG_JOINTS)] += ACTION_SCALE * action
        joints = mg.JointState.from_name(
            estimated_state.joints.robot_joint_space,
            positions=(list(JOINTS), wp.array(targets, dtype=wp.float32, device="cpu")),
        )
        return mg.RobotState(joints=joints)

    def observation(self, estimated_state: mg.RobotState, command, t: float) -> np.ndarray:
        """Build the 47-wide policy input."""
        joints, root = estimated_state.joints, estimated_state.root
        dof_names = joints.robot_joint_space
        legs = [dof_names.index(name) for name in LEG_JOINTS]
        q = joints.positions.numpy().reshape(-1)[legs]
        dq = joints.velocities.numpy().reshape(-1)[legs]

        orientation = root.orientation.numpy().astype(np.float32)
        angular_velocity = root.angular_velocity.numpy().astype(np.float32)
        gravity = to_body_frame(orientation, np.array([0.0, 0.0, -1.0], dtype=np.float32))

        phase = 2.0 * np.pi * (t % GAIT_PERIOD) / GAIT_PERIOD
        return np.concatenate(
            [
                angular_velocity * ANG_VEL_SCALE,
                gravity,
                np.asarray(command, dtype=np.float32) * CMD_SCALE,
                q - LEG_DEFAULT,
                dq * DOF_VEL_SCALE,
                self._last_action,
                [np.sin(phase), np.cos(phase)],
            ]
        ).astype(np.float32)
