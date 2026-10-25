"""Constants and NumPy helpers of NVIDIA SONIC's G1 policy (default weights, huggingface.co/nvidia/GEAR-SONIC): joint
orders, default pose, PD gains and action scales, the quaternion helpers and the encoder and decoder observation
builders. Pure numpy and onnxruntime.

Follows NVlabs/GR00T-WholeBodyControl gear_sonic_deploy/src/g1/g1_deploy_onnx_ref: include/policy_parameters.hpp and
src/g1_deploy_onnx_ref.cpp.
"""

import logging
import math
from pathlib import Path

import numpy as np
import onnxruntime as ort

SONIC_DIR = Path(__file__).parents[4] / "data" / "sonic"  # pixi run sonic-download
ONNX_THREADS = 4  # onnxruntime threads per model: the fastest count for all three models; more gain nothing
FUTURE_FRAMES = 10  # reference frames 0.1 s apart, 0.9 s ahead
DEFAULT_PROVIDERS = ["CUDAExecutionProvider", "CPUExecutionProvider"]

_log = logging.getLogger(__name__)

# Hardware (MuJoCo) joint order: motors, gains and the planner's qpos.
MUJOCO_JOINTS = tuple(
    f"{side}_{joint}_joint"
    for side in ("left", "right")
    for joint in ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")
) + ("waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint") + tuple(
    f"{side}_{joint}_joint"
    for side in ("left", "right")
    for joint in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")
)
# IsaacLab joint order (breadth-first): the policy's observations and actions.
ISAACLAB_JOINTS = tuple(
    name
    for group in (
        ("left_hip_pitch", "right_hip_pitch", "waist_yaw"),
        ("left_hip_roll", "right_hip_roll", "waist_roll"),
        ("left_hip_yaw", "right_hip_yaw", "waist_pitch"),
        ("left_knee", "right_knee"),
        ("left_shoulder_pitch", "right_shoulder_pitch"),
        ("left_ankle_pitch", "right_ankle_pitch"),
        ("left_shoulder_roll", "right_shoulder_roll"),
        ("left_ankle_roll", "right_ankle_roll"),
        ("left_shoulder_yaw", "right_shoulder_yaw"),
        ("left_elbow", "right_elbow"),
        ("left_wrist_roll", "right_wrist_roll"),
        ("left_wrist_pitch", "right_wrist_pitch"),
        ("left_wrist_yaw", "right_wrist_yaw"),
    )
    for name in (f"{joint}_joint" for joint in group)
)
MUJOCO_OF_ISAACLAB = np.array([MUJOCO_JOINTS.index(name) for name in ISAACLAB_JOINTS])  # mujoco[MUJOCO_OF_ISAACLAB] -> isaaclab
ISAACLAB_OF_MUJOCO = np.array([ISAACLAB_JOINTS.index(name) for name in MUJOCO_JOINTS])  # isaaclab[ISAACLAB_OF_MUJOCO] -> mujoco

DEFAULT_POSE = np.array(  # MuJoCo order, rad
    [-0.312, 0.0, 0.0, 0.669, -0.363, 0.0] * 2 + [0.0, 0.0, 0.0] + [0.2, 0.2, 0.0, 0.6, 0.0, 0.0, 0.0] + [0.2, -0.2, 0.0, 0.6, 0.0, 0.0, 0.0]
)
DEFAULT_HEIGHT = 0.788740  # m: the planner's standing pelvis height


def _session(path: Path, providers: list[str] | None = None) -> ort.InferenceSession:
    """An onnxruntime session on ONNX_THREADS threads, on providers (None: CUDA, falling back to CPU). Inside Kit,
    onnxruntime's default thread pool (one spinning thread per core) competes with Kit's own threads and runs several
    times slower."""
    if not path.is_file():
        raise FileNotFoundError(f"SONIC model missing: {path}; run `pixi run sonic-download`")
    available = ort.get_available_providers()
    requested = [p for p in (providers or DEFAULT_PROVIDERS) if p in available] or ["CPUExecutionProvider"]
    chosen = [
        ("TensorrtExecutionProvider", {"trt_engine_cache_enable": True, "trt_engine_cache_path": str(path.parent / "trt_cache")})
        if p == "TensorrtExecutionProvider"
        else p
        for p in requested
    ]
    options = ort.SessionOptions()
    options.log_severity_level = 3
    options.intra_op_num_threads = ONNX_THREADS
    options.inter_op_num_threads = 1
    options.add_session_config_entry("session.intra_op.allow_spinning", "0")
    session = ort.InferenceSession(str(path), options, providers=chosen)
    _log.info("%s runs on %s", path.name, session.get_providers()[0])
    return session


# --- Policy (encoder + decoder) constants ---------------------------------------------------------------------------
# Source: NVlabs/GR00T-WholeBodyControl @45c03fa. CPP = gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/src/
# g1_deploy_onnx_ref.cpp; TRN = gear_sonic/envs/manager_env/robots/g1.py.
CONTROL_DT = 0.02  # s: one policy tick (50 Hz; decimation 4 of the 200 Hz physics)
HISTORY_FRAMES = 10  # decoder history length, 50 Hz steps (CPP:1454-1636)
ENCODER_OBS_DIM = 1762  # model_encoder.onnx obs_dict width
DECODER_OBS_DIM = 994  # model_decoder.onnx obs_dict width
NUM_JOINTS = len(MUJOCO_JOINTS)  # 29

# Per joint kind: (armature kg m^2, effort limit Nm, velocity limit rad/s). TRN:238-357; policy_parameters.hpp:39-65.
_HIP_KNEE = (0.025102, 139.0, 20.0)
_YAW = (0.010178, 88.0, 32.0)  # hip_yaw, waist_yaw
_ANKLE_WAIST = (0.007219, 50.0, 37.0)  # ankle_pitch/roll, waist_roll/pitch
_ARM = (0.003610, 25.0, 37.0)  # shoulder, elbow, wrist_roll
_WRIST = (0.004250, 5.0, 22.0)  # wrist_pitch, wrist_yaw
_JOINT_KINDS = {
    "hip_pitch": _HIP_KNEE, "hip_roll": _HIP_KNEE, "knee": _HIP_KNEE, "hip_yaw": _YAW,
    "ankle_pitch": _ANKLE_WAIST, "ankle_roll": _ANKLE_WAIST,
    "waist_yaw": _YAW, "waist_roll": _ANKLE_WAIST, "waist_pitch": _ANKLE_WAIST,
    "shoulder_pitch": _ARM, "shoulder_roll": _ARM, "shoulder_yaw": _ARM, "elbow": _ARM, "wrist_roll": _ARM,
    "wrist_pitch": _WRIST, "wrist_yaw": _WRIST,
}  # fmt: skip
_PD_NATURAL_FREQ = 2.0 * math.pi * 10.0  # rad/s: 10 Hz, critically damped with ratio 2 (TRN:238-357)


def _joint_kind(name: str) -> tuple[float, float, float]:
    return _JOINT_KINDS[name.removeprefix("left_").removeprefix("right_").removesuffix("_joint")]


# MuJoCo order, rad units.
ARMATURE, EFFORT_LIMIT, VELOCITY_LIMIT = (np.array([_joint_kind(n)[i] for n in MUJOCO_JOINTS]) for i in range(3))
STIFFNESS = ARMATURE * _PD_NATURAL_FREQ**2  # kp
DAMPING = 4.0 * ARMATURE * _PD_NATURAL_FREQ  # kd
ACTION_SCALE = 0.25 * EFFORT_LIMIT / STIFFNESS  # TRN:360-371; CPP:3137-3168

# The same arrays in IsaacLab order, which the policy uses.
DEFAULT_POSE_IL = DEFAULT_POSE[MUJOCO_OF_ISAACLAB]
ACTION_SCALE_IL = ACTION_SCALE[MUJOCO_OF_ISAACLAB]
STIFFNESS_IL = STIFFNESS[MUJOCO_OF_ISAACLAB]
DAMPING_IL = DAMPING[MUJOCO_OF_ISAACLAB]
EFFORT_LIMIT_IL = EFFORT_LIMIT[MUJOCO_OF_ISAACLAB]
VELOCITY_LIMIT_IL = VELOCITY_LIMIT[MUJOCO_OF_ISAACLAB]
ARMATURE_IL = ARMATURE[MUJOCO_OF_ISAACLAB]

# Encoder obs_dict slots (CPP:2031,2096-2118). Everything else stays zero.
_ENC_MODE = slice(0, 4)  # encoder_mode_4: mode g1 is id 0
_ENC_JOINTS = slice(4, 294)  # motion_joint_positions_10frame_step5, frame-major
_ENC_VELS = slice(294, 584)  # motion_joint_velocities_10frame_step5
_ENC_ANCHOR = slice(601, 661)  # motion_anchor_orientation_10frame_step5, 6 per frame

# Decoder obs_dict terms in order, term-major with HISTORY_FRAMES frames each, oldest first (CPP:1454-1636).
_DEC_TOKEN, _DEC_ANG_VEL, _DEC_JOINT_POS, _DEC_JOINT_VEL, _DEC_LAST_ACTION, _DEC_GRAVITY = range(6)


# Quaternions are [w, x, y, z].
def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])  # fmt: skip


def quat_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_slerp(q0: np.ndarray, q1: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Shortest-path slerp from `q0` to `q1` ([..., 4]) by `t` ([...]); math_utils.hpp quat_slerp_d."""
    q0, q1, t = np.asarray(q0, float), np.asarray(q1, float), np.asarray(t, float)[..., None]
    dot = np.sum(q0 * q1, axis=-1, keepdims=True)
    q1 = np.where(dot < 0.0, -q1, q1)
    dot = np.abs(dot)
    close = dot > 0.9995  # nearly equal: lerp, avoiding a division by ~0
    theta = np.arccos(np.minimum(dot, 1.0))
    sin_theta = np.where(close, 1.0, np.sin(theta))
    slerp = (np.sin((1.0 - t) * theta) * q0 + np.sin(t * theta) * q1) / sin_theta
    out = np.where(close, q0 + t * (q1 - q0), slerp)
    return out / np.linalg.norm(out, axis=-1, keepdims=True)


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])  # fmt: skip


def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    return quat_to_matrix(q) @ np.asarray(v, dtype=float)


def quat_yaw(q: np.ndarray) -> np.ndarray:
    """The rotation about +z alone: yaw is the heading of the rotated x axis."""
    x_axis = quat_rotate(q, [1.0, 0.0, 0.0])
    half = 0.5 * math.atan2(x_axis[1], x_axis[0])
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)])


def delta_heading(q_root_init: np.ndarray, q_ref_init: np.ndarray) -> np.ndarray:
    """Maps the planner's frame (its heading at init) onto the world (CPP:559-606,3329-3331)."""
    return quat_mul(quat_yaw(q_root_init), quat_conj(quat_yaw(q_ref_init)))


def encoder_obs(
    ref_joints: np.ndarray, ref_vels: np.ndarray, ref_quats: np.ndarray, q_root: np.ndarray, heading: np.ndarray
) -> np.ndarray:
    """The [1762] encoder input (CPP:617-695,772-781,849-858). Joints are IsaacLab order, [FUTURE_FRAMES, 29]."""
    obs = np.zeros(ENCODER_OBS_DIM, dtype=np.float32)
    obs[_ENC_JOINTS] = np.asarray(ref_joints).reshape(-1)
    obs[_ENC_VELS] = np.asarray(ref_vels).reshape(-1)
    anchor = np.empty((FUTURE_FRAMES, 6))
    for k in range(FUTURE_FRAMES):
        rot = quat_to_matrix(quat_mul(quat_conj(q_root), quat_mul(heading, ref_quats[k])))
        anchor[k] = rot[:, :2].reshape(-1)  # R00, R01, R10, R11, R20, R21
    obs[_ENC_ANCHOR] = anchor.reshape(-1)
    return obs


def gravity_dir(q_root: np.ndarray) -> np.ndarray:
    """World gravity in the root frame; upright is (0, 0, -1) (CPP:1626-1631)."""
    return quat_rotate(quat_conj(q_root), [0.0, 0.0, -1.0])


def ang_vel_body(q_root: np.ndarray, ang_vel_world: np.ndarray) -> np.ndarray:
    return quat_to_matrix(q_root).T @ np.asarray(ang_vel_world, dtype=float)


class DecoderHistories:
    """The decoder's five state histories: HISTORY_FRAMES frames each, oldest first. The first push after
    reset() fills every frame, as IsaacLab's CircularBuffer does in training."""

    _WIDTHS = (3, NUM_JOINTS, NUM_JOINTS, NUM_JOINTS, 3)  # ang_vel, joint_pos, joint_vel, last_action, gravity

    def __init__(self) -> None:
        self._buffers = [np.zeros((HISTORY_FRAMES, w), dtype=np.float32) for w in self._WIDTHS]
        self._filled = False

    def reset(self) -> None:
        self._filled = False

    def push(self, *terms: np.ndarray) -> None:
        for buffer, term in zip(self._buffers, terms, strict=True):
            if self._filled:
                buffer[:-1] = buffer[1:]
                buffer[-1] = term
            else:
                buffer[:] = term
        self._filled = True

    def flat(self, index: int) -> np.ndarray:
        return self._buffers[index].reshape(-1)


def decoder_obs(token: np.ndarray, histories: DecoderHistories) -> np.ndarray:
    """The [994] decoder input: token | ang_vel | q - default | dq | last_action | gravity."""
    return np.concatenate(
        [np.asarray(token, dtype=np.float32).reshape(-1)] + [histories.flat(i) for i in range(5)]
    ).astype(np.float32)


def action_to_target(action: np.ndarray) -> np.ndarray:
    """Joint position targets from a raw decoder action, IsaacLab order; no clipping (CPP:3137-3168)."""
    return DEFAULT_POSE_IL + np.asarray(action, dtype=float) * ACTION_SCALE_IL
