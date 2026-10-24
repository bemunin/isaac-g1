"""NVIDIA SONIC's G1 tracking policy (default weights, huggingface.co/nvidia/GEAR-SONIC): an encoder turns a
reference motion into a motion token, a decoder turns the token and the robot's state history into joint targets.

Constants and observation layouts follow NVlabs/GR00T-WholeBodyControl gear_sonic_deploy/src/g1/g1_deploy_onnx_ref:
include/policy_parameters.hpp, src/g1_deploy_onnx_ref.cpp and policy/release/observation_config.yaml.
"""

import math
from collections import deque
from pathlib import Path

import numpy as np
import onnxruntime as ort

SONIC_DIR = Path(__file__).parents[4] / "data" / "sonic"  # pixi run sonic-download
CONTROL_DT = 0.02  # s: the policy runs at 50 Hz
ONNX_THREADS = 4  # onnxruntime threads per model: the fastest count for all three models; more gain nothing
HISTORY = 10  # frames of robot state history, oldest first
FUTURE_FRAMES, FUTURE_STEP = 10, 5  # reference frames 0.1 s apart, 0.9 s ahead

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

# Motors: armature [kg m^2], effort limit [N m]. kp = armature w^2, kd = 2 zeta armature w, w = 10 Hz, zeta = 2.
_OMEGA = 10 * 2.0 * math.pi
_MOTORS = {"7520_22": (0.025101925, 139.0), "7520_14": (0.010177520, 88.0), "5020": (0.003609725, 25.0), "4010": (0.00425, 5.0)}
_LEG = ("7520_22", "7520_22", "7520_14", "7520_22", "5020x2", "5020x2")
_ARM = ("5020", "5020", "5020", "5020", "5020", "4010", "4010")
_JOINT_MOTORS = _LEG + _LEG + ("7520_14", "5020x2", "5020x2") + _ARM + _ARM  # MuJoCo order; x2: two motors in parallel


def _motor(name: str) -> tuple[float, float, float]:
    """(armature, effort limit, action scale) of a joint's motor; action scale = 0.25 effort / kp of one motor."""
    armature, effort = _MOTORS[name.removesuffix("x2")]
    scale = 0.25 * effort / (armature * _OMEGA**2)
    count = 2 if name.endswith("x2") else 1
    return count * armature, count * effort, scale


ARMATURE, EFFORT_LIMIT, ACTION_SCALE = (np.array(values) for values in zip(*(_motor(m) for m in _JOINT_MOTORS)))
KP = ARMATURE * _OMEGA**2
KD = 2.0 * 2.0 * ARMATURE * _OMEGA
DEFAULT_POSE = np.array(  # MuJoCo order, rad
    [-0.312, 0.0, 0.0, 0.669, -0.363, 0.0] * 2 + [0.0, 0.0, 0.0] + [0.2, 0.2, 0.0, 0.6, 0.0, 0.0, 0.0] + [0.2, -0.2, 0.0, 0.6, 0.0, 0.0, 0.0]
)
DEFAULT_HEIGHT = 0.788740  # m: the planner's standing pelvis height

# Encoder input: every term of observation_config.yaml in order; the g1 mode (0) fills four of them, the rest stay zero.
ENCODER_TERMS = (
    ("encoder_mode_4", 4),
    ("motion_joint_positions_10frame_step5", 290),
    ("motion_joint_velocities_10frame_step5", 290),
    ("motion_root_z_position_10frame_step5", 10),
    ("motion_root_z_position", 1),
    ("motion_anchor_orientation", 6),
    ("motion_anchor_orientation_10frame_step5", 60),
    ("motion_joint_positions_lowerbody_10frame_step5", 120),
    ("motion_joint_velocities_lowerbody_10frame_step5", 120),
    ("vr_3point_local_target", 9),
    ("vr_3point_local_orn_target", 12),
    ("smpl_joints_10frame_step1", 720),
    ("smpl_anchor_orientation_10frame_step1", 60),
    ("motion_joint_positions_wrists_10frame_step1", 60),
)
ENCODER_SLICES = {}
_offset = 0
for _name, _size in ENCODER_TERMS:
    ENCODER_SLICES[_name] = slice(_offset, _offset + _size)
    _offset += _size
ENCODER_SIZE = _offset  # 1762
DECODER_SIZE = 64 + HISTORY * (3 + 29 + 29 + 29 + 3)  # 994: token, gyro, q - default, dq, last action, gravity
G1_MODE = 0


# Quaternions are [w, x, y, z].
def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate the vector v by q."""
    w, u = q[0], q[1:]
    t = 2.0 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


def quat_slerp(a: np.ndarray, b: np.ndarray, t: float) -> np.ndarray:
    dot = float(np.dot(a, b))
    if dot < 0.0:
        b, dot = -b, -dot
    if dot > 0.9995:
        q = a + t * (b - a)
        return q / np.linalg.norm(q)
    theta = math.acos(dot)
    return (math.sin((1 - t) * theta) * a + math.sin(t * theta) * b) / math.sin(theta)


def yaw_of(q: np.ndarray) -> float:
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_quat(yaw: float) -> np.ndarray:
    return np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])


def rotation_6d(q: np.ndarray) -> np.ndarray:
    """First two columns of q's rotation matrix, flattened row by row."""
    w, x, y, z = q
    return np.array(
        [
            1 - 2 * (y * y + z * z), 2 * (x * y - w * z),
            2 * (x * y + w * z), 1 - 2 * (x * x + z * z),
            2 * (x * z - w * y), 2 * (y * z + w * x),
        ]
    )


def _session(path: Path) -> ort.InferenceSession:
    """An onnxruntime CPU session on ONNX_THREADS threads. Inside Kit, onnxruntime's default thread pool (one spinning
    thread per core) competes with Kit's own threads and runs several times slower."""
    if not path.is_file():
        raise FileNotFoundError(f"SONIC model missing: {path}; run `pixi run sonic-download`")
    options = ort.SessionOptions()
    options.log_severity_level = 3
    options.intra_op_num_threads = ONNX_THREADS
    options.inter_op_num_threads = 1
    options.add_session_config_entry("session.intra_op.allow_spinning", "0")
    return ort.InferenceSession(str(path), options, providers=["CPUExecutionProvider"])


class SonicPolicy:
    """Tracks a reference motion: each act() (50 Hz) encodes the reference's next 0.9 s, then decodes joint targets.

    act takes the robot's joints in MuJoCo order and its pelvis IMU, and returns joint position targets in MuJoCo order.
    """

    def __init__(self, directory: Path = SONIC_DIR) -> None:
        self._encoder = _session(directory / "model_encoder.onnx")
        self._decoder = _session(directory / "model_decoder.onnx")
        self._history = None
        self._last_action = np.zeros(29)
        self._heading = np.array([1.0, 0.0, 0.0, 0.0])

    def reset(self, base_quat: np.ndarray, reference_quat: np.ndarray) -> None:
        """Forget the history, and align the reference's heading reference_quat with the robot's base_quat."""
        self._history = None
        self._last_action[:] = 0.0
        # g1_deploy_onnx_ref.cpp ComputeApplyDeltaHeading: robot heading at start x inverse reference heading at start.
        self._heading = quat_mul(yaw_quat(yaw_of(base_quat)), yaw_quat(-yaw_of(reference_quat)))

    def act(self, q, dq, base_quat, gyro, reference) -> np.ndarray:
        """Joint targets [rad, MuJoCo order] for joints q, dq [MuJoCo order], pelvis base_quat (world) and gyro
        (body frame), tracking reference = (joint positions [10, 29], joint velocities [10, 29], root quats [10, 4]),
        the 10 future frames 0.1 s apart, joints in IsaacLab order."""
        q, dq = np.asarray(q, float), np.asarray(dq, float)
        base_quat = np.asarray(base_quat, float)
        frame = (
            np.asarray(gyro, float),
            (q - DEFAULT_POSE)[MUJOCO_OF_ISAACLAB],
            dq[MUJOCO_OF_ISAACLAB],
            self._last_action.copy(),
            quat_rotate(quat_conjugate(base_quat), np.array([0.0, 0.0, -1.0])),
        )
        if self._history is None:
            self._history = [deque([term] * HISTORY, maxlen=HISTORY) for term in frame]
        else:
            for history, term in zip(self._history, frame):
                history.append(term)

        token = self.encode(base_quat, reference)
        observation = np.concatenate([token, *(np.concatenate(history) for history in self._history)])
        action = self._decoder.run(None, {"obs_dict": observation.astype(np.float32)[None]})[0].reshape(-1)
        if action.shape != (29,) or not np.isfinite(action).all():
            raise ValueError(f"SonicPolicy: decoder returned an invalid action {action}")
        self._last_action[:] = action
        return DEFAULT_POSE + action[ISAACLAB_OF_MUJOCO] * ACTION_SCALE

    def encode(self, base_quat: np.ndarray, reference) -> np.ndarray:
        """The 64-wide motion token of reference in the g1 encoder mode."""
        positions, velocities, quats = reference
        observation = np.zeros(ENCODER_SIZE)
        observation[ENCODER_SLICES["encoder_mode_4"]][0] = G1_MODE
        observation[ENCODER_SLICES["motion_joint_positions_10frame_step5"]] = np.asarray(positions).reshape(-1)
        observation[ENCODER_SLICES["motion_joint_velocities_10frame_step5"]] = np.asarray(velocities).reshape(-1)
        inverse_base = quat_conjugate(base_quat)
        observation[ENCODER_SLICES["motion_anchor_orientation_10frame_step5"]] = np.concatenate(
            [rotation_6d(quat_mul(inverse_base, quat_mul(self._heading, quat))) for quat in quats]
        )
        return self._encoder.run(None, {"obs_dict": observation.astype(np.float32)[None]})[0].reshape(-1)
