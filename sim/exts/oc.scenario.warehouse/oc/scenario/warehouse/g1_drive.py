"""Sets up the G1 for SONIC's joint position targets: on each Play, its joint gains, armatures and effort limits and its
standing pose. The targets themselves come over ROS 2 (/joint_commands) from the oc_sonic nodes."""

import math

import numpy as np
import omni.timeline
import omni.usd
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager

# These mirror ros_ws/src/oc_sonic's sonic_common tables (sonic.py of the former oc.control.g1_sonic); keep them
# identical. MuJoCo joint order. Motors: armature [kg m^2], effort limit [N m]. kp = armature w^2, kd = 2 zeta armature w,
# w = 10 Hz, zeta = 2.
_LEGS = ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")
_ARMS = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")
MUJOCO_JOINTS = (
    *(f"{side}_{joint}_joint" for side in ("left", "right") for joint in _LEGS),
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    *(f"{side}_{joint}_joint" for side in ("left", "right") for joint in _ARMS),
)
_OMEGA = 10 * 2.0 * math.pi
_MOTORS = {"7520_22": (0.025101925, 139.0), "7520_14": (0.010177520, 88.0), "5020": (0.003609725, 25.0), "4010": (0.00425, 5.0)}
_LEG = ("7520_22", "7520_22", "7520_14", "7520_22", "5020x2", "5020x2")
_ARM = ("5020", "5020", "5020", "5020", "5020", "4010", "4010")
_JOINT_MOTORS = _LEG + _LEG + ("7520_14", "5020x2", "5020x2") + _ARM + _ARM  # x2: two motors in parallel


def _motor(name: str) -> tuple[float, float]:
    armature, effort = _MOTORS[name.removesuffix("x2")]
    count = 2 if name.endswith("x2") else 1
    return count * armature, count * effort


ARMATURE, EFFORT_LIMIT = (np.array(values) for values in zip(*(_motor(m) for m in _JOINT_MOTORS)))
KP = ARMATURE * _OMEGA**2
KD = 2.0 * 2.0 * ARMATURE * _OMEGA
DEFAULT_POSE = np.array(  # rad
    [-0.312, 0.0, 0.0, 0.669, -0.363, 0.0] * 2 + [0.0, 0.0, 0.0] + [0.2, 0.2, 0.0, 0.6, 0.0, 0.0, 0.0] + [0.2, -0.2, 0.0, 0.6, 0.0, 0.0, 0.0]
)


class G1Drive:
    """Binds to the G1 at prim_path on the first physics step after each Play and applies the drive setup: gains,
    armatures, effort limits, the standing pose as position and target, and zero velocities."""

    def __init__(self, prim_path: str) -> None:
        self.prim_path = prim_path
        self._timeline_subscriptions = []
        self._step_callback_id = None

    def start(self) -> None:
        # Timeline subscriptions survive stage loads; SimulationManager callbacks are cleared on each one.
        if not self._timeline_subscriptions:
            timeline = omni.timeline.get_timeline_interface()
            stream = timeline.get_timeline_event_stream()
            self._timeline_subscriptions = [
                stream.create_subscription_to_pop_by_type(int(omni.timeline.TimelineEventType.PLAY), self._on_play),
                stream.create_subscription_to_pop_by_type(int(omni.timeline.TimelineEventType.STOP), self._on_stop),
            ]
            if timeline.is_playing():
                self._on_play(None)

    def stop(self) -> None:
        self._timeline_subscriptions = []
        self._on_stop(None)

    def _on_play(self, event) -> None:
        if self._step_callback_id is None:
            self._step_callback_id = SimulationManager.register_callback(self._on_physics_step, SimulationEvent.PHYSICS_POST_STEP)

    def _on_stop(self, event) -> None:
        if self._step_callback_id is not None:
            SimulationManager.deregister_callback(self._step_callback_id)
            self._step_callback_id = None

    def _on_physics_step(self, dt: float, context) -> None:
        if not omni.usd.get_context().get_stage().GetPrimAtPath(self.prim_path).IsValid():
            return
        articulation = Articulation(self.prim_path)
        dof_names = list(articulation.dof_names)
        dof_indices = [dof_names.index(name) for name in MUJOCO_JOINTS]
        articulation.set_dof_gains(np.array([KP]), np.array([KD]), dof_indices=dof_indices)
        articulation.set_dof_armatures(np.array([ARMATURE]), dof_indices=dof_indices)
        articulation.set_dof_max_efforts(np.array([EFFORT_LIMIT]), dof_indices=dof_indices)
        # Start from SONIC's standing pose at rest; the asset's authored state carries stale velocities.
        articulation.set_dof_positions(np.array([DEFAULT_POSE]), dof_indices=dof_indices)
        articulation.set_dof_position_targets(np.array([DEFAULT_POSE]), dof_indices=dof_indices)
        articulation.set_dof_velocities(np.zeros((1, len(dof_names))))
        articulation.set_velocities(np.zeros((1, 3)), np.zeros((1, 3)))
        self._on_stop(None)  # bound for this Play
