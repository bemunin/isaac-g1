"""Public API: move the G1 at a prim path with NVIDIA SONIC whole-body control, by Real-time control, with IMU and pose
feedback, while the simulation plays."""

import carb
import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import omni.timeline
import omni.usd
import warp as wp
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager
from isaacsim.robot.policy.examples.application import (
    apply_robot_state,
    read_robot_state,
)
from isaacsim.sensors.experimental.physics import IMUSensor

from .controllers import G1SonicController, MovementStatus
from .controllers.sonic import (
    ARMATURE,
    CONTROL_DT,
    DEFAULT_POSE,
    EFFORT_LIMIT,
    KD,
    KP,
    MUJOCO_JOINTS,
    quat_conjugate,
    quat_rotate,
)
from .locomotion_command import (
    POSTURE_HEIGHT_RANGE,
    Crawl,
    LocomotionCommand,
    Posture,
)

__all__ = ["Crawl", "G1Sonic", "G1SonicController", "MovementStatus", "Posture"]

PHYSICS_DT = 0.005  # s: SONIC was trained at 200 Hz physics, the policy every 4th step


class G1Sonic:
    """Steps a G1SonicController on every physics step while the simulation plays, once start() is called.

    Binds to the robot on the first physics step after Play and unbinds on Stop; each Play starts standing.
    Orientation and angular velocity come from the IMU at imu_path, position and
    joints from the articulation.

    command(cmd_vel) walks in SONIC's walk mode at a body-frame (vx, vy, wz), e.g. Nav2's cmd_vel, until commands stop
    arriving for 0.5 s; squat/kneel/kneel_one_leg(height) hold a static ground posture; crawl_hands/crawl_elbows()
    crawl low, driven by command(); halt() stands. Each command replaces the one before.
    """

    def __init__(self, prim_path: str, imu_path: str | None = None) -> None:
        self.prim_path = prim_path
        self.imu_path = imu_path or f"{prim_path}/pelvis/imu_sensor"
        self._controller = G1SonicController()
        self._articulation = None
        self._imu = None
        self._pending_reset = False
        self._tick = 0
        self._decimation = None
        self._failed = False
        self._timeline_subscriptions = []
        self._step_callback_id = None

    @property
    def status(self) -> MovementStatus:
        return self._controller.status

    def command(self, cmd_vel) -> None:
        """Walk at cmd_vel = (vx [m/s], vy [m/s], wz [rad/s]) in the robot frame: forward or backward at 0.8-1.5 m/s
        (backward up to 0.8), sideways up to 0.4 m/s, |wz| <= 1.5 rad/s; below 0.1 m/s it stands and turns in place.
        Call again within 0.5 s to keep moving, e.g. from a cmd_vel subscriber."""
        self._controller.command(LocomotionCommand(*cmd_vel))

    def squat(self, height: float = 0.6) -> None:
        """Squat in place to pelvis height [m] in [0.4, 0.8], held until the next command (halt() stands up)."""
        self._hold_posture(Posture.SQUAT, height)

    def kneel(self, height: float = 0.3) -> None:
        """Kneel on both legs at pelvis height [m] in [0.2, 0.4], held until the next command (halt() stands up)."""
        self._hold_posture(Posture.KNEEL_TWO_LEGS, height)

    def kneel_one_leg(self, height: float = 0.3) -> None:
        """Kneel on one leg at pelvis height [m] in [0.2, 0.4], held until the next command (halt() stands up)."""
        self._hold_posture(Posture.KNEEL_ONE_LEG, height)

    def _hold_posture(self, posture: Posture, height: float) -> None:
        low, high = POSTURE_HEIGHT_RANGE[posture]
        if not low <= height <= high:
            raise ValueError(f"G1Sonic: {posture.name} height must be in [{low}, {high}] m, got {height}")
        self._controller.posture(posture, height)

    def crawl_hands(self) -> None:
        """Crawl on hands (SONIC mode 8): a low moving mode that real-time commands (command/walk/cmd_vel) drive at
        0.4-1.0 m/s; it holds the low pose when they stop. halt() stands back up."""
        self._controller.crawl(Crawl.HANDS)

    def crawl_elbows(self) -> None:
        """Crawl on elbows (SONIC mode 14): a lower moving mode driven by real-time commands at 0.7-1.0 m/s, holding
        the low pose when they stop. halt() stands back up. (Overheats on the real robot.)"""
        self._controller.crawl(Crawl.ELBOWS)

    def halt(self) -> None:
        """Slow down and stand, dropping the running command (also a posture or crawl)."""
        self._controller.halt()

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
            self._step_callback_id = SimulationManager.register_callback(
                self._on_physics_step, SimulationEvent.PHYSICS_POST_STEP
            )

    def _on_stop(self, event) -> None:
        if self._step_callback_id is not None:
            SimulationManager.deregister_callback(self._step_callback_id)
            self._step_callback_id = None
        self._articulation = None
        self._imu = None
        self._failed = False

    def _bind(self) -> bool:
        if not omni.usd.get_context().get_stage().GetPrimAtPath(self.prim_path).IsValid():
            return False
        self._articulation = Articulation(self.prim_path)
        self._imu = IMUSensor(self.imu_path)
        dof_names = list(self._articulation.dof_names)
        dof_indices = [dof_names.index(name) for name in MUJOCO_JOINTS]
        self._articulation.set_dof_gains(np.array([KP]), np.array([KD]), dof_indices=dof_indices)
        self._articulation.set_dof_armatures(np.array([ARMATURE]), dof_indices=dof_indices)
        self._articulation.set_dof_max_efforts(np.array([EFFORT_LIMIT]), dof_indices=dof_indices)
        # Start from SONIC's standing pose at rest; the asset's authored state carries stale velocities.
        self._articulation.set_dof_positions(np.array([DEFAULT_POSE]), dof_indices=dof_indices)
        self._articulation.set_dof_position_targets(np.array([DEFAULT_POSE]), dof_indices=dof_indices)
        self._articulation.set_dof_velocities(np.zeros((1, len(dof_names))))
        self._articulation.set_velocities(np.zeros((1, 3)), np.zeros((1, 3)))
        self._tick = 0
        self._decimation = None
        self._pending_reset = True  # reset the controller on the first tick
        return True

    def _estimated_state(self, state: mg.RobotState) -> mg.RobotState:
        """Joints and root position from the articulation; orientation and body angular velocity from the IMU."""
        imu = self._imu.get_data()
        orientation, angular_velocity = imu["orientation"], imu["angular_velocity"]
        if not np.linalg.norm(orientation) > 0.5:  # no valid IMU reading yet
            orientation = state.root.orientation.numpy().reshape(-1)
            angular_velocity = quat_rotate(quat_conjugate(orientation), state.root.angular_velocity.numpy().reshape(-1))
        with wp.ScopedDevice("cpu"):
            return mg.RobotState(
                joints=state.joints,
                root=mg.RootState(
                    position=wp.array(state.root.position.numpy(), dtype=wp.float32),
                    orientation=wp.array(np.asarray(orientation, dtype=np.float32).reshape(-1), dtype=wp.float32),
                    angular_velocity=wp.array(np.asarray(angular_velocity, dtype=np.float32).reshape(-1), dtype=wp.float32),
                ),
            )

    def _on_physics_step(self, dt: float, context) -> None:
        if self._failed:
            return
        try:
            if self._articulation is None and not self._bind():
                return
            if self._decimation is None:
                self._decimation = max(1, round(CONTROL_DT / dt))
                if abs(dt - PHYSICS_DT) > 1e-6:
                    carb.log_warn(f"G1Sonic({self.prim_path}): physics runs at {1 / dt:.0f} Hz; SONIC expects {1 / PHYSICS_DT:.0f} Hz")
            if self._tick % self._decimation == 0:
                t = self._tick * dt
                state = self._estimated_state(read_robot_state(self._articulation))
                if self._pending_reset:
                    self._pending_reset = False
                    self._controller.reset(state, None, t)
                apply_robot_state(self._articulation, self._controller.forward(state, None, t))
            self._tick += 1
        except Exception as error:  # noqa: BLE001 - a physics callback must not raise
            self._failed = True
            carb.log_error(f"G1Sonic({self.prim_path}): stopped after error: {error}")
