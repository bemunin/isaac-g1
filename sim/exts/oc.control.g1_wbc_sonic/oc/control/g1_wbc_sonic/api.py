"""Public API: move the G1 at a prim path with NVIDIA SONIC whole-body control, by a Movement plan or Real-time
control, with IMU and pose feedback, while the simulation plays."""

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

from .controllers import G1WbcController, MovementStatus
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
from .locomotion_command import Gait, LocomotionCommand
from .movement_plan import MovementPlan, Segment
from .plan_executor import MovementFailure

__all__ = [
    "G1Wbc",
    "G1WbcController",
    "Gait",
    "LocomotionCommand",
    "MovementFailure",
    "MovementPlan",
    "MovementStatus",
    "Segment",
]

PHYSICS_DT = 0.005  # s: SONIC was trained at 200 Hz physics, the policy every 4th step


class G1Wbc:
    """Steps a G1WbcController on every physics step while the simulation plays, once start() is called.

    Binds to the robot on the first physics step after Play and unbinds on Stop; each Play starts standing and
    restarts the current Movement plan. Orientation and angular velocity come from the IMU at imu_path, position and
    joints from the articulation.

    Commands: execute(plan) follows a Movement plan; command(LocomotionCommand), and its shortcuts slow_walk(cmd_vel),
    walk(cmd_vel) and run(cmd_vel), move at a body-frame (vx, vy, wz) until commands stop arriving for 0.5 s;
    side_move_left/right(meters) and turn_left/right(degrees) run to completion; halt() stands. Each command cancels
    the one before.
    """

    def __init__(self, prim_path: str, imu_path: str | None = None, on_done=None) -> None:
        self.prim_path = prim_path
        self.imu_path = imu_path or f"{prim_path}/pelvis/imu_sensor"
        self._controller = G1WbcController(on_done=on_done)
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

    @property
    def failure(self) -> MovementFailure | None:
        """Why the last Movement plan failed (reason, segment index), if it did."""
        return self._controller.failure

    @property
    def plan(self) -> MovementPlan | None:
        return self._controller.plan

    def execute(self, plan: MovementPlan) -> None:
        """Follow plan from the robot's current pose, replacing the current command."""
        self._controller.execute(plan)

    def command(self, command: LocomotionCommand) -> None:
        """Move at command's body-frame velocity, no faster than its gait (None: any, chosen from the speed); limited
        to forward only, |vy| <= 0.4 m/s, |wz| <= 1.5 rad/s. Call again within 0.5 s to keep moving, e.g. from a
        cmd_vel subscriber."""
        self._controller.command(command)

    def slow_walk(self, cmd_vel) -> None:
        """command() at cmd_vel = (vx [m/s], vy [m/s], wz [rad/s]), slow-walking (vx up to 0.8)."""
        self._controller.command(LocomotionCommand(*cmd_vel, gait=Gait.SLOW_WALK))

    def walk(self, cmd_vel) -> None:
        """command() at cmd_vel = (vx, vy, wz), walking at most (vx up to 1.5)."""
        self._controller.command(LocomotionCommand(*cmd_vel, gait=Gait.WALK))

    def run(self, cmd_vel) -> None:
        """command() at cmd_vel = (vx, vy, wz), running from 1.5 m/s (vx up to 3.0)."""
        self._controller.command(LocomotionCommand(*cmd_vel, gait=Gait.RUN))

    def side_move_left(self, meters: float) -> None:
        self._controller.execute(MovementPlan().side_move_left(meters))

    def side_move_right(self, meters: float) -> None:
        self._controller.execute(MovementPlan().side_move_right(meters))

    def turn_left(self, degrees: float) -> None:
        self._controller.execute(MovementPlan().turn_left(degrees))

    def turn_right(self, degrees: float) -> None:
        self._controller.execute(MovementPlan().turn_right(degrees))

    def halt(self) -> None:
        """Slow down and stand, cancelling the running command (also a looping Movement plan)."""
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
                    carb.log_warn(f"G1Wbc({self.prim_path}): physics runs at {1 / dt:.0f} Hz; SONIC expects {1 / PHYSICS_DT:.0f} Hz")
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
            carb.log_error(f"G1Wbc({self.prim_path}): stopped after error: {error}")
