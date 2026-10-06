"""Public API: make the G1 at a prim path follow a Walk trajectory, with IMU and pose feedback, while the simulation plays."""

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

from .controllers import G1WalkController, WalkStatus
from .controllers.policy import DEFAULT_POSE, JOINTS, KD, KP, to_body_frame
from .walk_plan import WalkGoal, WalkPlan, WalkTrajectory

__all__ = ["G1WalkController", "G1Walker", "WalkGoal", "WalkPlan", "WalkStatus", "WalkTrajectory", "walkers"]

DECIMATION = 2  # physics steps per policy step: 100 Hz physics, 50 Hz policy

_walkers: list["G1Walker"] = []


def walkers() -> tuple["G1Walker", ...]:
    """The started G1Walkers, in start order."""
    return tuple(_walkers)


class G1Walker:
    """Steps a G1WalkController on every physics step while the simulation plays, once start() is called.

    Binds to the robot on the first physics step after Play, and unbinds on Stop, so the robot
    restarts its Walk trajectory from the beginning on each Play. Without one the robot stands.
    Orientation and angular velocity come from the IMU at imu_path, position and joints from the articulation.
    """

    def __init__(self, prim_path: str, imu_path: str | None = None, on_done=None) -> None:
        self.prim_path = prim_path
        self.imu_path = imu_path or f"{prim_path}/pelvis/imu_sensor"
        self.on_done = on_done  # called with the WalkStatus when a trajectory ends (DONE or FAILED)
        self._walk = G1WalkController()
        self._articulation = None
        self._imu = None
        self._pending_reset = False
        self._tick = 0
        self._failed = False
        self._timeline_subscriptions = []
        self._step_callback_id = None

    @property
    def status(self) -> WalkStatus:
        return self._walk.status

    @property
    def trajectory(self) -> WalkTrajectory | None:
        return self._walk.trajectory

    def world_path(self) -> tuple[tuple[float, float, float], ...]:
        """World poses (x, y, yaw) of the trajectory's start pose and Walk goals; empty unless playing with a trajectory."""
        if self._articulation is None or self._pending_reset:
            return ()
        return self._walk.world_path()

    def execute(self, trajectory: WalkTrajectory) -> None:
        """Follow trajectory from the robot's current pose, replacing the current one."""
        self._walk.set_trajectory(trajectory)

    def start(self) -> None:
        if self not in _walkers:
            _walkers.append(self)
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
        if self in _walkers:
            _walkers.remove(self)
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
        dof_indices = [dof_names.index(name) for name in JOINTS]
        self._articulation.set_dof_gains(np.array([KP]), np.array([KD]), dof_indices=dof_indices)
        # Start from the policy's standing pose at rest; the asset's authored state carries stale velocities.
        self._articulation.set_dof_positions(np.array([DEFAULT_POSE]), dof_indices=dof_indices)
        self._articulation.set_dof_position_targets(np.array([DEFAULT_POSE]), dof_indices=dof_indices)
        self._articulation.set_dof_velocities(np.zeros((1, len(dof_names))))
        self._articulation.set_velocities(np.zeros((1, 3)), np.zeros((1, 3)))
        self._tick = 0
        self._pending_reset = True  # reset the policy and restart the trajectory on the first tick
        return True

    def _estimated_state(self, state: mg.RobotState) -> mg.RobotState:
        """Joints and root position from the articulation; orientation and body angular velocity from the IMU."""
        imu = self._imu.get_data()
        orientation, angular_velocity = imu["orientation"], imu["angular_velocity"]
        if not np.linalg.norm(orientation) > 0.5:  # no valid IMU reading yet
            orientation = state.root.orientation.numpy()
            angular_velocity = to_body_frame(orientation, state.root.angular_velocity.numpy())
        with wp.ScopedDevice("cpu"):
            return mg.RobotState(
                joints=state.joints,
                root=mg.RootState(
                    position=wp.array(state.root.position.numpy(), dtype=wp.float32),
                    orientation=wp.array(orientation, dtype=wp.float32),
                    angular_velocity=wp.array(angular_velocity, dtype=wp.float32),
                ),
            )

    def _on_physics_step(self, dt: float, context) -> None:
        if self._failed:
            return
        try:
            if self._articulation is None and not self._bind():
                return
            if self._tick % DECIMATION == 0:
                t = self._tick * dt
                state = self._estimated_state(read_robot_state(self._articulation))
                if self._pending_reset:
                    self._pending_reset = False
                    self._walk.reset(state, None, t)
                status = self._walk.status
                target = self._walk.forward(state, None, t)
                if status == WalkStatus.RUNNING and self._walk.status != WalkStatus.RUNNING and self.on_done:
                    self.on_done(self._walk.status)
                apply_robot_state(self._articulation, target)
            self._tick += 1
        except Exception as error:  # noqa: BLE001 - a physics callback must not raise
            self._failed = True
            carb.log_error(f"G1Walker({self.prim_path}): stopped after error: {error}")
