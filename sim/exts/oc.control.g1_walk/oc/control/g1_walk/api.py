"""Public API: make the G1 at a prim path walk a repeating Walk sequence while the simulation plays."""

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

from .controller import DEFAULT_POSE, JOINTS, KD, KP, G1WalkController
from .sequence import command_at
from .walk_plan import WalkPlan

__all__ = ["G1WalkController", "G1Walker", "WalkPlan", "command_at"]

DECIMATION = 2  # physics steps per policy step: 100 Hz physics, 50 Hz policy


class G1Walker:
    """Steps a G1WalkController on every physics step while the simulation plays, once start() is called.

    Binds to the robot on the first physics step after Play, and unbinds on Stop, so the
    robot restarts the sequence from the beginning on each Play.
    """

    def __init__(self, prim_path: str, sequence) -> None:
        self.prim_path = prim_path
        self.sequence = sequence
        self._controller = G1WalkController()
        self._articulation = None
        self._tick = 0
        self._failed = False
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
            self._step_callback_id = SimulationManager.register_callback(
                self._on_physics_step, SimulationEvent.PHYSICS_POST_STEP
            )

    def _on_stop(self, event) -> None:
        if self._step_callback_id is not None:
            SimulationManager.deregister_callback(self._step_callback_id)
            self._step_callback_id = None
        self._articulation = None
        self._failed = False

    def _bind(self) -> bool:
        if not omni.usd.get_context().get_stage().GetPrimAtPath(self.prim_path).IsValid():
            return False
        self._articulation = Articulation(self.prim_path)
        dof_names = list(self._articulation.dof_names)
        dof_indices = [dof_names.index(name) for name in JOINTS]
        self._articulation.set_dof_gains(np.array([KP]), np.array([KD]), dof_indices=dof_indices)
        # Start from the policy's standing pose at rest; the asset's authored state carries stale velocities.
        self._articulation.set_dof_positions(np.array([DEFAULT_POSE]), dof_indices=dof_indices)
        self._articulation.set_dof_position_targets(np.array([DEFAULT_POSE]), dof_indices=dof_indices)
        self._articulation.set_dof_velocities(np.zeros((1, len(dof_names))))
        self._articulation.set_velocities(np.zeros((1, 3)), np.zeros((1, 3)))
        self._tick = 0
        self._controller.reset(None, None, 0.0)
        return True

    def _on_physics_step(self, dt: float, context) -> None:
        if self._failed:
            return
        try:
            if self._articulation is None and not self._bind():
                return
            if self._tick % DECIMATION == 0:
                t = self._tick * dt
                vx, vy, wz = command_at(t, self.sequence)
                with wp.ScopedDevice("cpu"):
                    setpoint = mg.RobotState(
                        root=mg.RootState(
                            linear_velocity=wp.array([vx, vy, 0.0], dtype=wp.float32),
                            angular_velocity=wp.array([0.0, 0.0, wz], dtype=wp.float32),
                        )
                    )
                target = self._controller.forward(read_robot_state(self._articulation), setpoint, t)
                apply_robot_state(self._articulation, target)
            self._tick += 1
        except Exception as error:  # noqa: BLE001 - a physics callback must not raise
            self._failed = True
            carb.log_error(f"G1Walker({self.prim_path}): stopped after error: {error}")
