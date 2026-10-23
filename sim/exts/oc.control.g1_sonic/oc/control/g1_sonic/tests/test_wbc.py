import math

import numpy as np
import omni.kit.test
from oc.control.g1_sonic import G1State, G1WbcController, Gait, LocomotionCommand
from oc.control.g1_sonic.controllers.kinematic_planner import (
    IDLE,
    WALK,
    KinematicPlanner,
)
from oc.control.g1_sonic.controllers.sonic import (
    DECODER_SIZE,
    DEFAULT_POSE,
    ENCODER_SIZE,
    ISAACLAB_JOINTS,
    ISAACLAB_OF_MUJOCO,
    MUJOCO_JOINTS,
    MUJOCO_OF_ISAACLAB,
    SONIC_DIR,
    SonicPolicy,
    _session,
)
from oc.control.g1_sonic.controllers.wbc_controller import (
    COMMAND_TIMEOUT,
    MAX_FACING_LEAD,
    limit,
)

_controller = None


def _shared_controller() -> G1WbcController:
    """One controller for the tests: loading SONIC's planner takes a second."""
    global _controller
    if _controller is None:
        _controller = G1WbcController()
    _controller.halt()
    _controller.set_speed_limit(None)
    return _controller


def _state(yaw: float) -> G1State:
    """A G1 standing in SONIC's default pose facing yaw, at rest."""
    return G1State(DEFAULT_POSE.copy(), np.zeros(29), np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]), np.zeros(3))


class TestLocomotionCommand(omni.kit.test.AsyncTestCase):
    async def test_limits(self):
        def limited(vx, vy, wz, gait):
            command = limit(LocomotionCommand(vx, vy, wz, gait))
            return command.vx, command.vy, command.wz

        self.assertEqual(limited(2.0, 1.0, 3.0, Gait.WALK), (1.5, 0.4, 1.5))
        self.assertEqual(limited(-1.0, -1.0, -3.0, Gait.WALK), (0.0, -0.4, -1.5))  # forward only
        self.assertEqual(limited(1.0, 0.0, 0.0, Gait.SLOW_WALK), (0.8, 0.0, 0.0))
        self.assertEqual(limited(1.0, 0.0, 0.0, Gait.RUN), (1.0, 0.0, 0.0))  # no longer raised to running speed
        self.assertEqual(limited(4.0, 0.0, 0.0, Gait.RUN), (3.0, 0.0, 0.0))
        self.assertEqual(limited(4.0, 0.0, 0.0, None), (3.0, 0.0, 0.0))
        self.assertEqual(limited(1.0, 0.3, 0.5, Gait.IDLE), (0.0, 0.0, 0.5))  # only turns

    async def test_gait_is_a_ceiling(self):
        controller = _shared_controller()
        controller._facing = 0.0
        self.assertEqual(controller.planner_inputs(LocomotionCommand(2.5, gait=Gait.RUN), 0.0)[0], Gait.RUN)
        self.assertEqual(controller.planner_inputs(LocomotionCommand(1.0, gait=Gait.RUN), 0.0)[0], Gait.WALK)
        self.assertEqual(controller.planner_inputs(LocomotionCommand(0.5, gait=Gait.RUN), 0.0)[0], Gait.SLOW_WALK)
        mode, _, speed = controller.planner_inputs(LocomotionCommand(2.5, gait=Gait.WALK), 0.0)
        self.assertEqual((mode, speed), (Gait.WALK, 1.5))
        self.assertEqual(controller.planner_inputs(LocomotionCommand(1.0, gait=Gait.IDLE), 0.0)[0], Gait.IDLE)
        self.assertEqual(controller.planner_inputs(LocomotionCommand(2.5), 0.0)[0], Gait.RUN)

    async def test_deadband_and_floor(self):
        controller = _shared_controller()
        self.assertEqual(controller.planner_inputs(LocomotionCommand(0.05, gait=Gait.WALK), 0.0), (Gait.IDLE, None, 0.0))
        mode, movement, speed = controller.planner_inputs(LocomotionCommand(0.2, gait=Gait.WALK), 0.0)
        self.assertEqual((mode, speed), (Gait.SLOW_WALK, 0.4))
        _, movement, speed = controller.planner_inputs(LocomotionCommand(0.0, 0.3, gait=Gait.SLOW_WALK), 1.0)
        self.assertAlmostEqual(speed, 0.3)  # side steps stay slow
        self.assertAlmostEqual(movement, 1.0 + math.pi / 2)  # relative to the measured yaw

    async def test_facing_stays_near_the_measured_yaw(self):
        controller = _shared_controller()
        controller._facing = 0.0
        for _ in range(100):
            controller.planner_inputs(LocomotionCommand(wz=1.5), 0.0)
        self.assertAlmostEqual(controller._facing, MAX_FACING_LEAD)


class TestSonic(omni.kit.test.AsyncTestCase):
    async def test_joint_orders_match_nvidia(self):
        # gear_sonic_deploy include/policy_parameters.hpp mujoco_to_isaaclab / isaaclab_to_mujoco
        self.assertEqual(list(MUJOCO_OF_ISAACLAB), [0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10, 16, 23, 5, 11, 17, 24, 18, 25, 19, 26, 20, 27, 21, 28])
        self.assertEqual(list(ISAACLAB_OF_MUJOCO), [0, 3, 6, 9, 13, 17, 1, 4, 7, 10, 14, 18, 2, 5, 8, 11, 15, 19, 21, 23, 25, 27, 12, 16, 20, 22, 24, 26, 28])
        self.assertEqual(sorted(ISAACLAB_JOINTS), sorted(MUJOCO_JOINTS))
        x = np.arange(29.0)
        np.testing.assert_array_equal(x[MUJOCO_OF_ISAACLAB][ISAACLAB_OF_MUJOCO], x)

    async def test_observation_sizes_match_models(self):
        self.assertEqual(_session(SONIC_DIR / "model_encoder.onnx").get_inputs()[0].shape, [1, ENCODER_SIZE])
        self.assertEqual(_session(SONIC_DIR / "model_decoder.onnx").get_inputs()[0].shape, [1, DECODER_SIZE])

    async def test_standing_targets_stay_near_default(self):
        planner = KinematicPlanner()
        planner.reset()
        policy = SonicPolicy()
        policy.reset(np.array([1.0, 0, 0, 0]), planner.root_quat())
        targets = policy.act(DEFAULT_POSE, np.zeros(29), np.array([1.0, 0, 0, 0]), np.zeros(3), planner.reference())
        self.assertEqual(targets.shape, (29,))
        self.assertLess(np.abs(targets - DEFAULT_POSE).max(), 0.5)


class TestKinematicPlanner(omni.kit.test.AsyncTestCase):
    async def test_walk_moves_forward_and_turn_in_place_turns(self):
        planner = KinematicPlanner()
        planner.reset()
        planner.command(WALK, 0.0, 0.0, 1.0)
        for _ in range(50):
            planner.step()
        self.assertGreater(planner.motion.positions[-1][0], 0.5)
        planner.reset()
        planner.command(IDLE, None, math.pi / 2, 0.0)
        for _ in range(100):
            planner.step()
        quat = planner.motion.quats[-1]
        self.assertAlmostEqual(2 * math.atan2(quat[3], quat[0]), math.pi / 2, delta=math.radians(20))


class TestG1WbcController(omni.kit.test.AsyncTestCase):
    async def test_real_time_command_times_out(self):
        controller = _shared_controller()
        controller.reset(_state(0))
        controller.command(LocomotionCommand(0.5, 0.0, 0.5, Gait.WALK))
        mode, _, speed = controller.planner_inputs(controller.step(1.0), 0.0)
        self.assertEqual(mode, Gait.SLOW_WALK)
        self.assertAlmostEqual(speed, 0.5)
        self.assertGreater(controller._facing, 0.0)  # turning left as it walks
        self.assertEqual(controller.step(1.0 + COMMAND_TIMEOUT + 0.01).gait, Gait.IDLE)

    async def test_halt_stands(self):
        controller = _shared_controller()
        controller.reset(_state(0))
        controller.command(LocomotionCommand(0.5, gait=Gait.WALK))
        controller.halt()
        self.assertEqual(controller.step(0.0).gait, Gait.IDLE)

    async def test_speed_limit_caps_speed_and_gait(self):
        controller = _shared_controller()
        controller.reset(_state(0))
        controller.set_speed_limit(0.7)
        controller.command(LocomotionCommand(2.5, gait=Gait.RUN))
        mode, _, speed = controller.planner_inputs(controller.step(0.0), 0.0)
        self.assertEqual(mode, Gait.SLOW_WALK)
        self.assertAlmostEqual(speed, 0.7)
        controller.set_speed_limit(None)
        controller.command(LocomotionCommand(2.5, gait=Gait.RUN))
        self.assertEqual(controller.planner_inputs(controller.step(0.1), 0.0)[0], Gait.RUN)
        with self.assertRaises(ValueError):
            controller.set_speed_limit(0.0)

    async def test_real_time_turn_in_place(self):
        controller = _shared_controller()
        controller.reset(_state(0))
        controller.command(LocomotionCommand(wz=1.0, gait=Gait.WALK))
        self.assertEqual(controller.planner_inputs(controller.step(0.0), 0.0), (Gait.IDLE, None, 0.0))
        self.assertGreater(controller._facing, 0.0)

    async def test_forward_outputs_joint_targets(self):
        controller = _shared_controller()
        controller.reset(_state(0))
        targets = controller.forward(_state(0), 0.0)
        self.assertEqual(targets.shape, (29,))
        self.assertTrue(np.isfinite(targets).all())
