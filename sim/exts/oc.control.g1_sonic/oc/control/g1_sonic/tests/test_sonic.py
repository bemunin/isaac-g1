import math

import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import omni.kit.test
import warp as wp
from oc.control.g1_sonic import G1SonicController, MovementStatus
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
from oc.control.g1_sonic.controllers.sonic_controller import (
    COMMAND_TIMEOUT,
    MAX_FACING_LEAD,
    limit,
)
from oc.control.g1_sonic.locomotion_command import Crawl, LocomotionCommand, Posture

_controller = None


def _shared_controller() -> G1SonicController:
    """One controller for the tests: loading SONIC's planner takes a second."""
    global _controller
    if _controller is None:
        _controller = G1SonicController()
    _controller.halt()
    return _controller


def _state(x: float, y: float, yaw: float) -> mg.RobotState:
    """A G1 standing in SONIC's default pose at (x, y) facing yaw, at rest."""
    with wp.ScopedDevice("cpu"):
        return mg.RobotState(
            joints=mg.JointState.from_name(
                list(MUJOCO_JOINTS),
                positions=(list(MUJOCO_JOINTS), wp.array(DEFAULT_POSE.astype(np.float32), dtype=wp.float32)),
                velocities=(list(MUJOCO_JOINTS), wp.zeros(len(MUJOCO_JOINTS), dtype=wp.float32)),
            ),
            root=mg.RootState(
                position=wp.array([x, y, 0.79], dtype=wp.float32),
                orientation=wp.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)], dtype=wp.float32),
                angular_velocity=wp.zeros(3, dtype=wp.float32),
            ),
        )


class TestLocomotionCommand(omni.kit.test.AsyncTestCase):
    async def test_limits(self):
        def limited(vx, vy, wz):
            command = limit(LocomotionCommand(vx, vy, wz))
            return command.vx, command.vy, command.wz

        self.assertEqual(limited(2.0, 1.0, 3.0), (1.5, 0.4, 1.5))
        self.assertEqual(limited(-2.0, -1.0, -3.0), (-0.8, -0.4, -1.5))  # backward up to 0.8 m/s
        self.assertEqual(limited(0.5, 0.2, 0.3), (0.5, 0.2, 0.3))

    async def test_walk_mode_and_speeds(self):
        controller = _shared_controller()
        controller._facing = 0.0
        self.assertEqual(controller.planner_inputs(LocomotionCommand(1.2), 0.0), (WALK, 0.0, 1.2))
        self.assertEqual(controller.planner_inputs(LocomotionCommand(2.5), 0.0), (WALK, 0.0, 1.5))
        mode, movement, speed = controller.planner_inputs(LocomotionCommand(-0.5), 0.0)
        self.assertEqual((mode, speed), (WALK, 0.8))  # raised to walking speed
        self.assertAlmostEqual(abs(movement), math.pi)  # backward

    async def test_deadband_floor_and_strafe(self):
        controller = _shared_controller()
        self.assertEqual(controller.planner_inputs(LocomotionCommand(0.05), 0.0), (IDLE, None, 0.0))
        self.assertEqual(controller.planner_inputs(LocomotionCommand(0.2), 0.0)[::2], (WALK, 0.8))
        mode, movement, speed = controller.planner_inputs(LocomotionCommand(0.0, 0.3), 1.0)
        self.assertEqual(mode, WALK)
        self.assertAlmostEqual(speed, 0.3)  # side steps stay slow
        self.assertAlmostEqual(movement, 1.0 + math.pi / 2)  # relative to the measured yaw

    async def test_facing_stays_near_the_measured_yaw(self):
        controller = _shared_controller()
        controller._facing = 0.0
        for _ in range(100):
            controller.planner_inputs(LocomotionCommand(wz=1.5), 0.0)
        self.assertAlmostEqual(controller._facing, MAX_FACING_LEAD)


class TestLowerBody(omni.kit.test.AsyncTestCase):
    async def test_posture_holds_until_a_command(self):
        controller = _shared_controller()
        controller.reset(_state(0, 0, 0), None, 0.0)
        controller.posture(Posture.SQUAT, 0.6)
        self.assertEqual(controller.status, MovementStatus.RUNNING)
        target = controller.forward(_state(0, 0, 0), None, 0.0)
        self.assertTrue(np.isfinite(target.joints.positions.numpy()).all())
        controller.command(LocomotionCommand(1.0))  # a drive stands back up
        self.assertIsNone(controller._posture)
        controller.posture(Posture.KNEEL_ONE_LEG, 0.3)
        controller.halt()
        self.assertEqual(controller.status, MovementStatus.IDLE)

    async def test_crawl_is_driven_by_commands_and_holds_when_stopped(self):
        controller = _shared_controller()
        controller.reset(_state(0, 0, 0), None, 0.0)
        controller.crawl(Crawl.HANDS)
        self.assertEqual(controller.planner_inputs(LocomotionCommand(), 0.0), (Crawl.HANDS, None, 0.0))
        self.assertEqual(controller.planner_inputs(LocomotionCommand(0.2), 0.0)[::2], (Crawl.HANDS, 0.4))  # raised to 0.4
        self.assertEqual(controller.planner_inputs(LocomotionCommand(2.0), 0.0)[::2], (Crawl.HANDS, 1.0))  # capped at the crawl's top speed
        self.assertEqual(controller.planner_inputs(LocomotionCommand(-1.0), 0.0)[::2], (Crawl.HANDS, 0.0))  # forward only: backward holds the pose
        controller.crawl(Crawl.ELBOWS)
        self.assertEqual(controller.planner_inputs(LocomotionCommand(0.5), 0.0)[::2], (Crawl.ELBOWS, 0.7))
        controller.halt()
        self.assertEqual(controller.planner_inputs(LocomotionCommand(0.5), 0.0)[0], WALK)


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


class TestG1SonicController(omni.kit.test.AsyncTestCase):
    async def test_walks_then_times_out(self):
        controller = _shared_controller()
        controller.reset(_state(3, 4, 0), None, 0.0)
        self.assertEqual(controller.status, MovementStatus.IDLE)
        controller.command(LocomotionCommand(1.0, 0.0, 0.5))
        self.assertEqual(controller.status, MovementStatus.RUNNING)
        mode, _, speed = controller.planner_inputs(controller.step((3, 4, 0), 1.0), 0.0)
        self.assertEqual((mode, speed), (WALK, 1.0))
        self.assertGreater(controller._facing, 0.0)  # turning left as it walks
        self.assertEqual(controller.step((3, 4, 0), 1.0 + COMMAND_TIMEOUT), LocomotionCommand(1.0, 0.0, 0.5))
        self.assertEqual(controller.step((3, 4, 0), 1.0 + COMMAND_TIMEOUT + 0.01), LocomotionCommand())
        self.assertEqual(controller.status, MovementStatus.IDLE)

    async def test_halt_stands(self):
        controller = _shared_controller()
        controller.reset(_state(0, 0, 0), None, 0.0)
        controller.command(LocomotionCommand(1.0))
        controller.step((0, 0, 0), 0.0)
        controller.halt()
        self.assertEqual(controller.status, MovementStatus.IDLE)
        self.assertEqual(controller.planner_inputs(controller.step((0, 0, 0), 0.1), 0.0), (IDLE, None, 0.0))

    async def test_real_time_turn_in_place(self):
        controller = _shared_controller()
        controller.reset(_state(0, 0, 0), None, 0.0)
        controller.command(LocomotionCommand(wz=1.0))
        self.assertEqual(controller.planner_inputs(controller.step((0, 0, 0), 0.0), 0.0), (IDLE, None, 0.0))
        self.assertGreater(controller._facing, 0.0)

    async def test_forward_outputs_joint_targets(self):
        controller = _shared_controller()
        controller.reset(_state(0, 0, 0), None, 0.0)
        controller.command(LocomotionCommand(1.0))
        target = controller.forward(_state(0, 0, 0), None, 0.0)
        self.assertEqual(list(target.joints.position_names), list(MUJOCO_JOINTS))
        self.assertTrue(np.isfinite(target.joints.positions.numpy()).all())
        self.assertIsNone(target.root)
