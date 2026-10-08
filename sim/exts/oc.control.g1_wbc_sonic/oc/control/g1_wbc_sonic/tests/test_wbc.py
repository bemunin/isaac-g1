import math

import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import omni.kit.test
import warp as wp
from oc.control.g1_wbc_sonic import (
    BehaviorSequence,
    G1WbcController,
    Gait,
    LocomotionCommand,
    MovementFailure,
    MovementStatus,
)
from oc.control.g1_wbc_sonic.behavior_sequence import RUN_DECEL, WALK_DECEL, WALK_SPEED
from oc.control.g1_wbc_sonic.controllers.kinematic_planner import (
    IDLE,
    WALK,
    KinematicPlanner,
)
from oc.control.g1_wbc_sonic.controllers.sonic import (
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
from oc.control.g1_wbc_sonic.controllers.wbc_controller import (
    COMMAND_TIMEOUT,
    MAX_FACING_LEAD,
    limit,
)
from oc.control.g1_wbc_sonic.sequence_executor import SequenceExecutor, profile_speed

_controller = None


def _shared_controller() -> G1WbcController:
    """One controller for the tests: loading SONIC's planner takes a second."""
    global _controller
    if _controller is None:
        _controller = G1WbcController()
    _controller.halt()
    _controller.on_done = None
    _controller.set_speed_limit(None)
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


def _started(sequence: BehaviorSequence, pose=(0.0, 0.0, 0.0)) -> SequenceExecutor:
    """An executor running sequence, anchored at pose at t = 0."""
    executor = SequenceExecutor()
    executor.execute(sequence)
    executor.update(pose, 0.0)
    return executor


class TestBehaviorSequence(omni.kit.test.AsyncTestCase):
    async def test_square_ends_at_start(self):
        sequence = BehaviorSequence(loop=True)
        for _ in range(4):
            sequence = sequence.walk(2).spin(90)
        segments = sequence.segments
        self.assertEqual(len(segments), 8)
        self.assertTrue(sequence.repeats)
        self.assertAlmostEqual(segments[0].target[0], 2)
        self.assertAlmostEqual(segments[2].target[1], 2)
        self.assertEqual(segments[2].start, segments[1].target)
        last = segments[-1].target
        self.assertAlmostEqual(last[0], 0, places=6)
        self.assertAlmostEqual(last[1], 0, places=6)

    async def test_behaviors_return_new_sequences(self):
        sequence = BehaviorSequence()
        longer = sequence.walk(2)
        self.assertIsNot(sequence, longer)
        with self.assertRaises(ValueError):
            sequence.segments  # still empty
        self.assertEqual(len(longer.segments), 1)

    async def test_gaits(self):
        sequence = BehaviorSequence().slow_walk(1).walk(1).run(5).side_step(1).spin(-90).wait()
        gaits = [segment.gait for segment in sequence.segments]
        self.assertEqual(gaits, [Gait.SLOW_WALK, Gait.WALK, Gait.RUN, Gait.SLOW_WALK, Gait.IDLE, Gait.IDLE])

    async def test_drive_on_heading_gait(self):
        def gait(speed, gait=None):
            return BehaviorSequence().drive_on_heading(2, speed, gait).segments[0].gait

        self.assertEqual(gait(0.6), Gait.SLOW_WALK)
        self.assertEqual(gait(1.2), Gait.WALK)
        self.assertEqual(gait(2.5), Gait.RUN)
        self.assertEqual(gait(0.8, Gait.WALK), Gait.WALK)  # an explicit gait wins at the shared bound
        self.assertEqual(BehaviorSequence().walk(2).segments[0], BehaviorSequence().drive_on_heading(2, 1.0).segments[0])

    async def test_side_step_keeps_heading(self):
        segment = BehaviorSequence().side_step(0.5).segments[0]
        self.assertEqual(segment.kind, "side_step")
        self.assertAlmostEqual(segment.target[0], 0)
        self.assertAlmostEqual(segment.target[1], 0.5)  # positive: left
        self.assertEqual(segment.target[2], 0)
        self.assertAlmostEqual(BehaviorSequence().side_step(-0.5).segments[0].target[1], -0.5)

    async def test_spin_is_counterclockwise_positive(self):
        self.assertAlmostEqual(BehaviorSequence().spin(90).segments[0].target[2], math.pi / 2)
        self.assertAlmostEqual(BehaviorSequence().spin(-90).segments[0].target[2], -math.pi / 2)

    async def test_invalid_primitives_raise(self):
        for build in (
            lambda: BehaviorSequence().walk(0),
            lambda: BehaviorSequence().walk(-2),  # forward only
            lambda: BehaviorSequence().walk(2, speed=2.0),  # above walking speed
            lambda: BehaviorSequence().walk(2, speed=0.5),  # slow-walking speed
            lambda: BehaviorSequence().slow_walk(2, speed=0.2),  # steps in place below 0.4 m/s
            lambda: BehaviorSequence().slow_walk(2, speed=1.0),
            lambda: BehaviorSequence().run(5, speed=1.0),  # below running speed
            lambda: BehaviorSequence().run(5, speed=3.5),
            lambda: BehaviorSequence().side_step(1, speed=0.6),  # feet collide above 0.4 m/s
            lambda: BehaviorSequence().side_step(0),
            lambda: BehaviorSequence().spin(0),
            lambda: BehaviorSequence().drive_on_heading(2, 0.3),  # below any gait
            lambda: BehaviorSequence().drive_on_heading(2, 1.0, Gait.RUN),
            lambda: BehaviorSequence().drive_on_heading(2, 1.0, Gait.IDLE),
            lambda: BehaviorSequence().wait(0),
            lambda: BehaviorSequence().segments,  # empty
        ):
            with self.assertRaises(ValueError):
                build()

    async def test_wait_without_seconds_ends_the_sequence(self):
        sequence = BehaviorSequence(loop=True).walk(2).wait()
        self.assertFalse(sequence.repeats)
        self.assertTrue(sequence.segments[-1].forever)
        with self.assertRaises(ValueError):
            BehaviorSequence().walk(2).wait().walk(2)

    async def test_wait_with_seconds_continues(self):
        segments = BehaviorSequence(loop=True).walk(2).wait(2).walk(2).segments
        self.assertEqual(segments[1].kind, "wait")
        self.assertEqual(segments[1].hold, 2)
        self.assertFalse(segments[1].forever)


class TestSpeedProfile(omni.kit.test.AsyncTestCase):
    async def test_walk_slows_to_zero_before_spin_side_step_and_wait(self):
        for sequence in (
            BehaviorSequence().walk(3).spin(90),
            BehaviorSequence().walk(3).side_step(1),
            BehaviorSequence().walk(3).wait(1),
            BehaviorSequence().walk(3),
        ):
            self.assertEqual(sequence.segments[0].exit_speed, 0.0)

    async def test_run_slows_to_walk_speed(self):
        run, walk = BehaviorSequence().run(10, speed=2.5).walk(3, speed=1.0).segments
        self.assertEqual(run.exit_speed, 1.0)
        self.assertEqual(walk.exit_speed, 0.0)
        self.assertAlmostEqual(profile_speed(run, 0.0), 1.0)

    async def test_walk_carries_into_run(self):
        walk, run = BehaviorSequence().walk(3, speed=1.0).run(10, speed=2.5).segments
        self.assertEqual(walk.exit_speed, 1.0)
        self.assertEqual(profile_speed(run, 10.0), 2.5)  # no ramp up: SONIC eases into speed itself

    async def test_braking_distance(self):
        run = BehaviorSequence().run(10, speed=2.5).segments[0]
        braking = 2.5**2 / (2 * RUN_DECEL)  # 2.08 m
        self.assertAlmostEqual(profile_speed(run, braking + 0.5), 2.5)
        self.assertLess(profile_speed(run, braking - 0.5), 2.5)
        self.assertAlmostEqual(profile_speed(run, 1.0), math.sqrt(2 * RUN_DECEL * 1.0))
        walk = BehaviorSequence().walk(5).segments[0]
        self.assertAlmostEqual(profile_speed(walk, 0.32), math.sqrt(2 * WALK_DECEL * 0.32))

    async def test_short_moves_are_allowed(self):
        BehaviorSequence().walk(0.8, speed=WALK_SPEED).segments  # never reaches cruise speed

    async def test_loop_carries_speed_over_the_seam(self):
        segments = BehaviorSequence(loop=True).walk(3).walk(3).segments
        self.assertEqual(segments[1].exit_speed, WALK_SPEED)


class TestSteering(omni.kit.test.AsyncTestCase):
    async def test_walk_cruises_and_steers_back_onto_the_segment(self):
        executor = _started(BehaviorSequence().walk(5))
        command = executor.update((2, 0, 0), 1.0)
        self.assertAlmostEqual(command.vx, WALK_SPEED)
        self.assertAlmostEqual(command.vy, 0.0)
        self.assertEqual(command.gait, Gait.WALK)
        command = executor.update((2, 0.3, 0), 1.1)  # 0.3 m left of the line
        self.assertLess(command.vy, 0.0)  # heads right, back to it

    async def test_walk_holds_the_heading(self):
        self.assertLess(_started(BehaviorSequence().walk(5)).update((1, 0, 0.1), 1.0).wz, 0.0)

    async def test_side_step_left(self):
        command = _started(BehaviorSequence().side_step(1)).update((0, 0.2, 0), 1.0)
        self.assertAlmostEqual(command.vx, 0.0, places=6)
        self.assertAlmostEqual(command.vy, 0.3)
        self.assertAlmostEqual(command.wz, 0.0)

    async def test_spin_right_only_turns(self):
        command = _started(BehaviorSequence().spin(-90)).update((0, 0, 0), 0.5)
        self.assertEqual((command.vx, command.vy), (0.0, 0.0))
        self.assertLess(command.wz, 0.0)
        self.assertEqual(command.gait, Gait.IDLE)

    async def test_works_in_the_anchor_frame(self):
        executor = _started(BehaviorSequence().walk(5), pose=(3, 4, math.pi / 2))
        command = executor.update((3, 6, math.pi / 2), 1.0)
        self.assertAlmostEqual(command.vx, WALK_SPEED)  # body frame: straight ahead
        self.assertAlmostEqual(command.vy, 0.0)
        self.assertIsNone(executor.update((3, 8.9, math.pi / 2), 4.0))  # target (3, 9)


class TestSequenceExecutor(omni.kit.test.AsyncTestCase):
    async def test_walk_ends_along_the_segment_and_finishes(self):
        done = []
        executor = _started(BehaviorSequence().walk(5))
        executor.on_done = done.append
        self.assertEqual(executor.status, MovementStatus.RUNNING)
        self.assertIsNotNone(executor.update((4.8, 0.4, 0), 5.0))  # 0.2 m to go, off the line
        self.assertIsNone(executor.update((4.9, -0.4, 0), 6.0))
        self.assertEqual(executor.status, MovementStatus.SUCCEEDED)
        self.assertEqual(done, [MovementStatus.SUCCEEDED])

    async def test_side_step_then_spin_right(self):
        executor = _started(BehaviorSequence().side_step(1).spin(-90))
        executor.update((0, 0.9, 0), 4.0)
        self.assertEqual(executor.index, 1)
        self.assertIsNotNone(executor.update((0, 1, -math.radians(80)), 5.0))
        self.assertIsNone(executor.update((0, 1, -math.radians(87)), 6.0))
        self.assertEqual(executor.status, MovementStatus.SUCCEEDED)

    async def test_wait_holds_then_continues(self):
        executor = _started(BehaviorSequence().wait(2).spin(90))
        executor.update((0, 0, 0), 1.0)
        self.assertEqual(executor.index, 0)
        executor.update((0, 0, 0), 2.1)
        self.assertEqual(executor.index, 1)

    async def test_wait_without_seconds_ends_the_sequence(self):
        executor = _started(BehaviorSequence().wait())
        self.assertIsNone(executor.update((0, 0, 0), 100.0))
        self.assertEqual(executor.status, MovementStatus.SUCCEEDED)  # stands until the next command

    async def test_loops(self):
        executor = _started(BehaviorSequence(loop=True).walk(2).spin(180))
        executor.update((1.9, 0, 0), 2.0)
        executor.update((2, 0, math.pi), 4.0)
        self.assertEqual(executor.index, 0)
        self.assertEqual(executor.status, MovementStatus.RUNNING)

    async def test_times_out(self):
        done = []
        executor = _started(BehaviorSequence().walk(2).walk(2))
        executor.on_done = done.append
        executor.update((0, 0, 0), 100.0)
        self.assertEqual(executor.status, MovementStatus.ABORTED)
        self.assertEqual(executor.failure, MovementFailure("timeout", 0))
        self.assertEqual(done, [MovementStatus.ABORTED])

    async def test_cancel(self):
        done = []
        executor = _started(BehaviorSequence(loop=True).walk(2).spin(180))
        executor.on_done = done.append
        executor.cancel()
        self.assertEqual(executor.status, MovementStatus.CANCELED)
        self.assertEqual(done, [MovementStatus.CANCELED])
        self.assertIsNone(executor.update((0, 0, 0), 1.0))


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
    async def test_runs_then_slows_through_walk_and_finishes(self):
        controller = _shared_controller()
        done = []
        controller.on_done = done.append
        controller.execute(BehaviorSequence().run(10))
        controller.reset(_state(3, 4, 0), None, 0.0)
        command = controller.step((3, 4, 0), 0.0)
        self.assertEqual(controller.status, MovementStatus.RUNNING)
        self.assertEqual(command.gait, Gait.RUN)
        self.assertEqual(controller.planner_inputs(controller.step((8, 4, 0), 3.0), 0.0)[0], Gait.RUN)  # cruising
        mode, _, speed = controller.planner_inputs(controller.step((12.5, 4, 0), 6.0), 0.0)  # 0.5 m out
        self.assertNotEqual(mode, Gait.RUN)
        self.assertLess(speed, 1.5)
        self.assertEqual(controller.step((12.9, 4, 0), 7.0).gait, Gait.IDLE)
        self.assertEqual(controller.status, MovementStatus.SUCCEEDED)
        self.assertEqual(done, [MovementStatus.SUCCEEDED])

    async def test_halt_cancels_a_looping_sequence(self):
        controller = _shared_controller()
        done = []
        controller.on_done = done.append
        controller.execute(BehaviorSequence(loop=True).walk(2).spin(180))
        controller.reset(_state(0, 0, 0), None, 0.0)
        controller.step((0, 0, 0), 0.0)
        controller.halt()
        self.assertEqual(controller.status, MovementStatus.CANCELED)
        self.assertEqual(done, [MovementStatus.CANCELED])
        self.assertEqual(controller.step((0, 0, 0), 1.0).gait, Gait.IDLE)

    async def test_real_time_command_preempts_sequence_and_times_out(self):
        controller = _shared_controller()
        done = []
        controller.on_done = done.append
        controller.execute(BehaviorSequence().walk(5))
        controller.reset(_state(0, 0, 0), None, 0.0)
        controller.step((0, 0, 0), 0.0)
        controller.command(LocomotionCommand(0.5, 0.0, 0.5, Gait.WALK))
        self.assertEqual(done, [MovementStatus.CANCELED])
        self.assertEqual(controller.status, MovementStatus.RUNNING)
        mode, movement, speed = controller.planner_inputs(controller.step((0, 0, 0), 1.0), 0.0)
        self.assertEqual(mode, Gait.SLOW_WALK)
        self.assertAlmostEqual(speed, 0.5)
        self.assertGreater(controller._facing, 0.0)  # turning left as it walks
        controller.step((0, 0, 0), 1.0 + COMMAND_TIMEOUT + 0.01)
        self.assertEqual(controller.status, MovementStatus.IDLE)

    async def test_speed_limit_caps_speed_and_gait(self):
        controller = _shared_controller()
        controller.reset(_state(0, 0, 0), None, 0.0)
        controller.set_speed_limit(0.7)
        controller.command(LocomotionCommand(2.5, gait=Gait.RUN))
        mode, _, speed = controller.planner_inputs(controller.step((0, 0, 0), 0.0), 0.0)
        self.assertEqual(mode, Gait.SLOW_WALK)
        self.assertAlmostEqual(speed, 0.7)
        controller.set_speed_limit(None)
        controller.command(LocomotionCommand(2.5, gait=Gait.RUN))
        self.assertEqual(controller.planner_inputs(controller.step((0, 0, 0), 0.1), 0.0)[0], Gait.RUN)
        with self.assertRaises(ValueError):
            controller.set_speed_limit(0.0)

    async def test_real_time_turn_in_place(self):
        controller = _shared_controller()
        controller.reset(_state(0, 0, 0), None, 0.0)
        controller.command(LocomotionCommand(wz=1.0, gait=Gait.WALK))
        self.assertEqual(controller.planner_inputs(controller.step((0, 0, 0), 0.0), 0.0), (Gait.IDLE, None, 0.0))
        self.assertGreater(controller._facing, 0.0)

    async def test_forward_outputs_joint_targets(self):
        controller = _shared_controller()
        controller.execute(BehaviorSequence().walk(2))
        controller.reset(_state(0, 0, 0), None, 0.0)
        target = controller.forward(_state(0, 0, 0), None, 0.0)
        self.assertEqual(list(target.joints.position_names), list(MUJOCO_JOINTS))
        self.assertTrue(np.isfinite(target.joints.positions.numpy()).all())
        self.assertIsNone(target.root)
