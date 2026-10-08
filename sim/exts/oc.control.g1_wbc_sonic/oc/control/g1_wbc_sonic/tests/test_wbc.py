import math

import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import omni.kit.test
import warp as wp
from oc.control.g1_wbc_sonic import (
    G1WbcController,
    Gait,
    LocomotionCommand,
    MovementFailure,
    MovementPlan,
    MovementStatus,
)
from oc.control.g1_wbc_sonic.controllers.planner import IDLE, WALK, Planner
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
from oc.control.g1_wbc_sonic.movement_plan import RUN_DECEL, WALK_DECEL, WALK_SPEED
from oc.control.g1_wbc_sonic.path_tracker import GoalLineTracker, profile_speed
from oc.control.g1_wbc_sonic.plan_executor import PlanExecutor

_controller = None


def _shared_controller() -> G1WbcController:
    """One controller for the tests: loading SONIC's planner takes a second."""
    global _controller
    if _controller is None:
        _controller = G1WbcController()
    _controller.halt()
    _controller.on_done = None
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


def _started(plan: MovementPlan, pose=(0.0, 0.0, 0.0)) -> PlanExecutor:
    """An executor running plan, anchored at pose at t = 0."""
    executor = PlanExecutor()
    executor.execute(plan)
    executor.update(pose, 0.0)
    return executor


class TestMovementPlan(omni.kit.test.AsyncTestCase):
    async def test_square_ends_at_start(self):
        plan = MovementPlan(loop=True)
        for _ in range(4):
            plan = plan.walk(2).turn_left(90)
        segments = plan.segments
        self.assertEqual(len(segments), 8)
        self.assertTrue(plan.repeats)
        self.assertAlmostEqual(segments[0].target[0], 2)
        self.assertAlmostEqual(segments[2].target[1], 2)
        self.assertEqual(segments[2].start, segments[1].target)
        last = segments[-1].target
        self.assertAlmostEqual(last[0], 0, places=6)
        self.assertAlmostEqual(last[1], 0, places=6)

    async def test_primitives_return_new_plans(self):
        plan = MovementPlan()
        longer = plan.walk(2)
        self.assertIsNot(plan, longer)
        with self.assertRaises(ValueError):
            plan.segments  # still empty
        self.assertEqual(len(longer.segments), 1)

    async def test_gaits(self):
        plan = MovementPlan().slow_walk(1).walk(1).run(5).side_move_left(1).turn_right(90).stop()
        gaits = [segment.gait for segment in plan.segments]
        self.assertEqual(gaits, [Gait.SLOW_WALK, Gait.WALK, Gait.RUN, Gait.SLOW_WALK, Gait.IDLE, Gait.IDLE])

    async def test_side_move_keeps_heading(self):
        segment = MovementPlan().side_move_left(0.5).segments[0]
        self.assertEqual(segment.kind, "side")
        self.assertAlmostEqual(segment.target[0], 0)
        self.assertAlmostEqual(segment.target[1], 0.5)
        self.assertEqual(segment.target[2], 0)
        self.assertAlmostEqual(MovementPlan().side_move_right(0.5).segments[0].target[1], -0.5)

    async def test_invalid_primitives_raise(self):
        for build in (
            lambda: MovementPlan().walk(0),
            lambda: MovementPlan().walk(-2),  # forward only
            lambda: MovementPlan().walk(2, speed=2.0),  # above walking speed
            lambda: MovementPlan().walk(2, speed=0.5),  # slow-walking speed
            lambda: MovementPlan().slow_walk(2, speed=0.2),  # steps in place below 0.4 m/s
            lambda: MovementPlan().slow_walk(2, speed=1.0),
            lambda: MovementPlan().run(5, speed=1.0),  # below running speed
            lambda: MovementPlan().run(5, speed=3.5),
            lambda: MovementPlan().side_move_left(1, speed=0.6),  # feet collide above 0.4 m/s
            lambda: MovementPlan().turn_left(-90),
            lambda: MovementPlan().stop(0),
            lambda: MovementPlan().segments,  # empty
        ):
            with self.assertRaises(ValueError):
                build()

    async def test_stop_without_seconds_ends_the_plan(self):
        plan = MovementPlan(loop=True).walk(2).stop()
        self.assertFalse(plan.repeats)
        self.assertTrue(plan.segments[-1].forever)
        with self.assertRaises(ValueError):
            MovementPlan().walk(2).stop().walk(2)

    async def test_stop_with_seconds_continues(self):
        segments = MovementPlan(loop=True).walk(2).stop(2).walk(2).segments
        self.assertEqual(segments[1].kind, "stop")
        self.assertEqual(segments[1].hold, 2)
        self.assertFalse(segments[1].forever)


class TestSpeedProfile(omni.kit.test.AsyncTestCase):
    async def test_walk_slows_to_zero_before_turn_side_move_and_stop(self):
        for plan in (
            MovementPlan().walk(3).turn_left(90),
            MovementPlan().walk(3).side_move_left(1),
            MovementPlan().walk(3).stop(1),
            MovementPlan().walk(3),
        ):
            self.assertEqual(plan.segments[0].exit_speed, 0.0)

    async def test_run_slows_to_walk_speed(self):
        run, walk = MovementPlan().run(10, speed=2.5).walk(3, speed=1.0).segments
        self.assertEqual(run.exit_speed, 1.0)
        self.assertEqual(walk.exit_speed, 0.0)
        self.assertAlmostEqual(profile_speed(run, 0.0), 1.0)

    async def test_walk_carries_into_run(self):
        walk, run = MovementPlan().walk(3, speed=1.0).run(10, speed=2.5).segments
        self.assertEqual(walk.exit_speed, 1.0)
        self.assertEqual(profile_speed(run, 10.0), 2.5)  # no ramp up: SONIC eases into speed itself

    async def test_braking_distance(self):
        run = MovementPlan().run(10, speed=2.5).segments[0]
        braking = 2.5**2 / (2 * RUN_DECEL)  # 2.08 m
        self.assertAlmostEqual(profile_speed(run, braking + 0.5), 2.5)
        self.assertLess(profile_speed(run, braking - 0.5), 2.5)
        self.assertAlmostEqual(profile_speed(run, 1.0), math.sqrt(2 * RUN_DECEL * 1.0))
        walk = MovementPlan().walk(5).segments[0]
        self.assertAlmostEqual(profile_speed(walk, 0.32), math.sqrt(2 * WALK_DECEL * 0.32))

    async def test_short_moves_are_allowed(self):
        MovementPlan().walk(0.8, speed=WALK_SPEED).segments  # never reaches cruise speed

    async def test_loop_carries_speed_over_the_seam(self):
        segments = MovementPlan(loop=True).walk(3).walk(3).segments
        self.assertEqual(segments[1].exit_speed, WALK_SPEED)


class TestPathTracker(omni.kit.test.AsyncTestCase):
    async def test_walk_cruises_and_steers_back_onto_the_segment(self):
        executor = _started(MovementPlan().walk(5))
        segment = executor.update((2, 0, 0), 1.0)
        command = GoalLineTracker().compute((2, 0, 0), segment)
        self.assertAlmostEqual(command.vx, WALK_SPEED)
        self.assertAlmostEqual(command.vy, 0.0)
        command = GoalLineTracker().compute((2, 0.3, 0), segment)  # 0.3 m left of the line
        self.assertLess(command.vy, 0.0)  # heads right, back to it

    async def test_walk_holds_the_heading(self):
        segment = _started(MovementPlan().walk(5)).update((1, 0, 0.1), 1.0)
        self.assertLess(GoalLineTracker().compute((1, 0, 0.1), segment).wz, 0.0)

    async def test_side_move_left(self):
        segment = _started(MovementPlan().side_move_left(1)).update((0, 0.2, 0), 1.0)
        command = GoalLineTracker().compute((0, 0.2, 0), segment)
        self.assertAlmostEqual(command.vx, 0.0, places=6)
        self.assertAlmostEqual(command.vy, 0.3)
        self.assertAlmostEqual(command.wz, 0.0)

    async def test_turn_right_only_turns(self):
        segment = _started(MovementPlan().turn_right(90)).update((0, 0, 0), 0.5)
        command = GoalLineTracker().compute((0, 0, 0), segment)
        self.assertEqual((command.vx, command.vy), (0.0, 0.0))
        self.assertLess(command.wz, 0.0)

    async def test_works_in_the_anchor_frame(self):
        segment = _started(MovementPlan().walk(5), pose=(3, 4, math.pi / 2)).update((3, 6, math.pi / 2), 1.0)
        self.assertAlmostEqual(segment.target[0], 3)
        self.assertAlmostEqual(segment.target[1], 9)
        command = GoalLineTracker().compute((3, 6, math.pi / 2), segment)
        self.assertAlmostEqual(command.vx, WALK_SPEED)  # body frame: straight ahead


class TestPlanExecutor(omni.kit.test.AsyncTestCase):
    async def test_walk_ends_along_the_segment_and_finishes(self):
        done = []
        executor = _started(MovementPlan().walk(5))
        executor.on_done = done.append
        self.assertEqual(executor.status, MovementStatus.RUNNING)
        self.assertIsNotNone(executor.update((4.8, 0.4, 0), 5.0))  # 0.2 m to go, off the line
        self.assertIsNone(executor.update((4.9, -0.4, 0), 6.0))
        self.assertEqual(executor.status, MovementStatus.DONE)
        self.assertEqual(done, [MovementStatus.DONE])

    async def test_side_move_then_turn_right(self):
        executor = _started(MovementPlan().side_move_left(1).turn_right(90))
        self.assertEqual(executor.update((0, 0.9, 0), 4.0).kind, "turn")
        self.assertIsNotNone(executor.update((0, 1, -math.radians(80)), 5.0))
        self.assertIsNone(executor.update((0, 1, -math.radians(87)), 6.0))
        self.assertEqual(executor.status, MovementStatus.DONE)

    async def test_stop_holds_then_continues(self):
        executor = _started(MovementPlan().stop(2).turn_left(90))
        executor.update((0, 0, 0), 1.0)
        self.assertEqual(executor.index, 0)
        executor.update((0, 0, 0), 2.1)
        self.assertEqual(executor.index, 1)

    async def test_stop_without_seconds_ends_the_plan(self):
        executor = _started(MovementPlan().stop())
        self.assertIsNone(executor.update((0, 0, 0), 100.0))
        self.assertEqual(executor.status, MovementStatus.DONE)  # stands until the next command

    async def test_loops(self):
        executor = _started(MovementPlan(loop=True).walk(2).turn_left(180))
        executor.update((1.9, 0, 0), 2.0)
        executor.update((2, 0, math.pi), 4.0)
        self.assertEqual(executor.index, 0)
        self.assertEqual(executor.status, MovementStatus.RUNNING)

    async def test_times_out(self):
        done = []
        executor = _started(MovementPlan().walk(2).walk(2))
        executor.on_done = done.append
        executor.update((0, 0, 0), 100.0)
        self.assertEqual(executor.status, MovementStatus.FAILED)
        self.assertEqual(executor.failure, MovementFailure("timeout", 0))
        self.assertEqual(done, [MovementStatus.FAILED])

    async def test_cancel(self):
        done = []
        executor = _started(MovementPlan(loop=True).walk(2).turn_left(180))
        executor.on_done = done.append
        executor.cancel()
        self.assertEqual(executor.status, MovementStatus.CANCELLED)
        self.assertEqual(done, [MovementStatus.CANCELLED])
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
        planner = Planner()
        planner.reset()
        policy = SonicPolicy()
        policy.reset(np.array([1.0, 0, 0, 0]), planner.root_quat())
        targets = policy.act(DEFAULT_POSE, np.zeros(29), np.array([1.0, 0, 0, 0]), np.zeros(3), planner.reference())
        self.assertEqual(targets.shape, (29,))
        self.assertLess(np.abs(targets - DEFAULT_POSE).max(), 0.5)


class TestPlanner(omni.kit.test.AsyncTestCase):
    async def test_walk_moves_forward_and_turn_in_place_turns(self):
        planner = Planner()
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
        controller.execute(MovementPlan().run(10))
        controller.reset(_state(3, 4, 0), None, 0.0)
        command = controller.step((3, 4, 0), 0.0)
        self.assertEqual(controller.status, MovementStatus.RUNNING)
        self.assertEqual(command.gait, Gait.RUN)
        self.assertEqual(controller.planner_inputs(controller.step((8, 4, 0), 3.0), 0.0)[0], Gait.RUN)  # cruising
        mode, _, speed = controller.planner_inputs(controller.step((12.5, 4, 0), 6.0), 0.0)  # 0.5 m out
        self.assertNotEqual(mode, Gait.RUN)
        self.assertLess(speed, 1.5)
        self.assertEqual(controller.step((12.9, 4, 0), 7.0).gait, Gait.IDLE)
        self.assertEqual(controller.status, MovementStatus.DONE)
        self.assertEqual(done, [MovementStatus.DONE])

    async def test_halt_cancels_a_looping_plan(self):
        controller = _shared_controller()
        done = []
        controller.on_done = done.append
        controller.execute(MovementPlan(loop=True).walk(2).turn_left(180))
        controller.reset(_state(0, 0, 0), None, 0.0)
        controller.step((0, 0, 0), 0.0)
        controller.halt()
        self.assertEqual(controller.status, MovementStatus.CANCELLED)
        self.assertEqual(done, [MovementStatus.CANCELLED])
        self.assertEqual(controller.step((0, 0, 0), 1.0).gait, Gait.IDLE)

    async def test_real_time_command_preempts_plan_and_times_out(self):
        controller = _shared_controller()
        done = []
        controller.on_done = done.append
        controller.execute(MovementPlan().walk(5))
        controller.reset(_state(0, 0, 0), None, 0.0)
        controller.step((0, 0, 0), 0.0)
        controller.command(LocomotionCommand(0.5, 0.0, 0.5, Gait.WALK))
        self.assertEqual(done, [MovementStatus.CANCELLED])
        self.assertEqual(controller.status, MovementStatus.RUNNING)
        mode, movement, speed = controller.planner_inputs(controller.step((0, 0, 0), 1.0), 0.0)
        self.assertEqual(mode, Gait.SLOW_WALK)
        self.assertAlmostEqual(speed, 0.5)
        self.assertGreater(controller._facing, 0.0)  # turning left as it walks
        controller.step((0, 0, 0), 1.0 + COMMAND_TIMEOUT + 0.01)
        self.assertEqual(controller.status, MovementStatus.IDLE)

    async def test_real_time_turn_in_place(self):
        controller = _shared_controller()
        controller.reset(_state(0, 0, 0), None, 0.0)
        controller.command(LocomotionCommand(wz=1.0, gait=Gait.WALK))
        self.assertEqual(controller.planner_inputs(controller.step((0, 0, 0), 0.0), 0.0), (Gait.IDLE, None, 0.0))
        self.assertGreater(controller._facing, 0.0)

    async def test_forward_outputs_joint_targets(self):
        controller = _shared_controller()
        controller.execute(MovementPlan().walk(2))
        controller.reset(_state(0, 0, 0), None, 0.0)
        target = controller.forward(_state(0, 0, 0), None, 0.0)
        self.assertEqual(list(target.joints.position_names), list(MUJOCO_JOINTS))
        self.assertTrue(np.isfinite(target.joints.positions.numpy()).all())
        self.assertIsNone(target.root)
