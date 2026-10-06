import math

import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import omni.kit.test
import warp as wp
from oc.control.g1_walk import G1WalkController, WalkGoal, WalkPlan, WalkStatus
from oc.control.g1_walk.policy import DEFAULT_POSE, JOINTS
from oc.control.g1_walk.walk_controller import velocity_command, wrap
from oc.control.g1_walk.walk_plan import SPEED


def _state(x: float, y: float, yaw: float) -> mg.RobotState:
    """A G1 standing in the policy's default pose at (x, y) facing yaw, at rest."""
    with wp.ScopedDevice("cpu"):
        return mg.RobotState(
            joints=mg.JointState.from_name(
                list(JOINTS),
                positions=(list(JOINTS), wp.array(DEFAULT_POSE, dtype=wp.float32)),
                velocities=(list(JOINTS), wp.zeros(len(JOINTS), dtype=wp.float32)),
            ),
            root=mg.RootState(
                position=wp.array([x, y, 0.79], dtype=wp.float32),
                orientation=wp.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)], dtype=wp.float32),
                angular_velocity=wp.zeros(3, dtype=wp.float32),
            ),
        )


class TestWalkPlan(omni.kit.test.AsyncTestCase):
    async def test_square_ends_at_start(self):
        plan = WalkPlan(loop=True).wait(2)
        for _ in range(4):
            plan.move_forward(2).turn_left(90)
        trajectory = plan.to_trajectory()
        self.assertEqual(len(trajectory.goals), 9)
        self.assertTrue(trajectory.loop)
        self.assertEqual(trajectory.frame, "start")
        self.assertEqual(trajectory.goals[0].hold, 2)
        self.assertAlmostEqual(trajectory.goals[1].x, 2)
        self.assertAlmostEqual(trajectory.goals[3].y, 2)
        last = trajectory.goals[-1]
        self.assertAlmostEqual(last.x, 0, places=6)
        self.assertAlmostEqual(last.y, 0, places=6)
        self.assertAlmostEqual(wrap(last.yaw), 0, places=6)

    async def test_step_keeps_heading(self):
        goal = WalkPlan().step_left(0.5).goals[0]
        self.assertAlmostEqual(goal.x, 0)
        self.assertAlmostEqual(goal.y, 0.5)
        self.assertEqual(goal.yaw, 0)

    async def test_invalid_primitives_raise(self):
        with self.assertRaises(ValueError):
            WalkPlan().move_forward(0)
        with self.assertRaises(ValueError):
            WalkPlan().turn_left(-90)
        with self.assertRaises(ValueError):
            WalkPlan().to_trajectory()  # empty
        with self.assertRaises(ValueError):
            WalkPlan().move_forward(1, speed=0)
        with self.assertRaises(ValueError):
            WalkPlan().move_forward(1, speed=1.5)  # above the policy's 1 m/s
        with self.assertRaises(ValueError):
            WalkPlan().turn_left(90, speed=90)  # above the policy's 1 rad/s

    async def test_speeds(self):
        move, turn = WalkPlan().move_forward(1, speed=0.3).turn_left(90, speed=30).goals
        self.assertEqual(move.speed, 0.3)
        self.assertAlmostEqual(turn.turn_speed, math.radians(30))
        self.assertEqual(WalkPlan().walk_to(1, 2, 0, speed=0.8).goals[0].speed, 0.8)

    async def test_stop_ends_the_plan(self):
        self.assertFalse(WalkPlan(loop=True).move_forward(1).stop().to_trajectory().loop)
        with self.assertRaises(ValueError):
            WalkPlan().move_forward(1).stop().move_forward(1)
        with self.assertRaises(ValueError):
            WalkPlan().move_forward(1).stop().stop()

    async def test_walk_to(self):
        trajectory = WalkPlan().walk_to(1, 2, 0.5).walk_to(3, 2, 0).to_trajectory()
        self.assertEqual(trajectory.frame, "world")
        self.assertEqual(trajectory.goals, (WalkGoal(1, 2, 0.5), WalkGoal(3, 2, 0)))

    async def test_walk_to_cannot_mix_with_relative(self):
        with self.assertRaises(ValueError):
            WalkPlan().move_forward(1).walk_to(1, 2, 0)
        with self.assertRaises(ValueError):
            WalkPlan().walk_to(1, 2, 0).turn_left(90)


class TestVelocityCommand(omni.kit.test.AsyncTestCase):
    async def test_goal_ahead(self):
        vx, vy, wz = velocity_command((0, 0, 0), WalkGoal(2, 0, 0), (0, 0, 0))
        self.assertAlmostEqual(vx, SPEED)
        self.assertAlmostEqual(vy, 0)
        self.assertAlmostEqual(wz, 0)

    async def test_yaw_error_turns_back(self):
        _, _, wz = velocity_command((0, 0, math.radians(10)), WalkGoal(0, 0, 0), (0, 0, 0))
        self.assertLess(wz, 0)

    async def test_goal_in_body_frame(self):
        # Robot faces +y; a world goal at +x is to its right.
        vx, vy, _ = velocity_command((0, 0, math.pi / 2), WalkGoal(1, 0, math.pi / 2), (0, 0, 0))
        self.assertAlmostEqual(vx, 0, places=6)
        self.assertLess(vy, 0)

    async def test_anchor(self):
        # A goal 1 m ahead of an anchor at (5, 5) facing +y lies at (5, 6).
        vx, vy, _ = velocity_command((5, 5, math.pi / 2), WalkGoal(1, 0, 0), (5, 5, math.pi / 2))
        self.assertGreater(vx, 0)
        self.assertAlmostEqual(vy, 0, places=6)


    async def test_goal_speed(self):
        vx, _, _ = velocity_command((0, 0, 0), WalkGoal(2, 0, 0, speed=0.3), (0, 0, 0))
        self.assertAlmostEqual(vx, 0.3)


class TestG1WalkController(omni.kit.test.AsyncTestCase):
    async def test_advances_and_finishes(self):
        controller = G1WalkController()
        controller.set_trajectory(WalkPlan().move_forward(1).to_trajectory())
        controller.reset(_state(3, 4, 0), None, 0.0)
        self.assertEqual(controller.status, WalkStatus.RUNNING)
        self.assertGreater(controller.step((3, 4, 0), 0.0)[0], 0)
        controller.step((4, 4, 0), 1.0)
        self.assertEqual(controller.status, WalkStatus.DONE)
        self.assertEqual(controller.step((4, 4, 0), 1.1), (0.0, 0.0, 0.0))

    async def test_loops(self):
        controller = G1WalkController()
        controller.set_trajectory(WalkPlan(loop=True).move_forward(1).move_backward(1).to_trajectory())
        controller.reset(_state(0, 0, 0), None, 0.0)
        controller.step((1, 0, 0), 1.0)
        controller.step((0, 0, 0), 2.0)
        self.assertEqual(controller.status, WalkStatus.RUNNING)
        self.assertGreater(controller.step((0, 0, 0), 2.1)[0], 0)  # heading for the first goal again

    async def test_hold(self):
        controller = G1WalkController()
        controller.set_trajectory(WalkPlan().wait(1).to_trajectory())
        controller.reset(_state(0, 0, 0), None, 0.0)
        controller.step((0, 0, 0), 0.0)  # arrives: the hold starts
        controller.step((0, 0, 0), 0.5)
        self.assertEqual(controller.status, WalkStatus.RUNNING)
        controller.step((0, 0, 0), 1.0)
        self.assertEqual(controller.status, WalkStatus.DONE)

    async def test_times_out(self):
        controller = G1WalkController()
        controller.set_trajectory(WalkPlan().move_forward(1).to_trajectory())  # 2 s nominal: fails after 6 s
        controller.reset(_state(0, 0, 0), None, 0.0)
        controller.step((0, 0, 0), 5.9)
        self.assertEqual(controller.status, WalkStatus.RUNNING)
        self.assertEqual(controller.step((0, 0, 0), 6.1), (0.0, 0.0, 0.0))
        self.assertEqual(controller.status, WalkStatus.FAILED)

    async def test_timeout_follows_speed(self):
        controller = G1WalkController()
        controller.set_trajectory(WalkPlan().move_forward(1, speed=0.25).to_trajectory())  # 4 s nominal: fails after 10 s
        controller.reset(_state(0, 0, 0), None, 0.0)
        controller.step((0, 0, 0), 9.9)
        self.assertEqual(controller.status, WalkStatus.RUNNING)
        controller.step((0, 0, 0), 10.1)
        self.assertEqual(controller.status, WalkStatus.FAILED)

    async def test_stopped_looping_plan_finishes(self):
        controller = G1WalkController()
        controller.set_trajectory(WalkPlan(loop=True).move_forward(1).stop().to_trajectory())
        controller.reset(_state(0, 0, 0), None, 0.0)
        controller.step((1, 0, 0), 1.0)
        self.assertEqual(controller.status, WalkStatus.DONE)

    async def test_forward_outputs_joint_targets(self):
        controller = G1WalkController()
        controller.set_trajectory(WalkPlan().move_forward(1).to_trajectory())
        controller.reset(_state(0, 0, 0), None, 0.0)
        target = controller.forward(_state(0, 0, 0), None, 0.0)
        self.assertEqual(list(target.joints.position_names), list(JOINTS))
        self.assertTrue(np.isfinite(target.joints.positions.numpy()).all())
        self.assertIsNone(target.root)

    async def test_set_trajectory_switches_without_reset(self):
        controller = G1WalkController()
        controller.reset(_state(0, 0, 0), None, 0.0)
        self.assertEqual(controller.status, WalkStatus.IDLE)
        controller.set_trajectory(WalkPlan().step_left(1).to_trajectory())
        controller.forward(_state(5, 5, 0), None, 1.0)  # anchors the new trajectory at (5, 5)
        self.assertEqual(controller.status, WalkStatus.RUNNING)
        controller.step((5, 6, 0), 2.0)
        self.assertEqual(controller.status, WalkStatus.DONE)

    async def test_no_trajectory_is_idle(self):
        controller = G1WalkController()
        controller.reset(_state(0, 0, 0), None, 0.0)
        self.assertEqual(controller.status, WalkStatus.IDLE)
        self.assertEqual(controller.step((0, 0, 0), 0.0), (0.0, 0.0, 0.0))
