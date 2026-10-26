"""Tests of the Kit-free kinematic planner. The package __init__ imports omni, so sonic_policy.py and kinematic_planner.py load by path
under a private stub package instead."""

import importlib
import math
import sys
import time
import types
import unittest
from pathlib import Path

import numpy as np

_PACKAGE_DIR = Path(__file__).parents[1] / "controller"
_STUB = "_g1_sonic_kinematic_planner_pure"
if _STUB not in sys.modules:
    _stub = types.ModuleType(_STUB)
    _stub.__path__ = [str(_PACKAGE_DIR)]
    sys.modules[_STUB] = _stub
kinematic_planner = importlib.import_module(f"{_STUB}.kinematic_planner")
sonic_policy = importlib.import_module(f"{_STUB}.sonic_policy")

_CPU = ["CPUExecutionProvider"]  # deterministic
_SLOW_WALK, _WALK, _RUN, _SQUAT = 1, 2, 3, 4
_GROUND_MODE = 8  # not a standing mode (localmotion_kplanner.hpp:117-139)
_DTYPES = {"tensor(float)": "float32", "tensor(int64)": "int64"}


class _SlowSession:
    """Wraps the real session; run() sleeps first, like a slow inference."""

    def __init__(self, session, delay=0.3):
        self._session = session
        self._delay = delay

    def run(self, *args):
        time.sleep(self._delay)
        return self._session.run(*args)


class TestKinematicPlanner(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.planner = kinematic_planner.KinematicPlanner(providers=_CPU, synchronous=True)

    def setUp(self):
        self.planner.set_cmd_vel(0.0, 0.0, 0.0)
        self.planner.set_mode(kinematic_planner.IDLE)
        self.planner.set_pelvis_height(-1.0)
        self.planner.initialize(sonic_policy.DEFAULT_POSE)

    def test_feed_covers_every_model_input(self):
        session = self.planner._session
        feed = kinematic_planner.model_feed(self.planner.last_inputs)
        for model_input in session.get_inputs():
            self.assertIn(model_input.name, feed)
            self.assertEqual(list(feed[model_input.name].shape), model_input.shape, model_input.name)
            self.assertEqual(feed[model_input.name].dtype.name, _DTYPES[model_input.type], model_input.name)
        self.assertEqual(len(feed), len(session.get_inputs()))

    def test_initialize_plans_a_finite_trajectory(self):
        plan = self.planner.mujoco_qpos
        self.assertEqual(plan.shape[1], 36)
        self.assertTrue(2 <= len(plan) <= 64)
        self.assertTrue(np.isfinite(plan).all())
        np.testing.assert_allclose(np.linalg.norm(plan[:, 3:7], axis=1), 1.0, atol=1e-3)
        self.assertEqual(self.planner.plans, 1)

    def test_wz_keeps_integrating_without_new_messages(self):
        self.planner.set_cmd_vel(0.0, 0.0, 0.5)
        for _ in range(200):
            self.planner.step(0.005)
        self.assertAlmostEqual(self.planner.facing_yaw, 0.5, places=6)

    def test_movement_direction_is_body_relative(self):
        self.planner.set_mode(_WALK)
        self.planner.set_cmd_vel(1.0, 0.0, math.pi / 2)   # 1 s at 90 deg/s, then hold the heading
        for _ in range(200):
            self.planner.step(0.005)
        self.planner.set_cmd_vel(1.0, 0.0, 0.0)
        for _ in range(20):
            self.planner.step(0.005)
        inputs = self.planner.last_inputs
        self.assertAlmostEqual(self.planner.facing_yaw, math.pi / 2, places=6)
        np.testing.assert_allclose(inputs.movement_direction, [0.0, 1.0, 0.0], atol=1e-6)
        np.testing.assert_allclose(inputs.facing_direction, [0.0, 1.0, 0.0], atol=1e-6)
        self.assertAlmostEqual(inputs.speed, 1.0)
        self.assertEqual(inputs.mode, _WALK)

    def test_idle_or_near_zero_speed_stands(self):
        self.planner.set_cmd_vel(1.0, 0.0, 0.0)  # idle mode
        self._replan()
        self.assertEqual(self.planner.last_inputs.speed, -1.0)
        np.testing.assert_array_equal(self.planner.last_inputs.movement_direction, np.zeros(3))
        self.planner.set_mode(_WALK)
        self.planner.set_cmd_vel(0.001, 0.0, 0.0)
        self._replan()
        self.assertEqual(self.planner.last_inputs.speed, -1.0)
        np.testing.assert_array_equal(self.planner.last_inputs.movement_direction, np.zeros(3))

    def test_speed_is_clamped_per_mode(self):
        for mode, command, expected in ((_SLOW_WALK, 5.0, 0.8), (_WALK, 0.1, 0.8), (_RUN, 2.0, 2.0)):
            self.planner.set_mode(mode)
            self.planner.set_cmd_vel(command, 0.0, 0.0)
            self._replan()
            self.assertAlmostEqual(self.planner.last_inputs.speed, expected, msg=f"mode {mode}")

    def test_squat_while_moving_sends_default_speed(self):
        self.planner.set_mode(_SQUAT)
        self.planner.set_cmd_vel(1.0, 0.0, 0.0)
        self._replan()
        self.assertEqual(self.planner.last_inputs.speed, -1.0)
        np.testing.assert_allclose(self.planner.last_inputs.movement_direction, [1.0, 0.0, 0.0], atol=1e-6)

    def test_movement_direction_is_binned_to_8_directions(self):
        self.planner.set_mode(_WALK)
        for degrees, expected in ((30, 45), (10, 0), (-100, -90), (170, 180)):
            angle = math.radians(degrees)
            self.planner.set_cmd_vel(math.cos(angle), math.sin(angle), 0.0)
            self._replan()
            direction = self.planner.last_inputs.movement_direction
            np.testing.assert_allclose(direction, [math.cos(math.radians(expected)), math.sin(math.radians(expected)), 0.0], atol=1e-6, err_msg=f"{degrees} deg")

    def test_stopped_in_a_standing_mode_goes_idle_and_moving_resumes(self):
        self.planner.set_mode(_WALK)
        self.planner.set_pelvis_height(0.5)
        self.planner.set_cmd_vel(0.0, 0.0, 0.0)
        self._replan()
        inputs = self.planner.last_inputs
        self.assertEqual((inputs.mode, inputs.speed, inputs.pelvis_height), (kinematic_planner.IDLE, -1.0, -1.0))
        np.testing.assert_array_equal(inputs.movement_direction, np.zeros(3))
        self.assertEqual(self.planner._mode, _WALK)
        self.planner.set_cmd_vel(1.0, 0.0, 0.0)
        self._replan()
        self.assertEqual(self.planner.last_inputs.mode, _WALK)
        self.assertAlmostEqual(self.planner.last_inputs.pelvis_height, 0.5)

    def test_stopped_in_a_ground_mode_keeps_mode_and_height(self):
        self.planner.set_mode(_GROUND_MODE)
        self.planner.set_pelvis_height(0.5)
        self.planner.set_cmd_vel(0.0, 0.0, 0.0)
        self._replan()
        inputs = self.planner.last_inputs
        self.assertEqual((inputs.mode, inputs.speed), (_GROUND_MODE, 0.0))
        self.assertAlmostEqual(inputs.pelvis_height, 0.5)

    def test_stale_cmd_vel_stands_and_stops_turning(self):
        self.planner.set_mode(_WALK)
        self.planner.set_cmd_vel(1.0, 0.0, 0.5)
        for _ in range(240):  # 1.2 s: the command times out after 1 s
            self.planner.step(0.005)
        yaw = self.planner.facing_yaw
        self.assertAlmostEqual(yaw, 0.5, places=2)
        self.assertEqual(self.planner.last_inputs.mode, kinematic_planner.IDLE)
        self.assertEqual(self.planner.last_inputs.speed, -1.0)
        for _ in range(100):
            self.planner.step(0.005)
        self.assertEqual(self.planner.facing_yaw, yaw)

    def test_tiny_wz_does_not_turn(self):
        self.planner.set_cmd_vel(0.0, 0.0, 0.005)
        for _ in range(100):
            self.planner.step(0.005)
        self.assertEqual(self.planner.facing_yaw, 0.0)

    def test_plans_at_10_hz_of_sim_time(self):
        for _ in range(200):
            self.planner.step(0.005)
        self.assertEqual(self.planner.plans, 1 + 10)

    def test_reference_shapes_and_standing_pose(self):
        joints, velocities, quats = self.planner.reference()
        self.assertEqual(joints.shape, (10, 29))
        self.assertEqual(velocities.shape, (10, 29))
        self.assertEqual(quats.shape, (10, 4))
        for array in (joints, velocities, quats):
            self.assertTrue(np.isfinite(array).all())
        np.testing.assert_allclose(np.linalg.norm(quats, axis=1), 1.0, atol=1e-3)
        self.assertLess(np.abs(joints[0] - sonic_policy.DEFAULT_POSE_IL).max(), 0.1)  # frame 0 is the measured pose

    def test_reference_before_initialize_raises(self):
        with self.assertRaises(RuntimeError):
            kinematic_planner.KinematicPlanner(providers=_CPU, synchronous=True).reference()

    def test_resample_50hz_frame_count_quats_and_velocities(self):
        plan = self.planner.mujoco_qpos
        motion = kinematic_planner.resample_50hz(plan)
        self.assertEqual(len(motion), len(plan) * 50 // 30)
        np.testing.assert_allclose(np.linalg.norm(motion.quats, axis=1), 1.0, atol=1e-3)
        np.testing.assert_allclose(motion.joint_velocities[:-1], np.diff(motion.joints, axis=0) * 50, atol=1e-9)

    def test_close_shuts_the_worker_down(self):
        planner = self._async_planner()
        planner.close()
        self.assertTrue(planner._executor._shutdown)

    def test_late_plan_never_blocks_and_is_spliced_without_a_jump(self):
        planner = self._async_planner()
        planner._session = _SlowSession(planner._session)
        slowest = 0.0
        for _ in range(25):  # 0.5 s of sim time; the 0.3 s plan lands during it
            start = time.perf_counter()
            planner.reference()
            planner.step(0.02)
            slowest = max(slowest, time.perf_counter() - start)
        self.assertLess(slowest, 0.02)
        self.assertGreater(planner.skipped_replans, 0)
        deadline = time.monotonic() + 5.0
        while planner._pending is not None and time.monotonic() < deadline:
            planner.reference()
            time.sleep(0.01)
        self.assertGreaterEqual(planner.late_plans, 1)
        before = planner.reference()[0]
        planner.step(0.02)
        self.assertLess(np.abs(planner.reference()[0] - before).max(), 0.1)  # the same reference rows, one frame later

    def test_step_before_initialize_raises(self):
        with self.assertRaises(RuntimeError):
            kinematic_planner.KinematicPlanner(providers=_CPU).step(0.02)

    def test_invalid_mode_raises(self):
        with self.assertRaises(ValueError):
            self.planner.set_mode(kinematic_planner.NUM_MODES)

    def _async_planner(self):
        planner = kinematic_planner.KinematicPlanner(providers=_CPU)
        self.addCleanup(planner.close)
        planner.initialize(sonic_policy.DEFAULT_POSE)
        return planner

    def _replan(self):
        for _ in range(20):
            self.planner.step(0.005)


if __name__ == "__main__":
    unittest.main()
