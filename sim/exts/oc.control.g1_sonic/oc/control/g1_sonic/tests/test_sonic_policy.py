"""Kit-free tests of SONIC's policy side: sonic_policy.py helpers, the real ONNX models and SonicController. The package __init__
imports omni, so the modules load by path under a private stub package, and where Isaac Sim is not importable, stubs
stand in for the Articulation and the SimulationManager."""

import importlib
import sys
import types
import unittest
from pathlib import Path
from typing import ClassVar

import numpy as np

_PACKAGE_DIR = Path(__file__).parents[1] / "controller"
_CPU = ["CPUExecutionProvider"]  # deterministic


def _install_kit_stubs() -> None:
    try:
        importlib.import_module("isaacsim.core.experimental.prims")
        importlib.import_module("isaacsim.core.simulation_manager")
        return
    except ImportError:
        pass

    class Articulation: ...

    class SimulationManager:
        @staticmethod
        def get_active_physics_engine():
            return "physx"

    for name, attrs in (
        ("isaacsim.core.experimental.prims", {"Articulation": Articulation}),
        ("isaacsim.core.simulation_manager", {"SimulationManager": SimulationManager}),
    ):
        module = sys.modules.get(name) or types.ModuleType(name)
        module.__dict__.update(attrs)
        sys.modules[name] = module
        parent = name.rpartition(".")[0]
        while parent:
            sys.modules.setdefault(parent, types.ModuleType(parent)).__path__ = []
            parent = parent.rpartition(".")[0]


_install_kit_stubs()
_STUB = "_g1_sonic_policy_pure"
if _STUB not in sys.modules:
    _stub = types.ModuleType(_STUB)
    _stub.__path__ = [str(_PACKAGE_DIR)]
    sys.modules[_STUB] = _stub
sonic_policy = importlib.import_module(f"{_STUB}.sonic_policy")
sonic_controller = importlib.import_module(f"{_STUB}.sonic_controller")

_IDENTITY = np.array([1.0, 0.0, 0.0, 0.0])
_N = sonic_policy.NUM_JOINTS


def _yaw_quat(angle: float) -> np.ndarray:
    return np.array([np.cos(angle / 2), 0.0, 0.0, np.sin(angle / 2)])


def _standing_reference():
    return np.tile(sonic_policy.DEFAULT_POSE_IL, (10, 1)), np.zeros((10, _N)), np.tile(_IDENTITY, (10, 1))


def _upright_histories():
    histories = sonic_policy.DecoderHistories()
    histories.push(np.zeros(3), np.zeros(_N), np.zeros(_N), np.zeros(_N), sonic_policy.gravity_dir(_IDENTITY))
    return histories


class TestConstants(unittest.TestCase):
    # name -> (kp, kd, action scale, effort, velocity, armature), the values of the reference tables
    _TABLE: ClassVar[dict] = {
        "hip_pitch": (99.0984, 6.3088, 0.350661, 139, 20, 0.025102),
        "hip_roll": (99.0984, 6.3088, 0.350661, 139, 20, 0.025102),
        "hip_yaw": (40.1792, 2.5579, 0.547546, 88, 32, 0.010178),
        "knee": (99.0984, 6.3088, 0.350661, 139, 20, 0.025102),
        "ankle_pitch": (28.5012, 1.8144, 0.438577, 50, 37, 0.007219),
        "ankle_roll": (28.5012, 1.8144, 0.438577, 50, 37, 0.007219),
        "waist_yaw": (40.1792, 2.5579, 0.547546, 88, 32, 0.010178),
        "waist_roll": (28.5012, 1.8144, 0.438577, 50, 37, 0.007219),
        "waist_pitch": (28.5012, 1.8144, 0.438577, 50, 37, 0.007219),
        "shoulder_pitch": (14.2506, 0.9072, 0.438577, 25, 37, 0.003610),
        "shoulder_roll": (14.2506, 0.9072, 0.438577, 25, 37, 0.003610),
        "shoulder_yaw": (14.2506, 0.9072, 0.438577, 25, 37, 0.003610),
        "elbow": (14.2506, 0.9072, 0.438577, 25, 37, 0.003610),
        "wrist_roll": (14.2506, 0.9072, 0.438577, 25, 37, 0.003610),
        "wrist_pitch": (16.7783, 1.0681, 0.074501, 5, 22, 0.004250),
        "wrist_yaw": (16.7783, 1.0681, 0.074501, 5, 22, 0.004250),
    }

    def test_derived_gains_match_the_table(self):
        for i, name in enumerate(sonic_policy.MUJOCO_JOINTS):
            kind = name.removeprefix("left_").removeprefix("right_").removesuffix("_joint")
            kp, kd, scale, effort, vel, armature = self._TABLE[kind]
            self.assertAlmostEqual(sonic_policy.STIFFNESS[i], kp, delta=1e-3 * kp, msg=name)
            self.assertAlmostEqual(sonic_policy.DAMPING[i], kd, delta=1e-3 * kd, msg=name)
            self.assertAlmostEqual(sonic_policy.ACTION_SCALE[i], scale, delta=1e-3 * scale, msg=name)
            self.assertEqual((sonic_policy.EFFORT_LIMIT[i], sonic_policy.VELOCITY_LIMIT[i], sonic_policy.ARMATURE[i]), (effort, vel, armature))

    def test_isaaclab_order_versions(self):
        for name in ("hip_pitch", "wrist_yaw"):
            for side in ("left", "right"):
                joint = f"{side}_{name}_joint"
                self.assertEqual(sonic_policy.ACTION_SCALE_IL[sonic_policy.ISAACLAB_JOINTS.index(joint)], sonic_policy.ACTION_SCALE[sonic_policy.MUJOCO_JOINTS.index(joint)])
        self.assertEqual(sonic_policy.DEFAULT_POSE_IL[sonic_policy.ISAACLAB_JOINTS.index("right_shoulder_roll_joint")], -0.2)

    def test_dims(self):
        self.assertEqual((sonic_policy.ENCODER_OBS_DIM, sonic_policy.DECODER_OBS_DIM, sonic_policy.NUM_JOINTS), (1762, 994, 29))


class TestObservations(unittest.TestCase):
    def test_encoder_layout(self):
        rng = np.random.default_rng(0)
        joints, vels = rng.normal(size=(10, _N)), rng.normal(size=(10, _N))
        obs = sonic_policy.encoder_obs(joints, vels, np.tile(_IDENTITY, (10, 1)), _IDENTITY, _IDENTITY)
        self.assertEqual(obs.shape, (1762,))
        self.assertEqual(obs.dtype, np.float32)
        np.testing.assert_array_equal(obs[0:4], 0)
        np.testing.assert_allclose(obs[4:294], joints.reshape(-1), atol=1e-6)
        np.testing.assert_allclose(obs[294:584], vels.reshape(-1), atol=1e-6)
        np.testing.assert_array_equal(obs[584:601], 0)
        np.testing.assert_allclose(obs[601:661], np.tile([1, 0, 0, 1, 0, 0], 10), atol=1e-6)
        np.testing.assert_array_equal(obs[661:], 0)

    def test_anchor_is_the_relative_rotation(self):
        quats = np.tile(_IDENTITY, (10, 1))
        obs = sonic_policy.encoder_obs(*_standing_reference()[:2], quats, _yaw_quat(np.pi / 2), _IDENTITY)
        np.testing.assert_allclose(obs[601:607], [0, 1, -1, 0, 0, 0], atol=1e-6)  # Rz(-90 deg), first two columns

    def test_delta_heading_maps_planner_frame_to_world(self):
        heading = sonic_policy.delta_heading(_yaw_quat(np.pi / 2), _IDENTITY)
        np.testing.assert_allclose(heading, _yaw_quat(np.pi / 2), atol=1e-9)
        obs = sonic_policy.encoder_obs(*_standing_reference()[:2], np.tile(_IDENTITY, (10, 1)), _yaw_quat(np.pi / 2), heading)
        np.testing.assert_allclose(obs[601:607], [1, 0, 0, 1, 0, 0], atol=1e-6)

    def test_yaw_ignores_tilt(self):
        tilt = np.array([np.cos(0.2), np.sin(0.2), 0.0, 0.0])  # roll about x
        q = sonic_policy.quat_mul(_yaw_quat(0.7), tilt)
        np.testing.assert_allclose(sonic_policy.quat_yaw(q), _yaw_quat(0.7), atol=1e-9)

    def test_gravity_and_body_rate(self):
        np.testing.assert_allclose(sonic_policy.gravity_dir(_IDENTITY), [0, 0, -1], atol=1e-12)
        np.testing.assert_allclose(sonic_policy.gravity_dir(_yaw_quat(1.0)), [0, 0, -1], atol=1e-12)
        np.testing.assert_allclose(sonic_policy.ang_vel_body(_yaw_quat(np.pi / 2), [1.0, 0.0, 0.5]), [0.0, -1.0, 0.5], atol=1e-12)

    def test_history_fills_then_shifts_oldest_first(self):
        histories = sonic_policy.DecoderHistories()
        push = lambda v: histories.push(np.full(3, v), np.full(_N, v), np.full(_N, v), np.full(_N, v), np.full(3, v))
        push(1.0)
        np.testing.assert_array_equal(histories.flat(1), 1.0)
        push(2.0)
        push(3.0)
        column = histories.flat(1).reshape(10, _N)[:, 0]
        np.testing.assert_array_equal(column, [1] * 8 + [2, 3])
        histories.reset()
        push(9.0)
        np.testing.assert_array_equal(histories.flat(0), 9.0)

    def test_decoder_layout_is_term_major(self):
        histories = sonic_policy.DecoderHistories()
        histories.push(np.full(3, 1.0), np.full(_N, 2.0), np.full(_N, 3.0), np.full(_N, 4.0), np.full(3, 5.0))
        obs = sonic_policy.decoder_obs(np.full(64, 0.5), histories)
        self.assertEqual(obs.shape, (994,))
        for lo, hi, value in ((0, 64, 0.5), (64, 94, 1), (94, 384, 2), (384, 674, 3), (674, 964, 4), (964, 994, 5)):
            np.testing.assert_array_equal(obs[lo:hi], value)

    def test_action_to_target(self):
        np.testing.assert_allclose(sonic_policy.action_to_target(np.zeros(_N)), sonic_policy.DEFAULT_POSE_IL)
        np.testing.assert_allclose(sonic_policy.action_to_target(np.full(_N, 2.0)), sonic_policy.DEFAULT_POSE_IL + 2.0 * sonic_policy.ACTION_SCALE_IL)


class TestModels(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.encoder = sonic_policy._session(sonic_policy.SONIC_DIR / "model_encoder.onnx", _CPU)
        cls.decoder = sonic_policy._session(sonic_policy.SONIC_DIR / "model_decoder.onnx", _CPU)

    def _max_action(self, joints, vels=None, quats=None):
        _, ref_vels, ref_quats = _standing_reference()
        obs = sonic_policy.encoder_obs(joints, ref_vels if vels is None else vels, ref_quats if quats is None else quats, _IDENTITY, _IDENTITY)
        token = self.encoder.run(None, {"obs_dict": obs[None]})[0][0]
        self.assertEqual(token.shape, (64,))
        action = self.decoder.run(None, {"obs_dict": sonic_policy.decoder_obs(token, _upright_histories())[None]})[0]
        self.assertEqual(action.shape, (1, _N))
        self.assertTrue(np.isfinite(action).all())
        return float(np.abs(action).max())

    def test_standing_is_calm(self):
        self.assertLess(self._max_action(_standing_reference()[0]), 0.4)

    def test_reference_in_mujoco_order_is_clearly_wrong(self):
        self.assertGreater(self._max_action(np.tile(sonic_policy.DEFAULT_POSE, (10, 1))), 1.0)

    def test_reference_minus_default_is_clearly_wrong(self):
        self.assertGreater(self._max_action(np.zeros((10, _N))), 1.0)


class _StubPlanner:
    def __init__(self):
        self.calls = []
        self.initialized_with = None
        self.prim_path = None
        self.fail_initialize = False
        self.command = {"mode": 7, "cmd_vel": (9.0, 9.0, 9.0), "pelvis_height": 9.0}  # a non-idle start

    def set_cmd_vel(self, vx, vy, wz):
        self.command["cmd_vel"] = (vx, vy, wz)

    def set_mode(self, mode):
        if not 0 <= mode < 27:
            raise ValueError(f"mode must be in [0, 27), got {mode}")
        self.command["mode"] = mode

    def set_pelvis_height(self, height):
        self.command["pelvis_height"] = height

    def initialize(self, joint_positions, robot_prim_path=None):
        if self.fail_initialize:
            raise RuntimeError("boom")
        self.calls.append("initialize")
        self.initialized_with = np.asarray(joint_positions)
        self.prim_path = robot_prim_path

    def reference(self):
        self.calls.append("reference")
        return _standing_reference()

    def step(self, dt):
        self.calls.append(("step", dt))

    def close(self):
        self.calls.append("close")


# USD-like joint space: reversed body joints plus two extra (hand) DOFs the controller must leave alone.
_JOINT_SPACE = list(reversed(sonic_policy.ISAACLAB_JOINTS)) + ["left_hand_joint", "right_hand_joint"]


class _FakeArticulation:
    def __init__(self, prim_path):
        self.prim_path = prim_path
        self.dof_names = _JOINT_SPACE
        self.calls = []
        for name in ("set_dof_gains", "set_dof_armatures", "set_dof_max_efforts", "set_dof_max_velocities"):
            setattr(self, name, lambda *a, _n=name, **k: self.calls.append((_n, a, k)))


class TestSonicController(unittest.TestCase):
    def setUp(self):
        self.planner = _StubPlanner()
        self._saved = (sonic_controller.Articulation, sonic_controller.KinematicPlanner)
        sonic_controller.Articulation = _FakeArticulation
        sonic_controller.KinematicPlanner = lambda providers=None: self.planner
        self.controller = sonic_controller.SonicController("/World/G1", providers=_CPU)
        self.state = (sonic_policy.DEFAULT_POSE_IL.copy(), np.zeros(_N), _IDENTITY, np.zeros(3))
        self.written = []
        self.controller._read_state = lambda: self.state
        self.controller._write_targets = self.written.append

    def tearDown(self):
        sonic_controller.Articulation, sonic_controller.KinematicPlanner = self._saved

    def _forward(self, n, dt=0.005):
        for _ in range(n):
            self.controller.forward(dt)

    def test_first_forward_starts_the_controller(self):
        self.controller.forward(0.005)
        self.assertEqual(self.planner.prim_path, "/World/G1")
        np.testing.assert_allclose(self.planner.initialized_with, sonic_policy.DEFAULT_POSE, atol=1e-6)  # MuJoCo order
        calls = {name: (args, kwargs) for name, args, kwargs in self.controller._articulation.calls}
        self.assertEqual(set(calls), {"set_dof_gains", "set_dof_armatures", "set_dof_max_efforts", "set_dof_max_velocities"})
        indices = calls["set_dof_gains"][1]["dof_indices"]
        self.assertEqual([_JOINT_SPACE[i] for i in indices], list(sonic_policy.ISAACLAB_JOINTS))
        np.testing.assert_array_equal(calls["set_dof_gains"][0][0], sonic_policy.STIFFNESS_IL)
        self.assertEqual(self.planner.calls.count("initialize"), 1)

    def test_policy_runs_on_ticks_0_4_8_and_holds_between(self):
        self._forward(9)
        self.assertEqual(len(self.written), 3)
        self.assertEqual(self.planner.calls.count("reference"), 3)
        self._forward(3)
        self.assertEqual(len(self.written), 3)  # ticks 9-11 write nothing
        self._forward(1)
        self.assertEqual(len(self.written), 4)

    def test_reference_is_read_before_the_planner_steps(self):
        self.controller.forward(0.005)
        self.assertEqual(self.planner.calls, ["initialize", "reference", ("step", sonic_policy.CONTROL_DT)])

    def test_heading_is_captured_once_and_reset_rearms_it(self):
        self.state = (self.state[0], self.state[1], _yaw_quat(0.5), self.state[3])
        self._forward(1)
        heading = self.controller._heading.copy()
        np.testing.assert_allclose(heading, _yaw_quat(0.5), atol=1e-6)
        self.state = (self.state[0], self.state[1], _yaw_quat(1.0), self.state[3])
        self._forward(4)  # the next policy tick
        np.testing.assert_array_equal(self.controller._heading, heading)
        self.controller.reset()
        self.controller.forward(0.005)
        np.testing.assert_allclose(self.controller._heading, _yaw_quat(1.0), atol=1e-6)

    def test_non_finite_action_raises(self):
        class Nan:
            def run(self, *_):
                return [np.full((1, _N), np.nan, dtype=np.float32)]

        self.controller._decoder = Nan()
        with self.assertRaises(ValueError):
            self.controller.forward(0.005)

    def test_reset_restarts_from_tick_0(self):
        self._forward(2)
        self.controller.reset()
        self.written.clear()
        self.controller.forward(0.005)
        self.assertEqual(self.planner.calls.count("initialize"), 2)
        self.assertEqual(self.controller._tick, 1)
        self.assertEqual(len(self.written), 1)  # tick 0 is due again

    def test_close_closes_planner_and_blocks_forward(self):
        self.controller.forward(0.005)
        self.controller.close()
        self.assertEqual(self.planner.calls[-1], "close")
        with self.assertRaises(RuntimeError):
            self.controller.forward(0.005)

    def test_command_forwards_all_values(self):
        self.controller.command(sonic_controller.SonicMode.WALK, 0.5, -0.1, 0.3, 0.7)
        self.assertEqual(self.planner.command, {"mode": 2, "cmd_vel": (0.5, -0.1, 0.3), "pelvis_height": 0.7})
        self.controller.command(4)  # defaults: zero velocity, pelvis height decided by the mode
        self.assertEqual(self.planner.command, {"mode": 4, "cmd_vel": (0.0, 0.0, 0.0), "pelvis_height": -1.0})

    def test_bad_mode_raises_and_keeps_the_previous_command(self):
        self.controller.command(sonic_controller.SonicMode.RUN, 1.0, 0.0, 0.0, 0.5)
        before = dict(self.planner.command)
        for mode in (27, -1):
            with self.assertRaises(ValueError):
                self.controller.command(mode, 2.0, 2.0, 2.0, 2.0)
        self.assertEqual(self.planner.command, before)

    def test_reset_clears_the_command_to_idle(self):
        self.controller.command(sonic_controller.SonicMode.WALK, 1.0, 0.5, 0.2, 0.6)
        self.controller.reset()
        self.assertEqual(self.planner.command, {"mode": 0, "cmd_vel": (0.0, 0.0, 0.0), "pelvis_height": -1.0})

    def test_ros_flag_selects_the_prim_path_given_to_the_planner(self):
        self.controller.forward(0.005)
        self.assertEqual(self.planner.prim_path, "/World/G1")
        quiet = sonic_controller.SonicController("/World/G1", ros=False, providers=_CPU)
        quiet._read_state = lambda: self.state
        quiet._write_targets = self.written.append
        quiet.forward(0.005)
        self.assertIsNone(self.planner.prim_path)

    def test_command_after_close_raises(self):
        self.controller.close()
        with self.assertRaises(RuntimeError):
            self.controller.command(sonic_controller.SonicMode.WALK)

    def test_sonic_mode_values_match_the_mode_table(self):
        names = [
            "IDLE", "SLOW_WALK", "WALK", "RUN", "SQUAT", "KNEEL_TWO_LEGS", "KNEEL_ONE_LEG", "LYING_FACE_DOWN", "HAND_CRAWL",
            "BOXING_IDLE", "BOXING_WALK", "BOXING_LEFT_JAB", "BOXING_RIGHT_JAB", "BOXING_RANDOM_PUNCHES", "ELBOW_CRAWL",
            "BOXING_LEFT_HOOK", "BOXING_RIGHT_HOOK", "HAPPY_WALK", "STEALTH_WALK", "INJURED_WALK", "CAREFUL_WALK",
            "OBJECT_CARRYING", "CROUCH_WALK", "HAPPY_DANCE_WALK", "ZOMBIE_WALK", "POINT_WALK", "SCARED_WALK",
        ]
        modes = list(sonic_controller.SonicMode)
        self.assertEqual(len(modes), 27)
        self.assertEqual([int(m) for m in modes], list(range(27)))
        self.assertEqual([m.name for m in modes], names)

    def test_command_accepts_every_sonic_mode(self):
        for mode in sonic_controller.SonicMode:
            self.controller.command(mode)
            self.assertEqual(self.planner.command["mode"], int(mode))

    def test_standing_forward_with_the_real_models_returns_finite_targets(self):
        self.controller.forward(0.005)
        (target,) = self.written
        self.assertEqual(target.shape, (_N,))
        self.assertTrue(np.isfinite(target).all())
        action = (target - sonic_policy.DEFAULT_POSE_IL) / sonic_policy.ACTION_SCALE_IL
        self.assertLess(np.abs(action).max(), 0.4)

    def test_real_async_planner_keeps_standing_calm(self):
        real_planner = sonic_controller.KinematicPlanner = self._saved[1]  # tearDown restores the saved pair
        controller = sonic_controller.SonicController("/World/G1", ros=False, providers=_CPU)
        self.addCleanup(controller.close)
        self.assertIsInstance(controller._kplanner, real_planner)
        written = []
        controller._read_state = lambda: self.state
        controller._write_targets = written.append
        for _ in range(100):  # 25 policy ticks
            controller.forward(0.005)
        self.assertEqual(len(written), 25)
        targets = np.array(written)
        self.assertTrue(np.isfinite(targets).all())
        # The fake robot never moves while the planner's idle motion drifts off the default pose, so the action grows; only bound it.
        self.assertLess(np.abs((targets - sonic_policy.DEFAULT_POSE_IL) / sonic_policy.ACTION_SCALE_IL).max(), 3.0)
        self.assertGreater(controller._kplanner.plans, 1)


if __name__ == "__main__":
    unittest.main()
