"""Generate test/data/*.f64 (raw float64) from the Python reference, oc.control.g1_sonic, for test_golden.cpp.

    .pixi/envs/sim/bin/python ros_ws/src/oc_sonic/test/make_goldens.py

The extension's package imports carb and Isaac Sim, so its modules are loaded here as a stub package by file path.
"""

import sys
import types
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
EXTENSION = ROOT / "sim" / "exts" / "oc.control.g1_sonic" / "oc" / "control" / "g1_sonic"
MODELS = ROOT / "sim" / "exts" / "oc.control.g1_sonic" / "data" / "sonic"
OUT = Path(__file__).resolve().parent / "data"


def load_reference():
    """The extension's controllers as modules of a stub package, without running its __init__ (carb, Isaac Sim)."""
    carb = types.ModuleType("carb")
    carb.log_warn = print
    sys.modules["carb"] = carb
    for name, path in (("g1s", EXTENSION), ("g1s.controllers", EXTENSION / "controllers")):
        package = types.ModuleType(name)
        package.__path__ = [str(path)]
        sys.modules[name] = package
    from g1s.controllers import kinematic_planner as planner
    from g1s.controllers import sonic

    return sonic, planner


def save(name, *arrays):
    np.concatenate([np.asarray(a, np.float64).reshape(-1) for a in arrays]).tofile(OUT / f"{name}.f64")


def random_qpos(rng, n):
    """A smooth fake planner output [n, 36]: a drifting root, a wobbling unit quaternion, joints near the standing pose."""
    t = np.linspace(0, 1, n)[:, None]
    qpos = np.zeros((n, 36))
    qpos[:, :3] = t * rng.normal(size=3) + [0, 0, 0.79]
    yaw = t[:, 0] * rng.normal() + 0.3
    qpos[:, 3:7] = np.stack([np.cos(yaw / 2), 0.05 * np.sin(5 * t[:, 0]), 0.0 * yaw, np.sin(yaw / 2)], 1)
    qpos[:, 3:7] /= np.linalg.norm(qpos[:, 3:7], axis=1, keepdims=True)
    qpos[:, 7:] = 0.2 * np.sin(t * rng.normal(size=29) * 6 + rng.normal(size=29))
    return qpos


def motion_arrays(motion):
    return motion.positions, motion.quats, motion.joints, motion.joint_velocities


def main():
    sonic, planner = load_reference()
    OUT.mkdir(exist_ok=True)
    rng = np.random.default_rng(0)

    # Resampling the planner's 30 Hz qpos to 50 Hz.
    qpos_a, qpos_b = random_qpos(rng, 40), random_qpos(rng, 50)
    old, new = planner._resample(qpos_a), planner._resample(qpos_b)
    save("resample", qpos_a, *motion_arrays(old))
    save("resample_b", qpos_b, *motion_arrays(new))

    # Cross-fading a plan into the current motion: on time, and late (start before the current frame).
    blender = object.__new__(planner.KinematicPlanner)
    blender.motion = old
    for name, frame, start in (("blend_ontime", 10, 12), ("blend_late", 10, 3)):
        blender.frame = frame
        save(name, *motion_arrays(blender._blend(new, start)))

    # The planner's context sampled from a motion at frame 14.
    context = np.zeros((4, 36))
    for n in range(4):
        position, quat, joints = old.sample(14 + n * planner.CONTROL_RATE / planner.PLANNER_RATE)
        context[n, :3], context[n, 3:7], context[n, 7:] = position, quat, joints[sonic.ISAACLAB_OF_MUJOCO]
    save("context", context)

    # The tracking policy over 12 steps: encoder token of the first reference, then the decoder's joint targets.
    policy = sonic.SonicPolicy(MODELS)
    base_yaw = 0.7
    policy.reset(sonic.yaw_quat(base_yaw), old.quats[0])
    steps = []
    for step in range(12):
        q = sonic.DEFAULT_POSE + 0.05 * rng.normal(size=29)
        dq = 0.1 * rng.normal(size=29)
        base = sonic.yaw_quat(base_yaw + 0.02 * step)
        base = base + 0.02 * rng.normal(size=4)
        base /= np.linalg.norm(base)
        gyro = 0.1 * rng.normal(size=3)
        frames = np.minimum(step + 5 * np.arange(10), len(old) - 1)
        reference = (old.joints[frames], old.joint_velocities[frames], old.quats[frames])
        if step == 0:
            token = policy.encode(base, reference)
        targets = policy.act(q, dq, base, gyro, reference)
        steps.append(np.concatenate([q, dq, base, gyro, *(np.asarray(r).reshape(-1) for r in reference), targets]))
    save("policy", token, np.array(steps))

    # The ONNX planner: standing and walking.
    kinematic = planner.KinematicPlanner(MODELS)
    stand = np.zeros((4, 36), np.float32)
    stand[:, 2], stand[:, 3], stand[:, 7:] = sonic.DEFAULT_HEIGHT, 1.0, sonic.DEFAULT_POSE
    for name, command in (
        ("plan_idle", (planner.IDLE, None, 0.0, 0.0, -1.0)),
        ("plan_walk", (planner.WALK, 0.3, 0.2, 0.9, -1.0)),
    ):
        save(name, *motion_arrays(kinematic._plan(stand, command)))
    print("wrote", sorted(p.name for p in OUT.iterdir()))


if __name__ == "__main__":
    main()
