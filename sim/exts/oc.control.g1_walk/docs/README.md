# G1 Walk

Makes a Unitree G1 walk a Walk sequence on flat ground while the simulation plays, for ROS benchmark runs.
Enabling the extension does not make any robot walk; a caller starts a `G1Walker` with its own
sequence (e.g. `oc.benchmark.ros_dds`).

Extension ID: `oc.control.g1_walk`

## Run

From the repository root:

1. Install the environments once: `pixi install` (ROS 2) and `pixi install -e sim` (Isaac Sim, torch).
2. Launch Isaac Sim with an extension that starts a walker, e.g. `oc.benchmark.ros_dds`, which
   enables this one:

   ```bash
   pixi run -e sim sim --ext-folder sim/exts --enable oc.benchmark.ros_dds
   ```

   `scripts/sim.py` launches PhysX by default (`isaacsim.exp.full`). Put `--physics newton` first
   to launch the Newton app instead (`isaacsim.exp.full.newton`, PhysX disabled; the scenario's
   PhysicsScene is set up for Newton's MuJoCo solver), and `--dds fast|cyclone|zenoh` to pick the RMW. Or, in a running
   Isaac Sim, open Window > Extensions, add `sim/exts` as a search path, and enable "ROS DDS Benchmark".
3. File > Open `sim/exts/oc.benchmark.ros_dds/usd/simple_g1_scenario.usda`; its G1 is at `/World/G1`.
4. Press Play. The G1 walks `oc.benchmark.ros_dds`'s `SIMPLE_WALK_SEQUENCE`. Stop resets it; the
   next Play starts the sequence again.

If the robot does not move, check that it is at `/World/G1`. Errors are logged to the console as
`G1Walker(/World/G1): stopped after error: ...`, and walking stays off until the next Play.

## Behavior

Once started, every Play makes the walker's G1 walk its sequence from the start. Pressing Stop on the timeline ends the walk. Falls are
not handled. Physics should run at 100 Hz (set in `sim/exts/oc.benchmark.ros_dds/usd/simple_g1_scenario.usda`); the policy
runs every 2nd physics step (50 Hz).

## API

```python
from oc.control.g1_walk import G1Walker, G1WalkController, WalkPlan, command_at

plan = WalkPlan().stop(2).move_forward(1.5).turn_left(90, speed=30).step_right(0.5, speed=0.3)
walker = G1Walker("/World/G1", plan.to_walk_sequence())  # ((seconds, vx, vy, wz), ...)
walker.start()  # walks on every Play until stop()
walker.stop()
```

`WalkPlan` builds a Walk sequence from Walk primitives in the robot's frame. Each method returns
the plan, so calls chain:

| Primitive | Amount | Default speed |
| --- | --- | --- |
| `move_forward`, `move_backward` | meters | 0.5 m/s |
| `step_left`, `step_right` | meters | 0.5 m/s |
| `turn_left`, `turn_right` | degrees | 45 deg/s |
| `stop` | seconds standing still | - |
| `abort` | none: ends the plan | - |

Every primitive except `abort` takes a positive amount, and each move or turn takes an optional `speed=`. Speeds are
capped at the policy's trained range, 1 m/s and 57.3 deg/s; anything else raises `ValueError`. The
sequence repeats forever, so a looping plan should end where it started. A plan that ends with
`abort()` plays once and then stands forever; primitives after `abort()` raise `ValueError`.

`G1WalkController` is the MotionGen controller
(`isaacsim.robot_motion.experimental.motion_generation.BaseController`) and can be stepped directly.

## Policy

`data/motion.pt` is Unitree's pretrained G1 walking policy from
[unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym) (`deploy/pre_train/g1/motion.pt`),
BSD 3-Clause, see `data/LICENSE-unitree_rl_gym`. It drives the 12 leg joints; the waist and arms
are held stiffly at zero because the policy was trained with a rigid upper body.
