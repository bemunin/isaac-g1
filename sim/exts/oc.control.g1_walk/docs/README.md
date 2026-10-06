# G1 Walk

Makes a Unitree G1 follow a Walk trajectory on flat ground while the simulation plays. It turns Walk
primitives (move forward, step left, turn right, ...) or a planner's poses into Walk goals, and walks
them with Unitree's walking policy, correcting heading from the pelvis IMU and position from the
articulation root.
Enabling the extension does not make any robot walk; a caller starts a `G1Walker` and hands it a
trajectory (e.g. `oc.benchmark.ros_dds`).

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
4. Press Play. The G1 walks `oc.benchmark.ros_dds`'s `SIMPLE_WALK_TRAJECTORY`. Stop resets it; the
   next Play starts the trajectory again.

If the robot does not move, check that it is at `/World/G1`. Errors are logged to the console as
`G1Walker(/World/G1): stopped after error: ...`, and walking stays off until the next Play.

## Behavior

Each policy step (50 Hz), `G1WalkController` reads the robot's state: orientation and body angular
velocity from the IMU at `<prim>/pelvis/imu_sensor`, x, y and joints from the articulation (simulation
ground truth). It steers
toward the current Walk goal: position error becomes body-frame `vx`, `vy` (so a goal beside the
robot is reached by stepping sideways), and heading error becomes `wz`. Each is capped at the
goal's speed. On top of that, a PI correction on the robot's distance from the current segment of
the Walk trajectory (previous goal to current goal) pushes it back onto the line at up to 0.2 m/s;
its integral cancels the policy's sideways drift. A goal is reached within 0.10 m and 3°, then held for its `hold` seconds. A goal not
reached within 2 × its nominal time + 2 s fails the trajectory, and the robot stands.
The walking policy then turns the command into joint targets; it reads gravity and body rates from the
same IMU.

Once started, every Play restarts the current trajectory from its first goal, measured from the robot's
pose at that moment. Pressing Stop on the timeline ends the walk. Falls are not handled. Physics
should run at 100 Hz (set in `sim/exts/oc.benchmark.ros_dds/usd/simple_g1_scenario.usda`); the
policy runs every 2nd physics step (50 Hz).

## API

```python
from oc.control.g1_walk import G1Walker, WalkPlan, WalkStatus

walker = G1Walker("/World/G1", on_done=lambda status: print(status))  # IMU at /World/G1/pelvis/imu_sensor
walker.start()  # steps on every Play until stop(); stands until it has a trajectory

walker.execute(WalkPlan().wait(2).move_forward(1.5).turn_left(90).step_right(0.5, speed=0.3).to_trajectory())
walker.execute(WalkPlan().walk_to(1.0, 0.0, 0.0).walk_to(1.0, 1.0, 1.57, speed=0.8).to_trajectory())  # world poses
walker.status  # WalkStatus.IDLE, RUNNING, DONE or FAILED
walker.stop()
```

`WalkPlan` is the generator: chain Walk primitives, then `to_trajectory()` returns the `WalkTrajectory`
(its Walk goals) that `execute` follows. `execute` replaces the current trajectory at any time.
`on_done` is called with `DONE` or `FAILED` when a trajectory ends.

| Primitive | Amount | `speed=` |
| --- | --- | --- |
| `move_forward`, `move_backward` | meters | m/s, default 0.5, max 1.0 |
| `step_left`, `step_right` | meters | m/s, default 0.5, max 1.0 |
| `turn_left`, `turn_right` | degrees | deg/s, default 45, max 57.3 (1 rad/s) |
| `wait` | seconds standing still | — |
| `stop` | none: ends the plan, so the robot stands after the last goal and it doesn't repeat | — |
| `walk_to` | world pose `x` [m], `y` [m], `yaw` [rad] | m/s, default 0.5, max 1.0 (turns at 45°/s) |

Moves, turns and waits are relative to where the previous primitive left the robot, and the
trajectory is measured from the robot's pose when it starts. Amounts must be positive and speeds
within the policy's trained range, or they raise `ValueError`. `walk_to` goals are in the world frame
and can't be mixed with relative primitives in one plan. Nothing can follow `stop()`. `WalkPlan(loop=True)`
repeats forever, unless stopped, always measured from the same start pose, so a looping plan should
end where it started.

`G1WalkController` (Walk trajectory + robot state → joint targets) is a MotionGen controller
(`isaacsim.robot_motion.experimental.motion_generation.BaseController`) and can be stepped directly:
`set_trajectory(trajectory)`, then `reset(state, None, t)` and `forward(state, None, t)` at 50 Hz.

## Window

Window > G1 Walk opens a window tabbed next to Property. Under Visualizer, Show Walk Trajectory
(off by default) draws each started `G1Walker`'s Walk trajectory on the ground: a line from the start
pose through each Walk goal (back to the first goal if it loops), with a tick for each goal's heading.
It appears once the robot starts walking after Play and clears on Stop.

## Tests

Extension Manager → G1 Walk → Tests tab runs `oc.control.g1_walk.tests` (Walk plans, Walk
trajectories and the control law, without a robot).

## Policy

`data/motion.pt` is Unitree's pretrained G1 walking policy from
[unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym) (`deploy/pre_train/g1/motion.pt`),
BSD 3-Clause, see `data/LICENSE-unitree_rl_gym`. It drives the 12 leg joints; the waist and arms
are held stiffly at zero because the policy was trained with a rigid upper body.
