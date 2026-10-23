# G1 Warehouse

Opens the G1 warehouse Scenario; its G1 is set up for SONIC on Play and driven over ROS 2 by the `oc_sonic` nodes.

Extension ID: `oc.scenario.warehouse`

## Run

From the repository root:

```bash
pixi run warehouse                   # Standalone workflow, Newton, SONIC (defaults), with a window
pixi run warehouse --workflow ext    # Extension workflow
pixi run warehouse --headless        # no window
pixi run warehouse --physics physx   # PhysX instead of Newton
pixi run warehouse --ros             # publish the G1's ROS 2 topics
pixi run warehouse -o clutter,forklift # show the hidden props (`-o none` or no flag: none)
```

The Scenario marks its clutter (boxes, crates, cones, signs) and forklift `active = false`. `-o/--obstacles`
sets `exts."oc.scenario.warehouse".obstacles` (comma-separated `clutter`, `forklift`); `open_scenario()` then
calls `apply_obstacles()`, which activates the paths in `OBSTACLES` in the session layer, so the `.usda` is
never changed. Prims missing from the stage are logged and skipped.

SONIC's weights must be downloaded first (`pixi run sonic-download`). The `oc_sonic` nodes (`ros2 launch oc_sonic sonic.launch.py`) read `/joint_states` and `/imu/data` and publish `/joint_commands`.

The G1's ROS 2 graphs are on by default (`exts."oc.scenario.warehouse".ros = true`); they are how the `oc_sonic`
nodes drive it. With the setting off, `open_scenario()` deactivates `/World/G1/Graph`, which holds its ROS 2 graphs
(camera, lidar, TF, IMU, joint states, odometry and the joint-command subscriber): the G1 then publishes nothing,
renders no camera or lidar, and nothing drives it. Note that `pixi run warehouse` still passes `ros=false` unless you give it `--ros`.
`/World/G1/Graph/ROS2_JointCommands`'s `robotPath` is overridden to `/World/G1/pelvis`.

## Behavior

On every Play, `G1Drive` (`g1_drive.py`) sets up the G1 at `/World/G1` on the first physics step: SONIC's joint gains,
armatures and effort limits, the standing pose as position and target, and zero velocities. `open_scenario()` sets
physics to `PHYSICS_RATE` (100 Hz; SONIC trained at 200 Hz, the policy still runs at 50 Hz). The `oc_sonic` nodes then drive the joints over ROS 2;
`/cmd_vel` and `ros2 service call /planner/set_mode` go to them, not to Isaac Sim.

## API

```python
import oc.scenario.warehouse

oc.scenario.warehouse.open_scenario()  # opens SCENARIO as the stage; raises RuntimeError if it can't
oc.scenario.warehouse.PHYSICS_RATE  # 100, the physics steps per second open_scenario() sets
oc.scenario.warehouse.ros()  # whether open_scenario() keeps the G1's ROS 2 graphs, from /exts/oc.scenario.warehouse/ros
```
