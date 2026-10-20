# G1 Warehouse

Opens the G1 warehouse Scenario; its G1 walks a loop through the open hall on Play.

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

SONIC's weights must be downloaded first (`pixi run sonic-download`).

The G1's ROS 2 topics are off by default. `open_scenario()` deactivates `/World/G1/Graph`, which holds
its eight ROS 2 graphs (camera, lidar, TF, IMU, joint states, odometry, and the joint-command and
`/cmd_vel` subscribers). The G1 then publishes nothing and doesn't render its camera or lidar, so the sim runs
faster. The controller reads the pelvis IMU directly and is unaffected. `--ros` sets
`exts."oc.scenario.warehouse".ros = true` to keep the graphs, e.g. for Foxglove.

With `--ros` the G1 also subscribes to `/cmd_vel` (`geometry_msgs/Twist`, graph `ROS2_CmdVel`). The extension
polls its counter on every app update and sends each Twist to SONIC as a Locomotion command, which takes over
from the loop (Real-time control) until the next Play; when `/cmd_vel` goes quiet, the G1 stands.

`--dds fast|cyclone|zenoh` picks the ROS 2 middleware (default: the session's). The Standalone
workflow runs `scripts/warehouse/warehouse_standalone.py`, which calls `open_scenario()` and owns the
app loop. The Extension workflow launches Isaac Sim with this extension and
`--/exts/oc.scenario.warehouse/autostart=true`. With autostart on, the Scenario, `usd/warehouse_g1_scene.usda`, opens and plays once the app is
ready. Enabled by hand (autostart off), the extension leaves the stage alone; open the Scenario from
the Script Editor with `open_scenario()`.

The Scenario turns off collision on six mirrored wall wires (`SM_WallWire_51`-`56`, scale -0.01):
Newton's MuJoCo solver rejects their negative scale as a zero-size shape and fails to start physics.

## Behavior sequence

On every Play, the G1
at `/World/G1` follows `WAREHOUSE_SEQUENCE` (`warehouse_sequence.py`) through
`oc.control.g1_sonic`'s `G1Wbc`, and `open_scenario()` sets physics to `PHYSICS_RATE` (200 Hz), the rate SONIC was
trained at. The loop:
1. Stand 2 s at (-8, 9.5), facing north.
2. Walk 17 m up aisle 4 at 1 m/s, slowing down before the spin.
3. Turn left and walk 5 m across to aisle 3.
4. Turn left and walk 19.5 m down aisle 3.
5. Turn left and walk 3 m east.
6. Turn left to face north, side-step 2 m right into aisle 4, then walk 2.5 m back to the start.

It repeats forever.

## API

```python
import oc.scenario.warehouse

oc.scenario.warehouse.open_scenario()  # opens SCENARIO as the stage; raises RuntimeError if it can't
oc.scenario.warehouse.WAREHOUSE_SEQUENCE  # a looping BehaviorSequence
oc.scenario.warehouse.PHYSICS_RATE  # 200, the physics steps per second open_scenario() sets
oc.scenario.warehouse.ros()  # whether open_scenario() keeps the G1's ROS 2 graphs, from /exts/oc.scenario.warehouse/ros
```
