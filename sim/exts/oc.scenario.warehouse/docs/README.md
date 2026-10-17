# G1 Warehouse

Opens the G1 warehouse Scenario; on Play its G1 walks with SONIC whole-body control at `/cmd_vel`, e.g. from
Nav2 or teleop.

Extension ID: `oc.scenario.warehouse`

## Run

From the repository root:

```bash
pixi run warehouse                   # Standalone workflow, Newton, with a window
pixi run warehouse --workflow ext    # Extension workflow
pixi run warehouse --headless        # no window
pixi run warehouse --physics physx   # PhysX instead of Newton
pixi run warehouse --ros             # publish the G1's ROS 2 topics and follow /cmd_vel (off: the G1 stands)
pixi run warehouse -o clutter,forklift # show the hidden props (`-o none` or no flag: none)
```

The Scenario marks its clutter (boxes, crates, cones, signs) and forklift `active = false`. `-o/--obstacles`
sets `exts."oc.scenario.warehouse".obstacles` (comma-separated `clutter`, `forklift`); `open_scenario()` then
calls `apply_obstacles()`, which activates the paths in `OBSTACLES` in the session layer, so the `.usda` is
never changed. Prims missing from the stage are logged and skipped.

SONIC's weights must be downloaded first (`pixi run sonic-download`). With `--ros`, drive the G1 from a second terminal:

```bash
pixi run ros2 run teleop_twist_keyboard teleop_twist_keyboard                     # by hand
pixi run ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"  # straight ahead
```

The G1's ROS 2 topics are off by default. `/World/G1/Graph` holds its eight ROS 2 graphs (camera, lidar,
TF, IMU, joint states, odometry, the joint-command subscriber and `ROS2_CmdVel`, the `/cmd_vel`
subscriber). Unless `exts."oc.scenario.warehouse".ros = true` (`--ros`), `open_scenario()` deactivates the
graphs, so the G1 publishes nothing, doesn't render its camera or lidar and gets no commands; it stands, and
the sim runs faster.

`--dds fast|cyclone|zenoh` picks the ROS 2 middleware (default: the session's). The Standalone
workflow runs `scripts/warehouse/warehouse_standalone.py`, which calls `open_scenario()` and owns the
app loop. The Extension workflow launches Isaac Sim with this extension and
`--/exts/oc.scenario.warehouse/autostart=true`. With autostart on, the Scenario, `usd/warehouse_g1_scene.usda`, opens and plays once the app is
ready. Enabled by hand (autostart off), the extension leaves the stage alone; open the Scenario from
the Script Editor with `open_scenario()`.

The Scenario turns off collision on six mirrored wall wires (`SM_WallWire_51`-`56`, scale -0.01):
Newton's MuJoCo solver rejects their negative scale as a zero-size shape and fails to start physics.

## Motion

On every Play, the G1 at `/World/G1` starts standing. `oc.control.g1_sonic`'s `G1Sonic` moves it in SONIC's
walk mode, and `open_scenario()` sets physics to 200 Hz, the rate SONIC was trained at. With ROS on, the
extension's `CmdVelBridge` (`cmd_vel.py`) reads `ROS2_CmdVel` each app update and forwards each new `/cmd_vel`
message (vx, vy, wz) to `G1Sonic.command()`; the G1 stands again 0.5 s of simulation time after the last one.
See `oc.control.g1_sonic`'s README for the speed limits Nav2 should respect.

The G1 starts in aisle 4 (BOXES J-L) at (-8, 9.5), facing north (+y). Aisle 4 is clear for x -9.75..-6.25 and
aisle 3 (BOXES G-I) for x -14.75..-11.25; their racks span y 8.75..25.0, with open floor past both ends.

## API

```python
import oc.scenario.warehouse

oc.scenario.warehouse.open_scenario()  # opens SCENARIO as the stage; raises RuntimeError if it can't
oc.scenario.warehouse.ros()  # whether open_scenario() keeps the G1's ROS 2 graphs, from /exts/oc.scenario.warehouse/ros
oc.scenario.warehouse.CmdVelBridge(command)  # calls command((vx, vy, wz)) for each new /cmd_vel from ROS2_CmdVel
```
