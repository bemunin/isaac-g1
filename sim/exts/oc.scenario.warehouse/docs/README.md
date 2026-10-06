# G1 Warehouse

Opens the G1 warehouse Scenario; its G1 walks a loop through the open hall on Play.

Extension ID: `oc.scenario.warehouse`

## Run

From the repository root:

```bash
pixi run warehouse                   # Standalone workflow, Newton (defaults), with a window
pixi run warehouse --workflow ext    # Extension workflow
pixi run warehouse --headless        # no window
pixi run warehouse --physics physx   # PhysX instead of Newton
```

`--dds fast|cyclone|zenoh` picks the ROS 2 middleware (default: the session's). The Standalone
workflow runs `scripts/warehouse/warehouse_standalone.py`, which calls `open_scenario()` and owns the
app loop. The Extension workflow launches Isaac Sim with this extension and
`--/exts/oc.scenario.warehouse/autostart=true`. With autostart on, the Scenario, `usd/warehouse_g1_scene.usda`, opens and plays once the app is
ready. Enabled by hand (autostart off), the extension leaves the stage alone; open the Scenario from
the Script Editor with `open_scenario()`.

The Scenario turns off collision on six mirrored wall wires (`SM_WallWire_51`-`56`, scale -0.01):
Newton's MuJoCo solver rejects their negative scale as a zero-size shape and fails to start physics.

Don't enable it together with `oc.benchmark.ros_dds`: both start a walker on `/World/G1`.

## Walk trajectory

On every Play, the G1 at `/World/G1` walks `WAREHOUSE_WALK_TRAJECTORY`
(`warehouse_walk_trajectory.py`) through `oc.control.g1_walk`'s `G1Walker`. The G1 starts in aisle 4
(BOXES J-L) at (-8, 9.5), facing north (+y). It stands 2 s, then walks a rectangle loop: 17 m up
aisle 4, left 5 m across to aisle 3, 19.5 m down aisle 3, left 5 m back to aisle 4, and 2.5 m up to
the start. It keeps 1.75 m from the racks on each side and turns 1.5 m past their north ends and
1.75 m below their south ends. A lap takes about 108 s at 0.5 m/s and 45°/s, and the loop repeats forever. Each lap is measured
from the start pose, so the loop closes instead of drifting.

## API

```python
import oc.scenario.warehouse

oc.scenario.warehouse.open_scenario()  # opens SCENARIO as the stage; raises RuntimeError if it can't
oc.scenario.warehouse.WAREHOUSE_WALK_TRAJECTORY  # a looping WalkTrajectory
```
