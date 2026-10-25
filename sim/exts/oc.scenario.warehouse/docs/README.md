# Warehouse

Opens the warehouse Scenario (`usd/warehouse.usda`) and Spawns the G1 from `oc.robot.g1` at `/World/G1`.

Extension ID: `oc.scenario.warehouse`

## Settings

| Setting (`exts."oc.scenario.warehouse"`) | Default | Meaning |
| --- | --- | --- |
| `autostart` | `false` | Open the warehouse once the app is ready. Off, the extension leaves the stage alone. |
| `autoplay` | `false` | Play the simulation once the Scenario opens. Only acts with `autostart`; off leaves it stopped, so press Play. |
| `ros` | `false` | Publish the G1's ROS 2 topics and `/clock`. Off, `/World/G1/Graph` and `/World/Graph/ROS2_Clock` are deactivated, so the G1 renders no camera or lidar. |
| `obstacles` | `""` | Props to show, comma-separated: `clutter` (boxes, crates, cones, signs) and/or `forklift`; empty or `none` shows none. |

## Run

From the repository root:

```bash
pixi run warehouse                      # Extension workflow, Newton, with a window, plays once the Scenario opens
pixi run warehouse -s                   # Standalone workflow
pixi run warehouse -m                   # stay stopped, press Play
pixi run warehouse --headless           # no window
pixi run warehouse -p physx             # PhysX instead of Newton
pixi run warehouse --ros                # publish the G1's ROS 2 topics
pixi run warehouse -o clutter,forklift  # show the hidden props
```

`warehouse.usda` marks its clutter and forklift `active = false`. `-o/--obstacles` sets the `obstacles` setting;
`ObjectManager.show()` then activates the paths in `ObjectManager.OBSTACLES` in the session layer, so the `.usda` is
never changed.
Prims missing from the stage are logged and skipped.

The G1, the obstacles and the physics rate also go in the session layer. Physics runs at 200 Hz (SONIC's training
rate), set on both the PhysX and Newton scene attributes. `isaacsim.robot_setup.virtual_gantry` is disabled: it
rescans the stage every few physics steps and the warehouse has no gantries.

## API

Runtime use from the Script Editor:

```python
import isaacsim.core.experimental.utils.stage as stage_utils
from oc.scenario.warehouse import ObjectManager

objects = ObjectManager(stage_utils.get_current_stage())
objects.hide(["forklift"])  # edits go to the session layer
objects.show(["clutter"])
objects.visible()  # kinds whose prims are all active
ObjectManager.OBSTACLES  # {"clutter": (prim paths...), "forklift": (prim paths...)}
```

`show()` and `hide()` raise `ValueError` for an unknown kind. A Standalone script sets `autostart` (and `autoplay`,
`ros`, `obstacles`) through `extra_args` and pumps `SimulationApp.update()`, which runs the extension's setup.
