# ROS DDS Benchmark

Opens the DDS Benchmark's Scenario, `usd/simple_g1_scenario.usda`, and plays it. The Scenario's
G1 at `/World/G1` walks `SIMPLE_WALK_TRAJECTORY` (`simple_walk_trajectory.py`) on every Play, through
`oc.control.g1_walk`'s `G1Walker`. Its looping `WalkPlan` stands 2 s, then four times walks 2 m
forward at 0.5 m/s and turns left 90° at 45°/s, correcting its heading and position as it goes.

Extension ID: `oc.benchmark.ros_dds`

## Run

From the repository root:

```bash
pixi run -e sim benchmark --name dds                        # Extension workflow (default)
pixi run -e sim benchmark --name dds --workflow standalone  # Standalone workflow
```

Both take `--physics newton|physx`, `--dds fast|cyclone|zenoh` and `--headless`. The Extension
workflow launches Isaac Sim with this extension and `--/exts/oc.benchmark.ros_dds/autostart=true`,
so the Scenario opens and plays once the app is ready. The Standalone workflow runs
`scripts/benchmark/dds_benchmark.py`, which calls `open_scenario()` and owns the app loop.

Enabled by hand (autostart off), the extension leaves the stage alone and only shows the ROS DDS
Benchmark window, docked next to Property. Open the Scenario from the Script Editor with
`open_scenario()`.

## API

```python
import oc.benchmark.ros_dds

oc.benchmark.ros_dds.open_scenario()  # opens SCENARIO as the stage; raises RuntimeError if it can't
oc.benchmark.ros_dds.SIMPLE_WALK_TRAJECTORY  # a looping WalkTrajectory
```
