"""Warehouse (Standalone workflow): open the warehouse Scenario (oc.scenario.warehouse), whose G1 walks with SONIC at
/cmd_vel, and play until the app closes.

With --benchmark SECONDS, record isaacsim.benchmark.services KPIs for SECONDS of wall-clock play, write them to
--benchmark-output and exit."""

import argparse
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
from sim import PHYSICS_EXPERIENCE, obstacles_argument

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--physics", choices=PHYSICS_EXPERIENCE, default="newton", help="physics engine (default: newton)")
parser.add_argument("--no-ros", action="store_true", help="don't publish the G1's ROS 2 topics or follow /cmd_vel")
parser.add_argument(
    "-o",
    "--obstacles",
    type=obstacles_argument,
    action="extend",
    default=[],
    metavar="LIST",
    help="warehouse props to show, comma-separated: clutter, forklift (default: none)",
)
parser.add_argument("--headless", action="store_true", help="run without a window")
parser.add_argument("--benchmark", type=float, metavar="SECONDS", help="record KPIs for SECONDS of play, then exit")
parser.add_argument("--benchmark-output", type=Path, default=ROOT / "research" / "benchmark_output", metavar="DIR")
args = parser.parse_args()

RECORDERS = [
    *("app_frametime", "physics_frametime", "runtime", "memory", "cpu_continuous", "hardware"),
    *("gpu_frametime", "render_frametime", "rtf_stability", "physics_step_interval"),
]

from isaacsim import SimulationApp

extensions = [
    *("--ext-folder", str(ROOT / "sim" / "exts"), "--enable", "oc.scenario.warehouse"),
    f"--/exts/oc.scenario.warehouse/ros={str(not args.no_ros).lower()}",
]
if args.obstacles:
    extensions.append(f"--/exts/oc.scenario.warehouse/obstacles={','.join(dict.fromkeys(args.obstacles))}")
if args.benchmark is not None:
    args.benchmark_output.mkdir(parents=True, exist_ok=True)
    extensions += [
        *("--enable", "isaacsim.benchmark.services"),
        f"--/exts/isaacsim.benchmark.services/metrics/metrics_output_folder={args.benchmark_output}",
    ]
simulation_app = SimulationApp(
    {"headless": args.headless, "extra_args": extensions},
    experience=f"{os.environ['EXP_PATH']}/{PHYSICS_EXPERIENCE[args.physics]}.kit",
)

# Kit modules are importable only once the app runs.
import isaacsim.core.experimental.utils.app as app_utils
import oc.scenario.warehouse
import omni.timeline

# Isaac Sim enables the ROS 2 bridge only after startup; the Scenario's graphs need its nodes now.
app_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

try:
    oc.scenario.warehouse.open_scenario()
except RuntimeError as error:
    simulation_app.close()
    sys.exit(str(error))
omni.timeline.get_timeline_interface().play()

if args.benchmark is None:
    while simulation_app.is_running():
        simulation_app.update()
else:
    from isaacsim.benchmark.services import BaseIsaacBenchmark

    benchmark = BaseIsaacBenchmark(
        benchmark_name=f"warehouse_{args.physics}{'' if args.no_ros else '_ros'}{'_headless' if args.headless else ''}",
        workflow_metadata={
            "metadata": [
                {"name": "physics", "data": args.physics},
                {"name": "ros", "data": str(not args.no_ros)},
                {"name": "headless", "data": str(args.headless)},
                {"name": "duration_s", "data": args.benchmark},
            ]
        },
        recorders=RECORDERS,
    )
    benchmark.run_warmup(60)  # let the renderer, physics and policy settle
    benchmark.set_phase("benchmark")
    end = time.monotonic() + args.benchmark
    while simulation_app.is_running() and time.monotonic() < end:
        simulation_app.update()
    benchmark.store_measurements()
    benchmark.stop()

# Stop first: closing while playing crashes in isaacsim.robot_setup.virtual_gantry's stop handler.
omni.timeline.get_timeline_interface().stop()
simulation_app.update()
simulation_app.close()
