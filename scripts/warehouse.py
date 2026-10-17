"""Open the warehouse Scenario (oc.scenario.warehouse) in Isaac Sim and play it, with ROS 2 Jazzy and an RMW
from the Pixi environment.

The Standalone workflow (default) runs scripts/warehouse/warehouse_standalone.py; the Extension workflow
launches Isaac Sim with the extension, which opens the Scenario and plays."""

import argparse
import os
import sys
from pathlib import Path

from dds import DDS_RMW

from sim import PHYSICS_EXPERIENCE, launch_environment, obstacles_argument, session_dds

ROOT = Path(__file__).resolve().parent.parent
EXTENSION = "oc.scenario.warehouse"
STANDALONE_SCRIPT = ROOT / "scripts" / "warehouse" / "warehouse_standalone.py"


def main() -> int:
    parser = argparse.ArgumentParser(prog="pixi run warehouse", description=__doc__)
    parser.add_argument(
        "--workflow",
        choices=("standalone", "ext"),
        default="standalone",
        help="standalone or extension workflow (default: standalone)",
    )
    parser.add_argument("--physics", choices=PHYSICS_EXPERIENCE, default="newton", help="physics engine (default: newton)")
    parser.add_argument("--dds", choices=DDS_RMW, help="ROS 2 middleware (default: the session's)")
    parser.add_argument(
        "--controller",
        choices=("walk", "sonic"),
        default="sonic",
        help="G1 controller: walk (Unitree walking policy) or sonic (SONIC whole-body control) (default: sonic)",
    )
    parser.add_argument(
        "--ros",
        action="store_true",
        help="publish the G1's ROS 2 topics (camera, lidar, TF, IMU, joint states, odometry); off by default",
    )
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
    parser.add_argument(
        "--benchmark",
        type=float,
        metavar="SECONDS",
        help="record isaacsim.benchmark.services KPIs for SECONDS of wall-clock play, then exit (standalone only)",
    )
    parser.add_argument(
        "--benchmark-output",
        type=Path,
        default=ROOT / "research" / "benchmark_output",
        metavar="DIR",
        help="folder for the benchmark's KPI file (default: research/benchmark_output)",
    )
    args = parser.parse_args()
    if args.benchmark is not None and args.workflow == "ext":
        parser.error("--benchmark needs --workflow standalone")

    obstacles = ",".join(dict.fromkeys(args.obstacles))  # de-duplicated, in the order given

    dds = args.dds or session_dds()
    if dds is None:
        parser.error(f"unsupported RMW_IMPLEMENTATION: {os.environ['RMW_IMPLEMENTATION']!r}; pass --dds")

    if args.workflow == "ext":
        command = [
            *(sys.executable, "-c", "from isaacsim import main; main()", PHYSICS_EXPERIENCE[args.physics]),
            *("--ext-folder", str(ROOT / "sim" / "exts"), "--enable", EXTENSION),
            f"--/exts/{EXTENSION}/autostart=true",
            f"--/exts/{EXTENSION}/controller={args.controller}",
            f"--/exts/{EXTENSION}/ros={str(args.ros).lower()}",
            *([f"--/exts/{EXTENSION}/obstacles={obstacles}"] if obstacles else []),
            *(["--no-window"] if args.headless else []),
        ]
    else:
        command = [
            *(sys.executable, str(STANDALONE_SCRIPT), "--physics", args.physics, "--controller", args.controller),
            *(["--ros"] if args.ros else []),
            *(["--obstacles", obstacles] if obstacles else []),
            *(["--headless"] if args.headless else []),
            *(
                ["--benchmark", str(args.benchmark), "--benchmark-output", str(args.benchmark_output.resolve())]
                if args.benchmark is not None
                else []
            ),
        ]
    try:
        os.execvpe(sys.executable, command, launch_environment(dds))
    except (OSError, RuntimeError) as exc:
        print(f"Unable to launch the warehouse: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
