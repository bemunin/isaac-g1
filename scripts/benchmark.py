"""Run a benchmark with ROS 2 Jazzy and an RMW from the Pixi environment.

The Extension workflow (default) launches Isaac Sim with the benchmark's extension, which opens its Scenario
and plays; the Standalone workflow runs scripts/benchmark/<name>_benchmark.py."""

import argparse
import os
import sys
from pathlib import Path

from dds import DDS_RMW

from sim import PHYSICS_EXPERIENCE, launch_environment, session_dds

ROOT = Path(__file__).resolve().parent.parent
BENCHMARK_DIR = ROOT / "scripts" / "benchmark"
BENCHMARK_EXTENSIONS = {"dds": "oc.benchmark.ros_dds"}  # each also has BENCHMARK_DIR/<name>_benchmark.py


def main() -> int:
    parser = argparse.ArgumentParser(prog="pixi run benchmark", description=__doc__)
    parser.add_argument("--name", required=True, choices=BENCHMARK_EXTENSIONS, help="benchmark to run")
    parser.add_argument(
        "--workflow", choices=("ext", "standalone"), default="ext", help="extension or standalone workflow (default: ext)"
    )
    parser.add_argument("--physics", choices=PHYSICS_EXPERIENCE, default="physx", help="physics engine (default: physx)")
    parser.add_argument("--dds", choices=DDS_RMW, help="ROS 2 middleware (default: the session's)")
    parser.add_argument("--headless", action="store_true", help="run without a window")
    args = parser.parse_args()

    dds = args.dds or session_dds()
    if dds is None:
        parser.error(f"unsupported RMW_IMPLEMENTATION: {os.environ['RMW_IMPLEMENTATION']!r}; pass --dds")

    if args.workflow == "ext":
        extension = BENCHMARK_EXTENSIONS[args.name]
        command = [
            *(sys.executable, "-c", "from isaacsim import main; main()", PHYSICS_EXPERIENCE[args.physics]),
            *("--ext-folder", str(ROOT / "sim" / "exts"), "--enable", extension),
            f"--/exts/{extension}/autostart=true",
            *(["--no-window"] if args.headless else []),
        ]
    else:
        script = BENCHMARK_DIR / f"{args.name}_benchmark.py"
        command = [sys.executable, str(script), "--physics", args.physics, *(["--headless"] if args.headless else [])]
    try:
        os.execvpe(sys.executable, command, launch_environment(dds))
    except (OSError, RuntimeError) as exc:
        print(f"Unable to launch benchmark {args.name!r}: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
