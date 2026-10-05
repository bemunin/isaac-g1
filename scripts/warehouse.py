"""Open the warehouse Scenario (oc.scenario.warehouse) in Isaac Sim and play it, with ROS 2 Jazzy and an RMW
from the Pixi environment.

The Standalone workflow (default) runs scripts/warehouse/warehouse_standalone.py; the Extension workflow
launches Isaac Sim with the extension, which opens the Scenario and plays."""

import argparse
import os
import sys
from pathlib import Path

from dds import DDS_RMW

from sim import PHYSICS_EXPERIENCE, launch_environment, session_dds

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
    parser.add_argument("--headless", action="store_true", help="run without a window")
    args = parser.parse_args()

    dds = args.dds or session_dds()
    if dds is None:
        parser.error(f"unsupported RMW_IMPLEMENTATION: {os.environ['RMW_IMPLEMENTATION']!r}; pass --dds")

    if args.workflow == "ext":
        command = [
            *(sys.executable, "-c", "from isaacsim import main; main()", PHYSICS_EXPERIENCE[args.physics]),
            *("--ext-folder", str(ROOT / "sim" / "exts"), "--enable", EXTENSION),
            f"--/exts/{EXTENSION}/autostart=true",
            *(["--no-window"] if args.headless else []),
        ]
    else:
        command = [
            *(sys.executable, str(STANDALONE_SCRIPT), "--physics", args.physics),
            *(["--headless"] if args.headless else []),
        ]
    try:
        os.execvpe(sys.executable, command, launch_environment(dds))
    except (OSError, RuntimeError) as exc:
        print(f"Unable to launch the warehouse: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
