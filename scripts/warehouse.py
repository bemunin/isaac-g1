"""Open the warehouse Scenario (oc.scenario.warehouse) in Isaac Sim and play it, with ROS 2 Jazzy and an RMW
from the Pixi environment.

The Standalone workflow (default) runs in this script, re-executed through sim.py with the ROS 2 environment;
the Extension workflow launches Isaac Sim with the extension, which opens the Scenario and plays."""

import argparse
import os
import sys
from pathlib import Path

from dds import DDS_RMW

from sim import OBSTACLES, PHYSICS_EXPERIENCE, obstacles_argument

ROOT = Path(__file__).resolve().parent.parent
EXTENSION = "oc.scenario.warehouse"
SIM_SCRIPT = ROOT / "scripts" / "sim.py"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="pixi run warehouse", description=__doc__)
    parser.add_argument(
        "--workflow",
        choices=("standalone", "ext"),
        default="standalone",
        help="standalone or extension workflow (default: standalone)",
    )
    parser.add_argument(
        "--physics",
        choices=PHYSICS_EXPERIENCE,
        default="newton",
        help="physics engine (default: newton)",
    )
    parser.add_argument(
        "--dds", choices=DDS_RMW, help="ROS 2 middleware (default: the session's)"
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
        help=f"warehouse props to show, comma-separated: {', '.join(OBSTACLES)} (default: none)",
    )
    parser.add_argument("--headless", action="store_true", help="run without a window")
    # Internal: the Standalone workflow's re-execution through sim.py runs the app in this process.
    parser.add_argument("--app", action="store_true", help=argparse.SUPPRESS)
    return parser.parse_args()


def launch(args: argparse.Namespace, obstacles: str) -> int:
    """Run sim.py, which sets up ROS 2 and starts Isaac Sim or this script again with --app."""
    launcher = [
        sys.executable,
        str(SIM_SCRIPT),
        *(["--dds", args.dds] if args.dds else []),
    ]
    if args.workflow == "ext":
        command = [
            *launcher,
            *("--physics", args.physics),
            *("--ext-folder", str(ROOT / "sim" / "exts"), "--enable", EXTENSION),
            f"--/exts/{EXTENSION}/autostart=true",
            f"--/exts/{EXTENSION}/ros={str(args.ros).lower()}",
            *([f"--/exts/{EXTENSION}/obstacles={obstacles}"] if obstacles else []),
            *(["--no-window"] if args.headless else []),
        ]
    else:
        command = [
            *launcher,
            str(Path(__file__).resolve()),
            "--app",
            *("--physics", args.physics),
            *(["--ros"] if args.ros else []),
            *(["--obstacles", obstacles] if obstacles else []),
            *(["--headless"] if args.headless else []),
        ]
    try:
        os.execv(sys.executable, command)
    except OSError as exc:
        print(f"Unable to launch the warehouse: {exc}", file=sys.stderr)
        return 1

    return 0


def run_standalone(args: argparse.Namespace, obstacles: str) -> int:
    """Open the Scenario, whose G1 stands on Play, or is driven over ROS 2 with --ros (SONIC whole-body
    control), and play until the app closes."""
    from isaacsim import SimulationApp

    extensions = [
        *("--ext-folder", str(ROOT / "sim" / "exts"), "--enable", EXTENSION),
        f"--/exts/{EXTENSION}/ros={str(args.ros).lower()}",
        *([f"--/exts/{EXTENSION}/obstacles={obstacles}"] if obstacles else []),
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

    while simulation_app.is_running():
        simulation_app.update()

    # Stop first: closing while playing crashes in isaacsim.robot_setup.virtual_gantry's stop handler.
    omni.timeline.get_timeline_interface().stop()
    simulation_app.update()
    simulation_app.close()
    return 0


def main() -> int:
    args = parse_args()
    obstacles = ",".join(
        dict.fromkeys(args.obstacles)
    )  # de-duplicated, in the order given
    return run_standalone(args, obstacles) if args.app else launch(args, obstacles)


if __name__ == "__main__":
    sys.exit(main())
