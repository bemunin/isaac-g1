"""Open the warehouse Scenario (oc.scenario.warehouse) in Isaac Sim (and play it with --autoplay), with ROS 2 Jazzy
and an RMW from the Pixi environment.

The Extension workflow (default) launches Isaac Sim with the extension, which opens the Scenario and, with --autoplay,
plays it; -s/--standalone runs the Standalone workflow in this script, re-executed once in the ROS 2 environment."""

import argparse
import os
import sys
from pathlib import Path

from dds import DDS_RMW

from sim import PHYSICS_EXPERIENCE, ensure_ros_environment, exec_with_ros, isaac_sim_command

ROOT = Path(__file__).resolve().parent.parent
EXTENSION = "oc.scenario.warehouse"
OBSTACLES = ("clutter", "forklift")  # warehouse props `-o/--obstacles` can show (see oc.scenario.warehouse)


def obstacles_argument(value: str) -> list[str]:
    """argparse type for -o/--obstacles: a comma-separated list of OBSTACLES, or "none"."""
    kinds = [kind.strip() for kind in value.split(",") if kind.strip()]
    for kind in kinds:
        if kind not in (*OBSTACLES, "none"):
            raise argparse.ArgumentTypeError(f"invalid obstacle {kind!r} (choose from {', '.join(OBSTACLES)}, none)")
    return [kind for kind in kinds if kind != "none"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="pixi run warehouse", description=__doc__)
    parser.add_argument(
        "-s",
        "--standalone",
        action="store_true",
        help="run the Standalone workflow in this script (default: the Extension workflow)",
    )
    parser.add_argument("-p", "--physics", choices=PHYSICS_EXPERIENCE, default="newton", help="physics engine (default: newton)")
    parser.add_argument("--dds", choices=DDS_RMW, help="ROS 2 middleware (default: the session's)")
    parser.add_argument(
        "-r",
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
    parser.add_argument(
        "-a",
        "--autoplay",
        action="store_true",
        help="play the simulation once the Scenario opens (default: stopped, press Play)",
    )
    parser.add_argument("--headless", action="store_true", help="run without a window")
    args = parser.parse_args()
    if args.headless and not args.autoplay:
        parser.error("--headless needs -a/--autoplay: nothing can press Play without a window")
    return args


def launch_extension(args: argparse.Namespace, obstacles: str) -> int:
    """Start Isaac Sim with the extension in the ROS 2 environment."""
    command = isaac_sim_command(
        args.physics,
        *("--ext-folder", str(ROOT / "sim" / "exts"), "--enable", EXTENSION),
        f"--/exts/{EXTENSION}/autostart=true",
        f"--/exts/{EXTENSION}/autoplay={str(args.autoplay).lower()}",
        f"--/exts/{EXTENSION}/ros={str(args.ros).lower()}",
        *([f"--/exts/{EXTENSION}/obstacles={obstacles}"] if obstacles else []),
        *(["--no-window"] if args.headless else []),
    )
    try:
        exec_with_ros(command, args.dds)
    except (OSError, RuntimeError) as exc:
        print(f"Unable to launch the warehouse: {exc}", file=sys.stderr)
        return 1

    return 0


def run_standalone(args: argparse.Namespace, obstacles: str) -> int:
    """Open the Scenario through the extension's autostart, whose G1 stands on Play, or is driven over ROS 2 with
    --ros (SONIC whole-body control), and run until the app closes."""
    from isaacsim import SimulationApp

    extensions = [
        *("--ext-folder", str(ROOT / "sim" / "exts"), "--enable", EXTENSION),
        # Enabled at startup, before the extension's setup_scene activates the G1's ROS 2 graphs; Isaac Sim itself
        # enables the bridge only after startup.
        *("--enable", "isaacsim.ros2.bridge"),
        f"--/exts/{EXTENSION}/autostart=true",
        f"--/exts/{EXTENSION}/autoplay={str(args.autoplay).lower()}",
        f"--/exts/{EXTENSION}/ros={str(args.ros).lower()}",
        *([f"--/exts/{EXTENSION}/obstacles={obstacles}"] if obstacles else []),
    ]
    simulation_app = SimulationApp(
        {"headless": args.headless, "extra_args": extensions},
        experience=f"{os.environ['EXP_PATH']}/{PHYSICS_EXPERIENCE[args.physics]}.kit",
    )

    # Kit modules are importable only once the app runs.
    import omni.timeline

    while simulation_app.is_running():
        simulation_app.update()

    # Stop first: closing while playing crashes in isaacsim.robot_setup.virtual_gantry's stop handler.
    omni.timeline.get_timeline_interface().stop()
    simulation_app.update()
    simulation_app.close()
    return 0


def main() -> int:
    args = parse_args()
    obstacles = ",".join(dict.fromkeys(args.obstacles))  # de-duplicated, in the order given
    if not args.standalone:
        return launch_extension(args, obstacles)
    try:
        ensure_ros_environment(args.dds)  # re-runs this script in the ROS 2 environment and does not return
    except (OSError, RuntimeError) as exc:
        print(f"Unable to launch the warehouse: {exc}", file=sys.stderr)
        return 1
    return run_standalone(args, obstacles)


if __name__ == "__main__":
    sys.exit(main())
