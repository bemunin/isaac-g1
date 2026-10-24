"""Open the Isaac Sim app with ROS 2 Jazzy and an RMW from the Pixi environment.

Other arguments go to Isaac Sim unchanged, e.g. --no-window, --/app/...=..., --ext-folder DIR --enable EXT."""

import argparse
import os
import subprocess
import sys
from pathlib import Path

from dds import DDS_RMW, DEFAULT_DDS

RMW_LIBRARIES = {f"lib{rmw}.so" for rmw in DDS_RMW.values()}
PHYSICS_EXPERIENCE = {
    "newton": "isaacsim.exp.full.newton",  # Newton entry point; disables PhysX
    "physx": "isaacsim.exp.full",
}


def _keep(path: str) -> bool:
    """Drop system ROS and Isaac Sim's bundled ROS from search paths."""
    return bool(path) and not path.startswith("/opt/ros/") and "/isaacsim/exts/isaacsim.ros2.core/" not in path


def session_dds() -> str | None:
    """The DDS selected for this session (RMW_IMPLEMENTATION, default Fast DDS), or None if unsupported."""
    rmw = os.environ.get("RMW_IMPLEMENTATION", DDS_RMW[DEFAULT_DDS])
    return next((name for name, implementation in DDS_RMW.items() if implementation == rmw), None)


def launch_environment(dds: str) -> dict[str, str]:
    env = os.environ.copy()
    if not env.get("CONDA_PREFIX"):
        raise RuntimeError("Run this launcher with pixi run so CONDA_PREFIX is set")

    ros_prefix = Path(__file__).resolve().parent.parent / ".pixi" / "envs" / "default"
    if not ros_prefix.is_dir():
        raise RuntimeError("Pixi's ROS environment is missing; run pixi install")

    lib_dir = ros_prefix / "lib"
    rmw_library = lib_dir / f"lib{DDS_RMW[dds]}.so"
    if not rmw_library.is_file():
        raise RuntimeError(f"External ROS 2 RMW library was not found: {rmw_library}")

    setup = ros_prefix / "setup.bash"
    if not setup.is_file():
        raise RuntimeError(f"External ROS 2 setup script was not found: {setup}")

    env["ROS_DISTRO"] = "jazzy"
    env["RMW_IMPLEMENTATION"] = DDS_RMW[dds]
    try:
        result = subprocess.run(
            ["bash", "-c", 'set -e; source "$1" 1>&2; env -0', "bash", str(setup)],
            env=env,
            stdout=subprocess.PIPE,
            check=True,
        )
    except (OSError, subprocess.CalledProcessError) as exc:
        raise RuntimeError(f"Could not activate Pixi's ROS environment: {exc}") from exc
    env = dict(os.fsdecode(entry).split("=", 1) for entry in result.stdout.split(b"\0") if entry)

    # Load ROS from the dedicated Pixi environment, not Isaac Sim's bundle.
    library_paths = [path for path in env.get("LD_LIBRARY_PATH", "").split(os.pathsep) if path != str(lib_dir) and _keep(path)]
    env["LD_LIBRARY_PATH"] = os.pathsep.join([str(lib_dir), *library_paths])
    env["PYTHONPATH"] = os.pathsep.join(filter(_keep, env.get("PYTHONPATH", "").split(os.pathsep)))
    ament_paths = [
        path
        for path in env.get("AMENT_PREFIX_PATH", "").split(os.pathsep)
        if path and path != str(ros_prefix) and not path.startswith("/opt/ros/")
    ]
    env["AMENT_PREFIX_PATH"] = os.pathsep.join([str(ros_prefix), *ament_paths])
    env["ROS_VERSION"] = "2"

    preload = [path for path in env.get("LD_PRELOAD", "").split() if Path(path).name not in RMW_LIBRARIES]
    if dds == "zenoh":
        preload.insert(0, str(rmw_library))
    if preload:
        env["LD_PRELOAD"] = " ".join(preload)
    else:
        env.pop("LD_PRELOAD", None)
    return env


def isaac_sim_command(physics: str, *isaac_args: str) -> list[str]:
    """The command that opens the Isaac Sim app with the `physics` experience."""
    return [sys.executable, "-c", "from isaacsim import main; main()", PHYSICS_EXPERIENCE[physics], *isaac_args]


def exec_with_ros(command: list[str], dds: str | None = None) -> None:
    """Replace this process with `command` in the ROS 2 environment of `dds` (default: the session's).

    Raises OSError or RuntimeError if the environment or the command cannot be set up."""
    dds = dds or session_dds()
    if dds is None:
        raise RuntimeError(f"unsupported RMW_IMPLEMENTATION: {os.environ['RMW_IMPLEMENTATION']!r}; pass --dds")
    os.execvpe(command[0], command, launch_environment(dds))


def main() -> int:
    parser = argparse.ArgumentParser(prog="pixi run sim", description=__doc__, allow_abbrev=False)
    parser.add_argument("--dds", choices=DDS_RMW, help="ROS 2 middleware (default: the session's)")
    parser.add_argument("--physics", choices=PHYSICS_EXPERIENCE, default="physx", help="physics engine (default: physx)")
    args, isaac_args = parser.parse_known_args()
    try:
        exec_with_ros(isaac_sim_command(args.physics, *isaac_args), args.dds)
    except (OSError, RuntimeError) as exc:
        print(f"Unable to launch Isaac Sim: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
