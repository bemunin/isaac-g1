"""Launch Isaac Sim with ROS 2 Jazzy and an RMW from the Pixi environment."""

import os
import subprocess
import sys
from pathlib import Path

from dds import DDS_RMW

USAGE = (
    "Usage: pixi run sim [--dds fast|cyclone|zenoh] [--physics newton|physx] "
    "[--isaac-option ... | script.py [script-args ...]]"
)
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
    rmw = os.environ.get("RMW_IMPLEMENTATION", DDS_RMW["fast"])
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


def main() -> int:
    args = sys.argv[1:]
    options = {"--dds": DDS_RMW, "--physics": PHYSICS_EXPERIENCE}
    chosen = {}
    # Launcher options come first, in any order, as "--opt value" or "--opt=value".
    while args and (name := args[0].partition("=")[0]) in options:
        if args[0] == name:
            value, args = (args[1] if len(args) > 1 else None), args[2:]
        else:
            value, args = args[0].partition("=")[2], args[1:]
        if value not in options[name]:
            print(USAGE, file=sys.stderr)
            return 1
        chosen[name] = value
    dds = chosen.get("--dds")
    physics = chosen.get("--physics", "physx")

    if dds is None:
        dds = session_dds()
        if dds is None:
            print(f"Unsupported RMW_IMPLEMENTATION: {os.environ['RMW_IMPLEMENTATION']!r}", file=sys.stderr)
            print(USAGE, file=sys.stderr)
            return 1

    if args and not args[0].startswith("--") and not Path(args[0]).is_file():
        print(f"Unknown argument or file not found: {args[0]!r}", file=sys.stderr)
        print(USAGE, file=sys.stderr)
        return 1

    if args and not args[0].startswith("--") and "--physics" in chosen:
        print("--physics only applies when launching the Isaac Sim app, not a script", file=sys.stderr)
        return 1

    try:
        env = launch_environment(dds)
        if args and not args[0].startswith("--"):
            command = [sys.executable, *args]
        else:
            command = [sys.executable, "-c", "from isaacsim import main; main()", PHYSICS_EXPERIENCE[physics], *args]
        os.execvpe(sys.executable, command, env)
    except (OSError, RuntimeError) as exc:
        print(f"Unable to launch Isaac Sim: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
