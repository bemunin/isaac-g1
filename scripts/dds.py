"""Show or set the DDS selected for new Pixi sessions in this checkout."""

import os
import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
SELECTION = PROJECT_ROOT / ".pixi" / "dds-selection"
DDS_RMW = {
    "fast": "rmw_fastrtps_cpp",
    "cyclone": "rmw_cyclonedds_cpp",
    "zenoh": "rmw_zenoh_cpp",
}
DEFAULT_DDS = "fast"  # also the default in scripts/dds/activate.sh and activate.bat
USAGE = "Usage: pixi run dds [fast|cyclone|zenoh]"


def _daemon(action: str, detach: bool = False) -> str | None:
    """Run `ros2 daemon <action>`, optionally detached; return an error message, or None on success."""
    try:
        result = subprocess.run(
            ["pixi", "run", "ros2", "daemon", action],
            cwd=PROJECT_ROOT,
            capture_output=True,
            text=True,
            timeout=20,
            check=False,
            # Keep the daemon outside the Pixi task's process group after this command exits.
            start_new_session=detach and os.name != "nt",
            creationflags=subprocess.CREATE_NEW_PROCESS_GROUP if detach and os.name == "nt" else 0,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return str(exc)
    return result.stderr.strip() if result.returncode != 0 else None


def show() -> int:
    """Print the DDS selected for new Pixi sessions."""
    choice = SELECTION.read_text(encoding="utf-8").strip() if SELECTION.exists() else DEFAULT_DDS
    if choice not in DDS_RMW:
        print(f"Invalid DDS selection in {SELECTION}: {choice!r}", file=sys.stderr)
        return 1
    print(choice)
    return 0


def select(choice: str) -> int:
    """Restart the ROS 2 CLI daemon and save `choice` for new Pixi sessions."""
    if error := _daemon("stop"):
        print(f"Could not stop the ROS 2 CLI daemon before changing DDS: {error}", file=sys.stderr)
        return 1

    SELECTION.parent.mkdir(parents=True, exist_ok=True)
    SELECTION.write_text(choice + "\n", encoding="utf-8")

    if error := _daemon("start", detach=True):
        print(f"Selected {choice}, but could not start the ROS 2 CLI daemon: {error}", file=sys.stderr)
        return 1

    print(f"Selected {choice} for new Pixi sessions")
    return 0


def main() -> int:
    match sys.argv[1:]:
        case []:
            return show()
        case [choice] if choice in DDS_RMW:
            return select(choice)
        case _:
            print(USAGE, file=sys.stderr)
            return 1


if __name__ == "__main__":
    sys.exit(main())
