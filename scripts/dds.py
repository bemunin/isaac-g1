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
USAGE = "Usage: pixi run dds [fast|cyclone|zenoh]"


def _daemon(action: str, **kwargs) -> str | None:
    """Run `ros2 daemon <action>`; return an error message, or None on success."""
    try:
        result = subprocess.run(
            ["pixi", "run", "ros2", "daemon", action],
            cwd=PROJECT_ROOT,
            capture_output=True,
            text=True,
            timeout=20,
            check=False,
            **kwargs,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return str(exc)
    return result.stderr.strip() if result.returncode != 0 else None


def main() -> int:
    if len(sys.argv) > 2 or (len(sys.argv) == 2 and sys.argv[1] not in DDS_RMW):
        print(USAGE, file=sys.stderr)
        return 1

    if len(sys.argv) == 1:
        choice = SELECTION.read_text(encoding="utf-8").strip() if SELECTION.exists() else "fast"
        if choice not in DDS_RMW:
            print(f"Invalid DDS selection in {SELECTION}: {choice!r}", file=sys.stderr)
            return 1
        print(choice)
        return 0

    choice = sys.argv[1]
    if error := _daemon("stop"):
        print(f"Could not stop the ROS 2 CLI daemon before changing DDS: {error}", file=sys.stderr)
        return 1

    SELECTION.parent.mkdir(parents=True, exist_ok=True)
    SELECTION.write_text(choice + "\n", encoding="utf-8")

    # Keep the daemon outside the Pixi task's process group after this command exits.
    if error := _daemon(
        "start",
        start_new_session=os.name != "nt",
        creationflags=subprocess.CREATE_NEW_PROCESS_GROUP if os.name == "nt" else 0,
    ):
        print(f"Selected {choice}, but could not start the ROS 2 CLI daemon: {error}", file=sys.stderr)
        return 1

    print(f"Selected {choice} for new Pixi sessions")
    return 0


if __name__ == "__main__":
    sys.exit(main())
