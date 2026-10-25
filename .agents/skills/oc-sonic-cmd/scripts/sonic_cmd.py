"""Send a SONIC Locomotion command to the G1 in a running warehouse, over ROS 2 or Python.

Exit codes: 0 ok, 1 command failed, 2 sim not ready (or bad arguments).
"""

import argparse
import asyncio
import json
import sys
import time

# Planner mode name -> id, for all 27 modes (SonicMode lowercased; the table in SKILL.md).
_MODES = {
    "idle": 0,
    "slow_walk": 1,
    "walk": 2,
    "run": 3,
    "squat": 4,
    "kneel_two_legs": 5,
    "kneel_one_leg": 6,
    "lying_face_down": 7,
    "hand_crawl": 8,
    "boxing_idle": 9,
    "boxing_walk": 10,
    "boxing_left_jab": 11,
    "boxing_right_jab": 12,
    "boxing_random_punches": 13,
    "elbow_crawl": 14,
    "boxing_left_hook": 15,
    "boxing_right_hook": 16,
    "happy_walk": 17,
    "stealth_walk": 18,
    "injured_walk": 19,
    "careful_walk": 20,
    "object_carrying": 21,
    "crouch_walk": 22,
    "happy_dance_walk": 23,
    "zombie_walk": 24,
    "point_walk": 25,
    "scared_walk": 26,
}

_ROS_HINT = "start the warehouse with ROS on: pixi run warehouse -a -r (check that pixi run dds matches)"
_PYTHON_HINT = "start the warehouse with: pixi run warehouse -a --vscode"
_WAIT_S = 5.0  # wall seconds to wait for the sim (subscribers, /clock) and to see sim time advance


def _mode(text: str) -> int:
    """Parses a mode name or an id 0-26."""
    if text in _MODES:
        return _MODES[text]
    if text.isdigit() and int(text) in _MODES.values():
        return int(text)
    raise argparse.ArgumentTypeError(f"{text!r} is not a mode: use a name ({', '.join(_MODES)}) or an id 0-26")


def _parse() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send a SONIC Locomotion command to the G1 in a running warehouse. The command holds until "
        "replaced, unless --duration is given.",
        epilog="exit codes: 0 ok, 1 command failed, 2 sim not ready or bad arguments",
    )
    parser.add_argument("--source", required=True, choices=("ros", "python"), help="ros: topics (warehouse -a -r); "
                        "python: python_server on --host/--port (warehouse -a --vscode)")
    parser.add_argument("--mode", type=_mode, default=0, metavar="NAME|ID", help="mode name or id 0-26 (default: idle)")
    parser.add_argument("--vx", type=float, default=0.0, help="forward speed [m/s] (default: 0)")
    parser.add_argument("--vy", type=float, default=0.0, help="left speed [m/s] (default: 0)")
    parser.add_argument("--wz", type=float, default=0.0, help="turn rate, left positive [rad/s] (default: 0)")
    parser.add_argument("--pelvis-height", type=float, default=-1.0, metavar="M",
                        help="target pelvis height [m]; -1 lets the mode decide (default: -1)")
    parser.add_argument("--duration", type=float, metavar="S",
                        help="hold the command S seconds of sim time, then send idle with zero velocity")
    parser.add_argument("--stop", action="store_true", help="idle with zero velocity (overrides the command flags)")
    parser.add_argument("--host", default="127.0.0.1", help="python_server host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8226, help="python_server port (default: 8226)")
    args = parser.parse_args()
    if args.duration is not None and args.duration <= 0:
        parser.error("--duration must be positive")
    if args.stop:
        args.mode, args.vx, args.vy, args.wz, args.pelvis_height, args.duration = 0, 0.0, 0.0, 0.0, -1.0, None
    return args


def _snippet(args: argparse.Namespace) -> str:
    """The Python that runs inside Isaac Sim."""
    lines = [
        "from oc.scenario.warehouse import get_sonic_controller",
        "sonic = get_sonic_controller()",
        'if sonic is None:\n    raise RuntimeError("warehouse SONIC is off")',
        f"sonic.command({args.mode}, {args.vx}, {args.vy}, {args.wz}, {args.pelvis_height})",
    ]
    if args.duration is not None:
        lines += [
            "import time",
            "import omni.kit.app",
            "from isaacsim.core.simulation_manager import SimulationManager",
            "app = omni.kit.app.get_app()",
            "start = last = SimulationManager.get_simulation_time()",
            "wall = time.monotonic()",
            "stalled = False",
            "try:",
            f"    while last - start < {args.duration}:",
            "        await app.next_update_async()",
            "        now = SimulationManager.get_simulation_time()",
            "        if now > last:",
            "            last, wall = now, time.monotonic()",
            f"        elif time.monotonic() - wall > {_WAIT_S}:",
            "            stalled = True",
            "            break",
            "finally:",
            "    sonic.command(0)",
            "if stalled:",
            '    raise RuntimeError("sim time stopped advancing (paused?); sent idle")',
            'print(f"held {last - start:.2f} s of sim time, then sent idle")',
        ]
    return "\n".join(lines)


async def _send(code: str, args: argparse.Namespace) -> dict:
    """Runs the code in python_server and returns its JSON response."""
    timeout = max(30.0, 10 * (args.duration or 0))
    reader, writer = await asyncio.open_connection(args.host, args.port)
    try:
        writer.write(json.dumps({"code": code, "context": "oc-sonic-cmd", "timeout": timeout}).encode())
        writer.write_eof()
        return json.loads(await asyncio.wait_for(reader.read(), timeout + 5))
    finally:
        writer.close()


def _run_python(args: argparse.Namespace) -> int:
    code = _snippet(args)
    print(f"# the same in VS Code (Isaac Sim: Run):\n{code}\n")
    try:
        response = asyncio.run(_send(code, args))
    except (ConnectionRefusedError, OSError) as error:
        print(f"cannot reach python_server at {args.host}:{args.port} ({error}); {_PYTHON_HINT}", file=sys.stderr)
        return 2
    if response.get("output"):
        print(response["output"].rstrip())
    if response.get("status") != "ok":
        print(f"{response.get('ename')}: {response.get('evalue')}", file=sys.stderr)
        return 1
    return 0


def _run_ros(args: argparse.Namespace) -> int:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import Float32, Int32

    rclpy.init()
    node = rclpy.create_node("sonic_cmd")
    try:
        mode_pub = node.create_publisher(Int32, "/sonic/mode", 10)
        height_pub = node.create_publisher(Float32, "/sonic/pelvis_height", 10)
        twist_pub = node.create_publisher(Twist, "/cmd_vel", 10)
        clock = {"t": None}  # latest /clock stamp [s]

        def on_clock(msg: Clock) -> None:
            clock["t"] = msg.clock.sec + msg.clock.nanosec * 1e-9

        node.create_subscription(Clock, "/clock", on_clock, qos_profile_sensor_data)

        def spin_until(done, seconds: float) -> bool:
            deadline = time.monotonic() + seconds
            while not done():
                if time.monotonic() > deadline:
                    return False
                rclpy.spin_once(node, timeout_sec=0.05)
            return True

        pubs = (mode_pub, height_pub, twist_pub)
        if not spin_until(lambda: clock["t"] is not None and all(p.get_subscription_count() for p in pubs), _WAIT_S):
            print(f"no /clock or no subscriber on /sonic/mode, /sonic/pelvis_height, /cmd_vel; {_ROS_HINT}", file=sys.stderr)
            return 2

        def send(mode: int, vx: float, vy: float, wz: float, pelvis_height: float) -> None:
            mode_pub.publish(Int32(data=mode))
            height_pub.publish(Float32(data=pelvis_height))
            twist = Twist()
            twist.linear.x, twist.linear.y, twist.angular.z = vx, vy, wz
            twist_pub.publish(twist)

        send(args.mode, args.vx, args.vy, args.wz, args.pelvis_height)
        stalled = False
        if args.duration is not None:
            start = last = clock["t"]
            wall = time.monotonic()
            while last - start < args.duration:
                rclpy.spin_once(node, timeout_sec=0.05)
                if clock["t"] > last:
                    last, wall = clock["t"], time.monotonic()
                elif time.monotonic() - wall > _WAIT_S:
                    stalled = True
                    break
            send(0, 0.0, 0.0, 0.0, -1.0)
            print(f"held {last - start:.2f} s of sim time, then sent idle")
        spin_until(lambda: False, 0.3)  # let the last messages leave before shutdown
        if stalled:
            print("/clock stopped advancing (sim paused?); sent idle", file=sys.stderr)
            return 1
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    args = _parse()
    code = (_run_ros if args.source == "ros" else _run_python)(args)
    if code == 0 and args.duration is None and not args.stop and args.mode != 0:
        print("sent; the command holds until replaced (send --stop to stop the robot)")
    return code


if __name__ == "__main__":
    sys.exit(main())
