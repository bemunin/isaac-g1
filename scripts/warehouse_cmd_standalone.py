"""Warehouse Cmd (Standalone workflow): open the warehouse Scenario's stage with the G1's ROS 2 graphs on, and
command the G1 over ROS 2 with SONIC. Play until the app closes.

Walk from /cmd_vel; sit, kneel, crawl and stand from /posture_cmd (geometry_msgs/Twist: linear.x = id 0 stand,
4 squat, 5/6 kneel two/one leg, 8/14 hand/elbow crawl; linear.z = pelvis height [m] for the postures). A posture
holds in place; a crawl is a low mode that /cmd_vel then drives, and holds its low pose when /cmd_vel is quiet. To
stand up from elbow crawl, enter hand crawl (8) first, then stand (0). The Scenario's layer is not saved. The
oc.scenario.warehouse extension opens the Scenario (physics rate, obstacles) and plays it, through its autostart."""

import argparse
import os
import sys
from pathlib import Path

from warehouse import obstacles_argument

from sim import PHYSICS_EXPERIENCE, ensure_ros_environment

ROOT = Path(__file__).resolve().parents[1]

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("-p", "--physics", choices=PHYSICS_EXPERIENCE, default="newton", help="physics engine (default: newton)")
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
args = parser.parse_args()

try:
    ensure_ros_environment()  # the ROS 2 environment only takes effect in a new process
except (OSError, RuntimeError) as exc:
    sys.exit(f"Unable to launch the warehouse: {exc}")

from isaacsim import SimulationApp

obstacles = ",".join(dict.fromkeys(args.obstacles))  # de-duplicated, in the order given

simulation_app = SimulationApp(
    {
        "headless": args.headless,
        "extra_args": [
            *("--ext-folder", str(ROOT / "sim" / "exts")),
            *("--enable", "oc.control.g1_sonic", "--enable", "oc.scenario.warehouse"),
            # Enabled at startup, before the extension's setup_scene activates the G1's ROS 2 graphs; Isaac Sim
            # itself enables the bridge only after startup.
            *("--enable", "isaacsim.ros2.bridge"),
            "--/exts/oc.scenario.warehouse/autostart=true",
            "--/exts/oc.scenario.warehouse/ros=true",  # the ROS 2 graphs stay on for /cmd_vel
            *([f"--/exts/oc.scenario.warehouse/obstacles={obstacles}"] if obstacles else []),
        ],
    },
    experience=f"{os.environ['EXP_PATH']}/{PHYSICS_EXPERIENCE[args.physics]}.kit",
)

# Kit modules are importable only once the app runs.
import carb
import omni.graph.core as og
import omni.timeline
import omni.usd
from oc.control.g1_sonic import G1Wbc, LocomotionCommand
from oc.control.g1_sonic.locomotion_command import Crawl, Posture

CMD_VEL_GRAPH = "/World/G1/Graph/ROS2_CmdVel"  # on the G1
POSTURE_GRAPH = "/World/Graph/ROS2_Posture"
POSTURE_TOPIC = "/posture_cmd"
_SETUP_UPDATES = 600  # updates the extension gets to open the Scenario and spawn the G1

# The extension opens the Scenario on the app's updates; wait for the G1's /cmd_vel graph.
for _ in range(_SETUP_UPDATES):
    if not simulation_app.is_running():
        sys.exit("The app closed before the Scenario was ready")
    simulation_app.update()
    if (stage := omni.usd.get_context().get_stage()) and stage.GetPrimAtPath(CMD_VEL_GRAPH):
        break
else:
    simulation_app.close()
    sys.exit(f"The Scenario was not ready after {_SETUP_UPDATES} updates: no {CMD_VEL_GRAPH}")

keys = og.Controller.Keys
# /cmd_vel drives walking (and a selected crawl): the Scenario's ROS2_CmdVel graph subscribes to it. The loop
# forwards each Twist once and SONIC's 0.5 s timeout stops the G1 when /cmd_vel goes quiet.
count = og.Controller.attribute(f"{CMD_VEL_GRAPH}/Counter.outputs:count")
linear = og.Controller.attribute(f"{CMD_VEL_GRAPH}/SubscribeTwist.outputs:linearVelocity")
angular = og.Controller.attribute(f"{CMD_VEL_GRAPH}/SubscribeTwist.outputs:angularVelocity")

# A second Twist subscriber for postures and crawling: linear.x = id (0 stand, 4 squat, 5/6 kneel, 8/14 crawl),
# linear.z = pelvis height [m] for the postures. Its own Counter, so the loop dispatches each /posture_cmd once.
og.Controller.edit(
    {"graph_path": POSTURE_GRAPH, "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("SubscribePosture", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
            ("Counter", "omni.graph.action.Counter"),
        ],
        keys.SET_VALUES: [("SubscribePosture.inputs:topicName", POSTURE_TOPIC)],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "SubscribePosture.inputs:execIn"),
            ("SubscribePosture.outputs:execOut", "Counter.inputs:execIn"),
        ],
    },
)
posture_count = og.Controller.attribute(f"{POSTURE_GRAPH}/Counter.outputs:count")
posture_linear = og.Controller.attribute(f"{POSTURE_GRAPH}/SubscribePosture.outputs:linearVelocity")

robot = G1Wbc("/World/G1")
robot.start()  # stands until a command arrives


def dispatch_posture(mode: int, height: float) -> None:
    """Act on a /posture_cmd id: 0 stands (halt); 4/5/6 = squat/kneel/kneel_one_leg, a static hold at pelvis height
    [m]; 8/14 = hand/elbow crawl, a low moving mode that /cmd_vel then drives (height ignored). An out-of-range
    height or an unknown id is logged, never raised (a raise here ends the run)."""
    try:
        if mode == 0:
            robot.halt()
        elif mode == Posture.SQUAT:
            robot.squat(height)
        elif mode == Posture.KNEEL_TWO_LEGS:
            robot.kneel(height)
        elif mode == Posture.KNEEL_ONE_LEG:
            robot.kneel_one_leg(height)
        elif mode == Crawl.HANDS:
            robot.crawl_hands()
        elif mode == Crawl.ELBOWS:
            robot.crawl_elbows()
        else:
            carb.log_warn(f"/posture_cmd: ignoring unknown posture id {mode}")
    except ValueError as error:  # G1Wbc raises on an out-of-range height
        carb.log_warn(f"/posture_cmd: {error}")


received = 0
posture_received = 0
while simulation_app.is_running():
    simulation_app.update()
    if (latest := count.get()) != received:  # /cmd_vel -> walk (or drive a selected crawl)
        received = latest
        vx, vy, _ = linear.get()
        robot.command(LocomotionCommand(vx, vy, angular.get()[2]))
    if (latest := posture_count.get()) != posture_received:  # /posture_cmd -> posture or crawl mode
        posture_received = latest
        mode_raw, _, height = posture_linear.get()
        dispatch_posture(round(float(mode_raw)), float(height))

# Stop before closing, as warehouse.py does.
omni.timeline.get_timeline_interface().stop()
simulation_app.update()
simulation_app.close()
