"""Warehouse Cmd (Standalone workflow): open the warehouse Scenario's stage with the G1's ROS 2 graphs on, and
command the G1 over ROS 2 with SONIC instead of the warehouse loop. Play until the app closes.

Walk from /cmd_vel; sit, kneel, crawl and stand from /posture_cmd (geometry_msgs/Twist: linear.x = id 0 stand,
4 squat, 5/6 kneel two/one leg, 8/14 hand/elbow crawl; linear.z = pelvis height [m] for the postures). A posture
holds in place; a crawl is a low mode that /cmd_vel then drives, and holds its low pose when /cmd_vel is quiet. To
stand up from elbow crawl, enter hand crawl (8) first, then stand (0). The Scenario's layer is not saved. The
oc.scenario.warehouse extension is enabled only for its obstacles (-o); its G1 loop is not started."""

import argparse
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
from sim import PHYSICS_EXPERIENCE, obstacles_argument

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--physics", choices=PHYSICS_EXPERIENCE, default="newton", help="physics engine (default: newton)")
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

from isaacsim import SimulationApp

simulation_app = SimulationApp(
    {
        "headless": args.headless,
        "extra_args": [
            *("--ext-folder", str(ROOT / "sim" / "exts")),
            *("--enable", "oc.control.g1_sonic", "--enable", "oc.scenario.warehouse"),  # the latter only for -o
        ],
    },
    experience=f"{os.environ['EXP_PATH']}/{PHYSICS_EXPERIENCE[args.physics]}.kit",
)

# Kit modules are importable only once the app runs.
import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.graph.core as og
import omni.timeline
from oc.control.g1_sonic import G1Wbc, LocomotionCommand
from oc.control.g1_sonic.locomotion_command import Crawl, Posture
from oc.scenario.warehouse import apply_obstacles

# Isaac Sim enables the ROS 2 bridge only after startup; the Scenario's graphs need its nodes now.
app_utils.enable_extension("isaacsim.ros2.bridge")
app_utils.enable_extension("isaacsim.robot_setup.virtual_gantry", enabled=False)  # stage scans slow the sim; no gantries here
simulation_app.update()

SCENARIO = ROOT / "sim" / "exts" / "oc.scenario.warehouse" / "usd" / "warehouse_g1_scene.usda"
PHYSICS_RATE = 200  # Hz, the rate SONIC was trained at
CMD_VEL_GRAPH = "/World/Graph/ROS2_CmdVel"
POSTURE_GRAPH = "/World/Graph/ROS2_Posture"
CMD_VEL_TOPIC = "/cmd_vel"
POSTURE_TOPIC = "/posture_cmd"

opened, stage = stage_utils.open_stage(str(SCENARIO))
if not opened:
    simulation_app.close()
    sys.exit(f"could not open {SCENARIO}")
apply_obstacles(stage, args.obstacles)
scene = stage.GetPrimAtPath("/PhysicsScene")
for name in ("newton:timeStepsPerSecond", "physxScene:timeStepsPerSecond"):
    if attribute := scene.GetAttribute(name):
        attribute.Set(PHYSICS_RATE)

keys = og.Controller.Keys
# /cmd_vel drives walking (and a selected crawl); the loop forwards each Twist once and SONIC's 0.5 s timeout
# stops the G1 when /cmd_vel goes quiet.
og.Controller.edit(
    {"graph_path": CMD_VEL_GRAPH, "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
            ("Counter", "omni.graph.action.Counter"),
        ],
        keys.SET_VALUES: [("SubscribeTwist.inputs:topicName", CMD_VEL_TOPIC)],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
            ("SubscribeTwist.outputs:execOut", "Counter.inputs:execIn"),
        ],
    },
)
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
omni.timeline.get_timeline_interface().play()


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

# Stop before closing, as warehouse_standalone.py does.
omni.timeline.get_timeline_interface().stop()
simulation_app.update()
simulation_app.close()
