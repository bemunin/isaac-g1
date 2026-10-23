"""Warehouse Cmd (Standalone workflow): open the warehouse Scenario's stage with the G1's ROS 2 graphs on and play until
the app closes. The G1 is driven by the oc_sonic ROS 2 nodes, not by Isaac Sim: in another shell run
`ros2 launch oc_sonic sonic.launch.py` (after `pixi run build`), then command it over ROS 2:

    ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
    ros2 service call /planner/set_mode oc_sonic_msgs/srv/SetMode "{mode: 2, pelvis_height: -1.0}"

Modes: 0 idle, 1 slow walk, 2 walk (default), 3 run, 4 squat, 5/6 kneel two/one leg (pelvis_height in m, -1 = default),
8/14 hand/elbow crawl; the walking and crawling modes follow /cmd_vel. To stand up from elbow crawl, enter hand crawl
(8) first, then idle (0). The Scenario's layer is not saved. The oc.scenario.warehouse extension is enabled for its
G1Drive (SONIC gains and standing pose on Play) and for the obstacles (-o)."""

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
            *("--enable", "oc.scenario.warehouse"),  # its G1Drive sets the G1 up on Play
        ],
    },
    experience=f"{os.environ['EXP_PATH']}/{PHYSICS_EXPERIENCE[args.physics]}.kit",
)

# Kit modules are importable only once the app runs.
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.timeline
from oc.scenario.warehouse import PHYSICS_RATE, apply_obstacles

# Isaac Sim enables the ROS 2 bridge only after startup; the Scenario's graphs need its nodes now.
app_utils.enable_extension("isaacsim.ros2.bridge")
app_utils.enable_extension("isaacsim.robot_setup.virtual_gantry", enabled=False)  # stage scans slow the sim; no gantries here
simulation_app.update()

SCENARIO = ROOT / "sim" / "exts" / "oc.scenario.warehouse" / "usd" / "warehouse_g1_scene.usda"

opened, stage = stage_utils.open_stage(str(SCENARIO))
if not opened:
    simulation_app.close()
    sys.exit(f"could not open {SCENARIO}")
apply_obstacles(stage, args.obstacles)
scene = stage.GetPrimAtPath("/PhysicsScene")
for name in ("newton:timeStepsPerSecond", "physxScene:timeStepsPerSecond"):
    if attribute := scene.GetAttribute(name):
        attribute.Set(PHYSICS_RATE)

omni.timeline.get_timeline_interface().play()

while simulation_app.is_running():
    simulation_app.update()

# Stop before closing, as warehouse_standalone.py does.
omni.timeline.get_timeline_interface().stop()
simulation_app.update()
simulation_app.close()
