"""Warehouse (Standalone workflow): open the warehouse Scenario (oc.scenario.warehouse), whose G1 walks a
loop through the open hall (with the controller chosen by --controller), and play until the app closes."""

import argparse
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
from sim import PHYSICS_EXPERIENCE

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--physics", choices=PHYSICS_EXPERIENCE, default="newton", help="physics engine (default: newton)")
parser.add_argument("--controller", choices=("walk", "wbc"), default="walk", help="G1 controller (default: walk)")
parser.add_argument("--ros", action="store_true", help="publish the G1's ROS 2 topics")
parser.add_argument("--headless", action="store_true", help="run without a window")
args = parser.parse_args()

from isaacsim import SimulationApp

extensions = [
    *("--ext-folder", str(ROOT / "sim" / "exts"), "--enable", "oc.scenario.warehouse"),
    f"--/exts/oc.scenario.warehouse/controller={args.controller}",
    f"--/exts/oc.scenario.warehouse/ros={str(args.ros).lower()}",
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
