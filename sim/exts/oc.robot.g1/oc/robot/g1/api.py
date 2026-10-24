from collections.abc import Callable
from pathlib import Path

import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.core.experimental.utils.transform as transform_utils
from isaacsim.core.experimental.prims import XformPrim

from .ros import CmdVelSubscription

__all__ = ["G1", "G1_USD", "CmdVelSubscription"]

# G1 asset; its root prim is /G1 (defaultPrim).
G1_USD = Path(__file__).resolve().parents[3] / "usd" / "g1.usda"

# Graph scope under the robot prim; holds every ROS 2 graph (camera, lidar, cmd_vel, ...).
_GRAPH = "Graph"


class G1:
    """A Spawned G1 on the current stage at `prim_path`."""

    def __init__(self, prim_path: str):
        """Wrap a G1 that already exists at `prim_path`."""
        self._prim_path = prim_path

    @classmethod
    def spawn(
        cls,
        prim_path: str = "/World/G1",
        translation=(0.0, 0.0, 0.0),
        orientation=(0.0, 0.0, 0.0),
        ros: bool = False,
    ) -> "G1":
        """Reference the G1 at `prim_path` and place it; orientation is roll, pitch, yaw in degrees (extrinsic XYZ).

        Writes to the stage's current edit target, so the caller chooses the layer.
        """
        stage_utils.add_reference_to_stage(str(G1_USD), prim_path, prim_type="Xform")
        XformPrim(
            prim_path,
            translations=translation,
            orientations=transform_utils.euler_angles_to_quaternion(orientation, degrees=True),
            reset_xform_op_properties=True,
        )
        robot = cls(prim_path)
        robot.set_ros(ros)
        return robot

    @property
    def ros(self) -> bool:
        """Whether the robot's ROS 2 graphs are active."""
        return self._graph_prim().IsActive()

    def set_ros(self, enabled: bool) -> None:
        """Activate or deactivate the Graph scope; off, the robot publishes nothing and renders no camera or lidar."""
        self._graph_prim().SetActive(enabled)

    def subscribe_cmd_vel(self, callback: Callable[[float, float, float], None]) -> CmdVelSubscription:
        """Call `callback(vx, vy, wz)` (body-frame linear x/y, angular z) for each new /cmd_vel Twist."""
        return CmdVelSubscription(self._prim_path, callback)

    def _graph_prim(self):
        path = f"{self._prim_path}/{_GRAPH}"
        prim = stage_utils.get_current_stage().GetPrimAtPath(path)
        if not prim:
            raise RuntimeError(f"no Graph prim at {path}")
        return prim
