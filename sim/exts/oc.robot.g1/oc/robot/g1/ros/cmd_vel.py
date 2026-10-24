from collections.abc import Callable

import isaacsim.core.experimental.utils.stage as stage_utils
import omni.graph.core as og
import omni.kit.app

# Graph under the robot prim that subscribes to /cmd_vel.
_CMD_VEL_GRAPH = "Graph/ROS2_CmdVel"


class CmdVelSubscription:
    """Calls `callback(vx, vy, wz)` for each new /cmd_vel Twist the G1's ROS2_CmdVel graph receives.

    The graph is absent until Spawn and inactive while ROS is off, so the og attributes are resolved lazily.
    """

    def __init__(self, prim_path: str, callback: Callable[[float, float, float], None]):
        self._graph = f"{prim_path}/{_CMD_VEL_GRAPH}"
        self._callback = callback
        self._attributes = None  # (count, linear, angular) of the graph's nodes once resolved
        self._seen = 0  # the Counter value last handled
        self._subscription = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
            lambda _: self._poll(), name=f"oc.robot.g1.cmd_vel {prim_path}"
        )

    def stop(self) -> None:
        """Stop polling; the callback is not called again."""
        self._subscription = None

    def _poll(self) -> None:
        if self._attributes is None:
            stage = stage_utils.get_current_stage()
            if stage is None:  # no stage open yet
                return
            prim = stage.GetPrimAtPath(self._graph)
            if not prim or not prim.IsActive():
                return
            self._attributes = (
                og.Controller.attribute(f"{self._graph}/Counter.outputs:count"),
                og.Controller.attribute(f"{self._graph}/SubscribeTwist.outputs:linearVelocity"),
                og.Controller.attribute(f"{self._graph}/SubscribeTwist.outputs:angularVelocity"),
            )
            self._seen = self._attributes[0].get()  # a count left from an earlier Play is not a new Twist
        count, linear, angular = self._attributes
        if not count.is_valid():  # another stage was opened
            self._attributes = None
            return
        latest = count.get()
        if latest != self._seen:
            self._seen = latest
            if latest > 0:  # 0: the Counter was reset by a Stop
                vx, vy, _ = linear.get()
                self._callback(vx, vy, angular.get()[2])
