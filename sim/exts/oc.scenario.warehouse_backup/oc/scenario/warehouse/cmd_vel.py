import omni.graph.core as og
import omni.kit.app
import omni.usd
from oc.control.g1_sonic import G1Wbc, LocomotionCommand

from .api import CMD_VEL_GRAPH, ros


class CmdVelPoller:
    """While ros(), forwards each /cmd_vel Twist that the Scenario's ROS2_CmdVel graph receives to `walker` as a
    Locomotion command, on every app update. The graph is absent until open_scenario() (and when ros() is off it
    is inactive), so the poller resolves its attributes lazily."""

    def __init__(self, walker: G1Wbc):
        self._walker = walker
        self._attributes = None  # (count, linear, angular) of the graph's nodes once resolved
        self._seen = 0  # the Counter value last handled
        self._subscription = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
            lambda _: self._poll(), name="oc.scenario.warehouse.cmd_vel"
        )

    def stop(self) -> None:
        self._subscription = None

    def _poll(self) -> None:
        if not ros():
            return
        if self._attributes is None:
            stage = omni.usd.get_context().get_stage()
            if stage is None:  # no stage open yet
                return
            prim = stage.GetPrimAtPath(CMD_VEL_GRAPH)
            if not prim or not prim.IsActive():
                return
            self._attributes = (
                og.Controller.attribute(f"{CMD_VEL_GRAPH}/Counter.outputs:count"),
                og.Controller.attribute(f"{CMD_VEL_GRAPH}/SubscribeTwist.outputs:linearVelocity"),
                og.Controller.attribute(f"{CMD_VEL_GRAPH}/SubscribeTwist.outputs:angularVelocity"),
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
                self._walker.command(LocomotionCommand(vx, vy, angular.get()[2]))
