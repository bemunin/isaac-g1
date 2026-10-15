"""cmd_vel bridge: forwards /cmd_vel, received by the G1's ROS2_CmdVel graph, to G1Sonic.command()."""

import omni.graph.core as og
import omni.kit.app

CMD_VEL_GRAPH = "/World/G1/Graph/ROS2_CmdVel"  # OnPlaybackTick -> ROS2SubscribeTwist("/cmd_vel") -> Counter


class CmdVelBridge:
    """Each app update, calls command((vx, vy, wz)) with the latest /cmd_vel if a new message arrived.

    The graph's Counter counts received messages, so a repeated identical Twist still counts as new; the controller's
    0.5 s timeout then stops the robot when messages stop.
    """

    def __init__(self, command, graph_path: str = CMD_VEL_GRAPH) -> None:
        self._command = command
        self._graph_path = graph_path
        self._count = 0
        stream = omni.kit.app.get_app().get_update_event_stream()
        self._subscription = stream.create_subscription_to_pop(self._on_update, name="oc.scenario.warehouse.cmd_vel")

    def destroy(self) -> None:
        self._subscription = None

    def _on_update(self, event) -> None:
        try:
            count = og.Controller.get(og.Controller.attribute(f"{self._graph_path}/Counter.outputs:count"))
        except og.OmniGraphError:  # no stage or graph yet
            return
        if count == self._count or count == 0:  # 0: no message since the graph (re)started
            return
        self._count = count
        twist = f"{self._graph_path}/SubscribeTwist.outputs:"
        linear = og.Controller.get(og.Controller.attribute(twist + "linearVelocity"))
        angular = og.Controller.get(og.Controller.attribute(twist + "angularVelocity"))
        self._command((float(linear[0]), float(linear[1]), float(angular[2])))
