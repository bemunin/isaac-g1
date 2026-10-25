"""Kit side of the kinematic planner: subscribes to the ROS 2 commands."""

from collections.abc import Callable

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.graph.core as og
import omni.kit.app
from oc.robot.g1 import G1

# Graph under the robot prim that subscribes to /sonic/mode and /sonic/pelvis_height.
_GRAPH = "Graph/ROS2_KinematicPlannerCommand"
# (node name, topic, message package, subfolder, name) of each command subscriber.
_MODE = ("Mode", "/sonic/mode", "std_msgs", "msg", "Int32")
_PELVIS_HEIGHT = ("PelvisHeight", "/sonic/pelvis_height", "std_msgs", "msg", "Float32")


def _create_graph(graph_path: str) -> None:
    """Creates the graph with a ROS2Subscriber and a Counter per command."""
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [("OnPlaybackTick", "omni.graph.action.OnPlaybackTick")]
            + [
                (f"{node}{suffix}", type_)
                for node, *_ in (_MODE, _PELVIS_HEIGHT)
                for suffix, type_ in (("", "isaacsim.ros2.bridge.ROS2Subscriber"), ("Counter", "omni.graph.action.Counter"))
            ],
            og.Controller.Keys.SET_VALUES: [(f"{node}.inputs:topicName", topic) for node, topic, *_ in (_MODE, _PELVIS_HEIGHT)],
            og.Controller.Keys.CONNECT: [("OnPlaybackTick.outputs:tick", f"{node}.inputs:execIn") for node, *_ in (_MODE, _PELVIS_HEIGHT)]
            + [(f"{node}.outputs:execOut", f"{node}Counter.inputs:execIn") for node, *_ in (_MODE, _PELVIS_HEIGHT)],
        },
    )
    for node, _, package, subfolder, name in (_MODE, _PELVIS_HEIGHT):  # the message type is set after the node exists
        for attribute, value in (("messagePackage", package), ("messageSubfolder", subfolder), ("messageName", name)):
            og.Controller.attribute(f"{graph_path}/{node}.inputs:{attribute}").set(value)


def _poller(graph_path: str, handlers: dict[str, Callable]) -> Callable[[object], None]:
    """Returns the update callback that calls the handler of a node each time its Counter changes."""
    attributes = None  # {node: (count, data)} once resolved
    seen = {}  # node -> the Counter value last handled

    def poll(_) -> None:
        nonlocal attributes
        if attributes is None:
            attributes = {
                node: (
                    og.Controller.attribute(f"{graph_path}/{node}Counter.outputs:count"),
                    og.Controller.attribute(f"{graph_path}/{node}.outputs:data"),
                )
                for node in handlers
            }
            seen.update({node: count.get() for node, (count, _) in attributes.items()})  # left from an earlier Play
        for node, (count, data) in attributes.items():
            if not count.is_valid():  # another stage was opened
                attributes = None
                return
            latest = count.get()
            if latest != seen[node]:
                seen[node] = latest
                if latest > 0:  # 0: the Counter was reset by a Stop
                    handlers[node](data.get())

    return poll


def subscribe_commands(
    robot_prim_path: str, on_mode: Callable[[int], None], on_pelvis_height: Callable[[float], None]
) -> Callable[[], None]:
    """Calls `on_mode(int)` and `on_pelvis_height(float)` for each message on /sonic/mode and /sonic/pelvis_height; returns stop().

    The graph sits under the robot's Graph scope, so G1.set_ros() toggles it; it is not created while that scope is
    inactive.
    """
    graph_path = f"{robot_prim_path}/{_GRAPH}"
    scope = stage_utils.get_current_stage().GetPrimAtPath(graph_path.rsplit("/", 1)[0])
    if not scope or not scope.IsActive():
        carb.log_warn(f"{graph_path}: Graph scope is inactive, /sonic commands are not subscribed")
        return lambda: None
    _create_graph(graph_path)
    subscription = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
        _poller(graph_path, {_MODE[0]: on_mode, _PELVIS_HEIGHT[0]: on_pelvis_height}),
        name=f"oc.control.g1_sonic.kinematic_planner_command {robot_prim_path}",
    )

    def stop() -> None:
        nonlocal subscription
        subscription = None  # dropping the subscription unsubscribes; the callbacks are not called again

    return stop


def connect(kinematic_planner, robot_prim_path: str) -> Callable[[], None]:
    """Feeds `kinematic_planner` from /cmd_vel, /sonic/mode and /sonic/pelvis_height of the G1 at `robot_prim_path`; returns stop()."""
    cmd_vel = G1(robot_prim_path).subscribe_cmd_vel(kinematic_planner.set_cmd_vel)
    stop_commands = subscribe_commands(robot_prim_path, kinematic_planner.set_mode, kinematic_planner.set_pelvis_height)

    def stop() -> None:
        cmd_vel.stop()
        stop_commands()

    return stop
