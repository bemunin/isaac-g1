"""Kit side of the kinematic planner: subscribes to the ROS 2 commands.

One OmniGraph, Graph/ROS2_SonicCommand, is created under the robot's Graph scope. It has one
ROS2Subscriber -> Counter chain per topic, both driven by a shared OnPlaybackTick:

    node prefix   topic                 message
    Mode          /sonic/mode           std_msgs/Int32
    PelvisHeight  /sonic/pelvis_height  std_msgs/Float32

Each chain is polled by a _CommandSubscription on Kit updates.
"""

from collections.abc import Callable
from typing import Any

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.graph.core as og
import omni.kit.app

# --- Internal: used only in this module -------------------------------------------------------------------------------

# Scope under the robot prim that holds G1's ROS 2 graphs.
_GRAPH_SCOPE = "Graph"
# Graph under the robot prim that subscribes to /sonic/mode and /sonic/pelvis_height.
_GRAPH = f"{_GRAPH_SCOPE}/ROS2_SonicCommand"


def _set_message_type(node_path: str, message_name: str) -> None:
    """Sets the ROS2Subscriber at `node_path` to the std_msgs/`message_name` message."""
    og.Controller.attribute(f"{node_path}.inputs:messagePackage").set("std_msgs")
    og.Controller.attribute(f"{node_path}.inputs:messageSubfolder").set("msg")
    og.Controller.attribute(f"{node_path}.inputs:messageName").set(message_name)


def _create_graph(graph_path: str) -> None:
    """Creates the graph that counts the messages received on /sonic/mode and /sonic/pelvis_height."""
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ModeSubscriber", "isaacsim.ros2.bridge.ROS2Subscriber"),
                ("ModeCounter", "omni.graph.action.Counter"),
                ("PelvisHeightSubscriber", "isaacsim.ros2.bridge.ROS2Subscriber"),
                ("PelvisHeightCounter", "omni.graph.action.Counter"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ModeSubscriber.inputs:topicName", "/sonic/mode"),
                ("PelvisHeightSubscriber.inputs:topicName", "/sonic/pelvis_height"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ModeSubscriber.inputs:execIn"),
                ("ModeSubscriber.outputs:execOut", "ModeCounter.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PelvisHeightSubscriber.inputs:execIn"),
                ("PelvisHeightSubscriber.outputs:execOut", "PelvisHeightCounter.inputs:execIn"),
            ],
        },
    )
    # The message type is set after the node exists.
    _set_message_type(f"{graph_path}/ModeSubscriber", "Int32")
    _set_message_type(f"{graph_path}/PelvisHeightSubscriber", "Float32")


class _CommandSubscription:
    """Calls `callback(data)` for each new message of one Subscriber -> Counter chain of the command graph.

    `node` is the chain's node prefix ("Mode" or "PelvisHeight"). The og attributes are resolved lazily, once the
    graph prim exists and is active.
    """

    def __init__(self, graph_path: str, node: str, callback: Callable[[Any], None], name: str):
        self._graph = graph_path
        self._node = node
        self._callback = callback
        self._attributes = None  # (count, data) of the graph's nodes once resolved
        self._seen = 0  # the Counter value last handled
        self._subscription = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
            lambda _: self._poll(), name=name
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
                og.Controller.attribute(f"{self._graph}/{self._node}Counter.outputs:count"),
                og.Controller.attribute(f"{self._graph}/{self._node}Subscriber.outputs:data"),
            )
            self._seen = self._attributes[0].get()  # a count left from an earlier Play is not a new message
        count, data = self._attributes
        if not count.is_valid():  # another stage was opened
            self._attributes = None
            return
        latest = count.get()
        if latest != self._seen:
            self._seen = latest
            if latest > 0:  # 0: the Counter was reset by a Stop
                self._callback(data.get())


# --- External: the module's API ---------------------------------------------------------------------------------------
# subscribe_commands() is the only name used outside this module (by kinematic_planner.py).


def subscribe_commands(
    robot_prim_path: str, on_mode: Callable[[int], None], on_pelvis_height: Callable[[float], None]
) -> list[_CommandSubscription]:
    """Creates the /sonic/mode and /sonic/pelvis_height graph and calls `on_mode(int)` and `on_pelvis_height(float)`.

    The graph sits under G1's Graph scope, so G1.set_ros() toggles it; it is not created while that scope is
    inactive, and then no subscription is returned. Each returned item has `.stop()`.
    """
    scope = stage_utils.get_current_stage().GetPrimAtPath(f"{robot_prim_path}/{_GRAPH_SCOPE}")
    if not scope or not scope.IsActive():
        carb.log_warn(f"{robot_prim_path}: Graph scope is inactive, /sonic commands are not subscribed")
        return []

    graph_path = f"{robot_prim_path}/{_GRAPH}"
    _create_graph(graph_path)
    return [
        _CommandSubscription(
            graph_path, "Mode", on_mode, name=f"oc.control.g1_sonic.sonic_mode {robot_prim_path}"
        ),
        _CommandSubscription(
            graph_path,
            "PelvisHeight",
            on_pelvis_height,
            name=f"oc.control.g1_sonic.sonic_pelvis_height {robot_prim_path}",
        ),
    ]

