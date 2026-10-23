"""SONIC planner and policy as two composable nodes in one multithreaded container, with intra-process comms.

The planner's ONNX model takes ~75 ms on the CPU; the policy runs at 50 Hz and never waits for it.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

# This file resolves (through colcon's symlink install) to ros_ws/src/oc_sonic/launch/.
WEIGHTS = Path(__file__).resolve().parents[1] / "data" / "sonic"


def generate_launch_description():
    model_dir = LaunchConfiguration("model_dir")
    use_sim_time = LaunchConfiguration("use_sim_time")
    decimation = LaunchConfiguration("decimation")
    parameters = {"model_dir": model_dir, "use_sim_time": use_sim_time}
    options = [{"use_intra_process_comms": True}]
    return LaunchDescription([
        DeclareLaunchArgument("model_dir", default_value=str(WEIGHTS), description="Folder of the SONIC ONNX models"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("decimation", default_value="2", description="Sim steps per policy step: physics Hz / 50"),
        ComposableNodeContainer(
            name="sonic_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[
                ComposableNode(
                    package="oc_sonic",
                    plugin="oc_sonic::PlannerNode",
                    name="planner",
                    parameters=[parameters],
                    extra_arguments=options,
                ),
                ComposableNode(
                    package="oc_sonic",
                    plugin="oc_sonic::PolicyNode",
                    name="policy",
                    parameters=[parameters, {"decimation": ParameterValue(decimation, value_type=int)}],
                    extra_arguments=options,
                ),
            ],
            output="screen",
        ),
    ])
