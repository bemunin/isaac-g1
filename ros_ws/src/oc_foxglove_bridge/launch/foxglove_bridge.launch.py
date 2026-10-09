"""Foxglove bridge plus the G1's robot description and link TFs, on sim time.

Isaac Sim publishes /joint_states and odom -> pelvis; robot_state_publisher adds pelvis -> every link.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node

URDF = "g1_29dof_with_hand_rev_1_0.urdf"


def generate_launch_description():
    share = Path(get_package_share_directory("oc_foxglove_bridge"))
    bridge_launch = Path(get_package_share_directory("foxglove_bridge")) / "launch" / "foxglove_bridge_launch.xml"
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": (share / "urdf" / URDF).read_text(),
                "use_sim_time": True,
            }],
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(str(bridge_launch)),
            launch_arguments={"use_sim_time": "true"}.items(),
        ),
    ])
