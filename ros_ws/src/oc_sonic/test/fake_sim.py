"""A stand-in for Isaac Sim to try the SONIC nodes without it: publishes /clock, /joint_states (the standing pose, MuJoCo
joint names) and an identity /imu/data, all at 100 Hz.

    ros2 launch oc_sonic sonic.launch.py        # in one shell
    python ros_ws/src/oc_sonic/test/fake_sim.py # in another
    ros2 topic hz /joint_commands               # about 50 Hz
"""

import time

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState

LEG = ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")
ARM = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")
NAMES = (
    [f"{side}_{joint}_joint" for side in ("left", "right") for joint in LEG]
    + ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"]
    + [f"{side}_{joint}_joint" for side in ("left", "right") for joint in ARM]
)
STANDING = [-0.312, 0.0, 0.0, 0.669, -0.363, 0.0] * 2 + [0.0] * 3 + [0.2, 0.2, 0.0, 0.6, 0.0, 0.0, 0.0] + [0.2, -0.2, 0.0, 0.6, 0.0, 0.0, 0.0]


class FakeSim(Node):
    def __init__(self):
        super().__init__("fake_sim")
        self.clock = self.create_publisher(Clock, "/clock", 10)
        self.joints = self.create_publisher(JointState, "/joint_states", 10)
        self.imu = self.create_publisher(Imu, "/imu/data", 10)
        self.start = time.monotonic()
        self.create_timer(0.01, self.tick)

    def tick(self):
        t = time.monotonic() - self.start
        stamp = Time(sec=int(t), nanosec=int((t % 1) * 1e9))
        self.clock.publish(Clock(clock=stamp))
        joints = JointState(name=NAMES, position=STANDING, velocity=[0.0] * 29)
        joints.header.stamp = stamp
        self.joints.publish(joints)
        imu = Imu()
        imu.header.stamp = stamp
        imu.orientation.w = 1.0
        self.imu.publish(imu)


def main():
    rclpy.init()
    rclpy.spin(FakeSim())


if __name__ == "__main__":
    main()
