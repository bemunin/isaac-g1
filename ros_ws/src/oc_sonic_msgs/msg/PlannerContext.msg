# What the policy publishes for the planner to plan from (every policy step).
builtin_interfaces/Time stamp
# Absolute policy frame (50 Hz) the context starts at; a plan made from it starts at this frame.
uint64 frame
# 4 frames at 30 Hz of the blended motion, MuJoCo qpos (root xyz, root quaternion wxyz, 29 joints in MuJoCo order),
# 4x36 row by row.
float32[] qpos
# World yaw - planner yaw [rad]
float64 heading_offset
# Measured world yaw of the pelvis [rad]
float64 measured_yaw
