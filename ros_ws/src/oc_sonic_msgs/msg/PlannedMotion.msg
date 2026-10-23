# A 50 Hz reference motion from the planner, in the planner's frame, joints in IsaacLab order.
builtin_interfaces/Time stamp
# Absolute policy frame of the first sample.
uint64 start_frame
uint32 num_frames
# num_frames x 29
float32[] joints
# num_frames x 29
float32[] joint_velocities
# num_frames x 4, wxyz
float32[] quats
# num_frames x 3
float32[] positions
