"""The DDS Benchmark's Walk sequence: stand 2 s, then walk a 2 m square, repeating."""

from oc.control.g1_walk import WalkPlan

_plan = WalkPlan().stop(2)
for _ in range(4):
    _plan.move_forward(2).turn_left(90)
SIMPLE_WALK_SEQUENCE = _plan.to_walk_sequence()
