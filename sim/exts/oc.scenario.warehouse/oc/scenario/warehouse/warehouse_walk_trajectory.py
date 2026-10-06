"""The warehouse Walk trajectory: stand 2 s, then a rectangle loop through aisles 4 and 3, repeating.

The G1 starts in aisle 4 (BOXES J-L) at (-8, 9.5) facing +y. Aisle 4 is clear for x -9.75..-6.25 and
aisle 3 (BOXES G-I) for x -14.75..-11.25; their racks span y 8.75..25.0, with open floor past both ends.
"""

from oc.control.g1_walk import WalkPlan

_plan = WalkPlan(loop=True).wait(2)
_plan.move_forward(17)  # up aisle 4 to (-8, 26.5), 1.5 m past the rack ends
_plan.turn_left(90).move_forward(5)  # across to aisle 3 at (-13, 26.5)
_plan.turn_left(90).move_forward(19.5)  # down aisle 3 to (-13, 7), 1.75 m below the rack ends
_plan.turn_left(90).move_forward(5)  # across to aisle 4 at (-8, 7)
_plan.turn_left(90).move_forward(2.5)  # back to the start (-8, 9.5) facing +y
WAREHOUSE_WALK_TRAJECTORY = _plan.to_trajectory()
