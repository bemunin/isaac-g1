"""The warehouse Behavior sequence for the whole-body controller: stand 2 s, walk up aisle 4, down aisle 3, side-step
back into aisle 4, repeating.

Same loop as warehouse_walk_trajectory.py: the G1 starts in aisle 4 (BOXES J-L) at (-8, 9.5) facing +y. Aisle 4 is
clear for x -9.75..-6.25 and aisle 3 (BOXES G-I) for x -14.75..-11.25; their racks span y 8.75..25.0, with open floor
past both ends.
"""

from oc.control.g1_wbc_sonic import BehaviorSequence

WAREHOUSE_SEQUENCE = (
    BehaviorSequence(loop=True)
    .wait(seconds=2)
    .walk(meters=17)  # up aisle 4 to (-8, 26.5), slowing down before the spin
    .spin(degrees=90)
    .walk(meters=5)  # across to aisle 3 at (-13, 26.5)
    .spin(degrees=90)
    .walk(meters=19.5)  # down aisle 3 to (-13, 7), 1.75 m below the rack ends
    .spin(degrees=90)
    .walk(meters=3)  # toward aisle 4, to (-10, 7)
    .spin(degrees=90)
    .side_step(meters=-2)  # facing +y, step right into aisle 4 at (-8, 7)
    .walk(meters=2.5)  # back to the start (-8, 9.5) facing +y
)
