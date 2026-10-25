# G1 Sonic Whole-Body Controller

G1 Sonic Whole-Body Controller extension.

Extension ID: `oc.control.g1_sonic`

## Usage

```python
controller = SonicController("/World/G1")  # ros=False skips the ROS 2 subscriptions
controller.command(SonicMode.WALK, vx=0.5)  # holds until replaced; with ros=True the latest of ROS 2 and Python wins
controller.forward(dt)  # every physics step; sets up on the first call after construction or reset()
controller.reset()  # on Stop: the next forward() starts again from the measured pose
controller.close()  # on shutdown
```
