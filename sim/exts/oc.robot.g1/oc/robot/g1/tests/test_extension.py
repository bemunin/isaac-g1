import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.app
import omni.kit.test

from oc.robot.g1 import G1


class TestG1(omni.kit.test.AsyncTestCase):
    async def test_extension_enabled(self):
        manager = omni.kit.app.get_app().get_extension_manager()
        self.assertTrue(manager.is_extension_enabled("oc.robot.g1"))

    async def test_spawn(self):
        stage = await stage_utils.create_new_stage_async()
        robot = G1.spawn("/World/G1", (1, 2, 0.8), (0, 0, 90))
        self.assertTrue(stage.GetPrimAtPath("/World/G1").IsValid())
        self.assertFalse(robot.ros)
        robot.set_ros(True)
        self.assertTrue(robot.ros)
