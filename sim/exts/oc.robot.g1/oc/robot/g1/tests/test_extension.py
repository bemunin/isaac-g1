import omni.kit.app
import omni.kit.test

import oc.robot.g1


class TestG1(omni.kit.test.AsyncTestCase):
    async def test_extension_enabled(self):
        manager = omni.kit.app.get_app().get_extension_manager()
        self.assertTrue(manager.is_extension_enabled("oc.robot.g1"))

    async def test_add(self):
        self.assertEqual(oc.robot.g1.add(2, 3), 5)
