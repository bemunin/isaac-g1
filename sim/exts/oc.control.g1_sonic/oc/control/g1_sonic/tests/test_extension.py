import omni.kit.app
import omni.kit.test

import oc.control.g1_sonic


class TestG1Sonic(omni.kit.test.AsyncTestCase):
    async def test_extension_enabled(self):
        manager = omni.kit.app.get_app().get_extension_manager()
        self.assertTrue(manager.is_extension_enabled("oc.control.g1_sonic"))

    async def test_add(self):
        self.assertEqual(oc.control.g1_sonic.add(2, 3), 5)
