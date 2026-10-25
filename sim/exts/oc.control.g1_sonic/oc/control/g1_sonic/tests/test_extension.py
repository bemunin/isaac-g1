import omni.kit.app
import omni.kit.test


class TestG1Sonic(omni.kit.test.AsyncTestCase):
    async def test_extension_enabled(self):
        manager = omni.kit.app.get_app().get_extension_manager()
        self.assertTrue(manager.is_extension_enabled("oc.control.g1_sonic"))
