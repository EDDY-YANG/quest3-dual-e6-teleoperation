"""Read-only visualization asset delivery on an isolated HTTP server."""

import unittest

from aiohttp.test_utils import TestClient, TestServer

from server import make_app


class StaticAssetTests(unittest.IsolatedAsyncioTestCase):
    async def test_allowlisted_urdf_and_rejection_of_unlisted_files(self):
        client = TestClient(TestServer(make_app(recording_enabled=False)))
        await client.start_server()
        headers = {"Host": "127.0.0.1:8765"}
        try:
            response = await client.get("/static/robot-assets/e6/me6_robot.urdf", headers=headers)
            self.assertEqual(response.status, 200)
            self.assertIn('name="me6_robot"', await response.text())
            for url in ("/static/robot-assets/e6/lan-key.pem",
                        "/static/robot-assets/runtime/config.json",
                        "/static/robot-assets/three/unapproved.js"):
                response = await client.get(url, headers=headers)
                self.assertEqual(response.status, 404, url)
        finally:
            await client.close()


if __name__ == "__main__":
    unittest.main()
