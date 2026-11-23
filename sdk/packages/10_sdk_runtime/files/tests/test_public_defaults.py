"""Offline release tests for opt-in device binding and empty return targets."""
import json
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from aiohttp.test_utils import TestClient, TestServer
from device_settings import LOCAL_FILE, load_device_config, validate
from server import e6_targets, make_app
from start_positions import StartPositions


class PublicDefaults(unittest.TestCase):
    def test_new_install_has_no_network_targets_or_home(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            cfg = load_device_config(path)
            self.assertEqual(validate(cfg), ())
            for side in ("left", "right"):
                with self.assertRaisesRegex(ValueError, "DEVICE_NOT_CONFIGURED"):
                    e6_targets(cfg, [side])
            home = StartPositions(path / "starts.json", None, None, None, None)
            self.assertEqual(home.saved, {})
            self.assertEqual(home.presets, {})

    def test_single_arm_and_duplicate_controller_addresses(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            cfg = load_device_config(root)
            cfg.update(left_e6_ip="172.20.8.2", left_e6_local_ip="172.20.8.5")
            p = root / LOCAL_FILE
            p.parent.mkdir()
            p.write_text(json.dumps(cfg), encoding="utf-8")
            self.assertEqual(validate(load_device_config(root)), ("left",))
            self.assertEqual(e6_targets(cfg, ["left"]), [("172.20.8.2", "172.20.8.5")])
            with self.assertRaisesRegex(ValueError, "DEVICE_NOT_CONFIGURED"):
                e6_targets(cfg, ["right"])
            cfg.update(right_e6_ip=cfg["left_e6_ip"], right_e6_local_ip=cfg["left_e6_local_ip"])
            with self.assertRaises(ValueError):
                validate(cfg)
            cfg["right_e6_local_ip"] = "172.20.8.6"
            self.assertEqual(validate(cfg), ("left", "right"))


class PreflightFeedback(unittest.IsolatedAsyncioTestCase):
    async def test_collision_before_or_during_preflight_remains_visible(self):
        for initial_mode in (11, 4):
            with self.subTest(initial_mode=initial_mode):
                await self._check_collision(initial_mode)

    async def _check_collision(self, initial_mode):
        mode = initial_mode
        cfg = {"left_e6_ip": None, "left_e6_local_ip": None,
               "right_e6_ip": "172.20.8.2", "right_e6_local_ip": "172.20.8.5"}

        class Reader:
            def __init__(self, ip, source_ip, robot_id, **kwargs):
                self.robot_id = robot_id
                self.connected = False

            async def start(self):
                self.connected = True

            async def stop(self):
                self.connected = False

            def snapshot(self):
                return {
                    "robot_id": self.robot_id, "connected": self.connected, "packet_valid": self.connected,
                    "robot_mode": mode, "valid": self.connected and mode == 4,
                    "host_monotonic_s": time.monotonic(), "feedback_age_ms": 0,
                    "robot_type": 150, "user": 0, "tool": 0, "error_status": 0,
                    "joint_position_deg": [0.0] * 6,
                }

        def control_reader(*args, **kwargs):
            nonlocal mode
            mode = 11  # Protection appears after the initial monitor check.
            return Reader(*args, **kwargs)

        with (tempfile.TemporaryDirectory() as tmp,
              patch("server.load_device_config", return_value=cfg),
              patch("gripper_control.settings", return_value={}),
              patch("robot_monitor.FeedbackReader", Reader),
              patch("e6_feedback.FeedbackReader", side_effect=control_reader),
              patch("dobot_safe_sdk.BoundDobotDashboard") as dashboard):
            app = make_app(recording_enabled=False, start_positions_path=Path(tmp) / "starts.json")
            headers = {"Host": "127.0.0.1:8765"}
            async with TestClient(TestServer(app)) as client:
                before = await client.post("/api/monitor/connect?side=right", headers=headers)
                self.assertEqual(before.status, 200)
                failed = await client.post("/api/teleop/prepare?side=right", headers=headers)
                self.assertEqual(failed.status, 400)
                self.assertIn("右臂处于碰撞保护（Mode 11）", (await failed.json())["error"])
                after = await client.get("/api/headset/status", headers=headers)
                status = await after.json()
                robot = status["robots"]["right"]
                self.assertEqual(robot["monitor_owner"], "passive_monitor")
                self.assertTrue(robot["connected"])
                self.assertEqual(robot["robot_mode"], 11)
                self.assertFalse(status["capabilities"]["arm"])
                if initial_mode == 11:
                    dashboard.assert_not_called()
                else:
                    dashboard.return_value.close.assert_called_once()
                    dashboard.return_value.EnableRobot.assert_not_called()
