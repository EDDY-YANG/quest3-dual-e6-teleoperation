"""Offline release tests for opt-in device binding and empty return targets."""
import json
import tempfile
import unittest
from pathlib import Path

from device_settings import LOCAL_FILE, load_device_config, validate
from server import e6_targets
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
