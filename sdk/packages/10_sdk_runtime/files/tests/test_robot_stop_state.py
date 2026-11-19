"""Protected stationary feedback is valid only for stop confirmation."""
import time
import unittest

from robot_stop_state import stop_feedback


class StopStateTests(unittest.TestCase):
    def test_error_or_stale_protected_feedback_is_not_accepted(self):
        packet = {"connected": True, "valid": False, "packet_valid": True,
                  "host_monotonic_s": time.monotonic(), "robot_mode": 11, "robot_type": 150,
                  "error_status": 0, "user": 0, "tool": 0, "joint_position_deg": [0]*6}
        self.assertFalse(stop_feedback(packet)["valid"])
        for change in ({"error_status": 1}, {"packet_valid": False}, {"host_monotonic_s": 0}, {"robot_mode": 9}):
            with self.assertRaises(ValueError):
                stop_feedback({**packet, **change})
