"""Offline pairing checks only; never pairs a PC browser as a real headset."""

import asyncio
import unittest
from unittest.mock import patch
from quest_lan import PAIR_CODE_SECONDS, QuestLan, pairing_page


class PairingTests(unittest.TestCase):
    def test_renewed_code_has_one_hour_window(self):
        self.assertEqual(PAIR_CODE_SECONDS, 3600)
        self.lan.runner = object()
        with patch("quest_lan.time.monotonic", return_value=100):
            asyncio.run(self.lan.renew())
            self.assertEqual(self.lan.expires, 3700)
            self.assertEqual(self.lan.snapshot()["code_remaining_seconds"], 3600)
            self.assertEqual(self.lan.info["code_valid_seconds"], 3600)

    def setUp(self):
        self.lan = QuestLan(None, None, None)
        self.lan.code = "012345"  # Isolated fixture, never used by a live listener.
        self.lan.expires = 100.0

    def test_current_code_wrong_code_and_expiry(self):
        self.assertIsNone(self.lan.code_error(" 012345 ", now=99))
        self.assertIn("does not match", self.lan.code_error("999999", now=99))
        self.assertIn("expired", self.lan.code_error("012345", now=100))
        self.assertIsNotNone(self.lan.code_error("１２３４５６", now=99))

    def test_remaining_time_expires_instead_of_fixed_fifteen_minutes(self):
        with patch("quest_lan.time.monotonic", return_value=90):
            self.assertEqual(self.lan.snapshot()["code_remaining_seconds"], 10)
        with patch("quest_lan.time.monotonic", return_value=101):
            self.assertTrue(self.lan.snapshot()["code_expired"])

    def test_error_page_keeps_retry_form_and_escapes_messages(self):
        html = pairing_page("Wrong <code>")
        self.assertIn('action="/pair"', html)
        self.assertIn('href="/"', html)
        self.assertIn("Wrong &lt;code&gt;", html)


if __name__ == "__main__":
    unittest.main()
