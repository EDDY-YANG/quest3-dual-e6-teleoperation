"""Owned passive feedback readers. No dashboard commands or motion API."""

from e6_feedback import FeedbackReader


class RobotMonitor:
    def __init__(self, reader_factory=None):
        self.reader_factory = reader_factory
        self.readers = {}

    async def connect(self, side, robot_ip, source_ip):
        if side not in ("left", "right"):
            raise ValueError("Unknown robot side")
        if side in self.readers:
            return
        factory = self.reader_factory or FeedbackReader
        reader = factory(robot_ip, source_ip, f"{side}@{robot_ip}-via-{source_ip}", publish_hz=50)
        try:
            await reader.start()
        except Exception:
            await reader.stop()
            raise
        self.readers[side] = reader

    async def disconnect(self, side):
        reader = self.readers.pop(side, None)
        if reader:
            await reader.stop()

    def snapshot(self, side, control_reader=None):
        reader = control_reader or self.readers.get(side)
        if reader is None:
            return {"connected": False, "valid": False, "packet_valid": False,
                    "reason": "MONITOR_NOT_CONNECTED", "feedback_age_ms": None,
                    "receive_hz": 0, "monitor_owner": "none", "side": side}
        return {**reader.snapshot(), "side": side,
                "monitor_owner": "control" if control_reader else "passive_monitor"}

    async def close(self):
        for side in list(self.readers):
            await self.disconnect(side)
