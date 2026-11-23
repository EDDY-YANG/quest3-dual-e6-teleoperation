"""Offline clear-only branch tests. No robot socket or motion-capable test driver."""
import json
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import AsyncMock, patch

from start_positions import StartPositions


class AlarmReset(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        root = Path(self.tmp.name)
        self.mode = 9
        self.ids = [[3, 5], [], [], [], [], [], []]
        self.identity = {"ip":"192.168.5.1","source_ip":"192.168.5.10","mac":"00-00-00-00-00-01"}
        self.info = {"remoteControl":"tcp","emergencyStop":False,"isMotion":False,"prjState":"stopped","powerState":"on"}
        self.methods = []
        self.persistent = False
        async def call(method, *, guard=None):
            if guard:
                guard()
            self.methods.append(method)
            if method == "GetErrorID":
                return {"payload":json.dumps(self.ids)}
            if method == "RecoveryClearError":
                if not self.persistent:
                    self.ids, self.mode = [[],[]], 4
            elif method != "Stop":
                raise AssertionError(f"Unexpected command {method}")
            return {"payload":"0"}
        self.ch = SimpleNamespace(identity=self.identity, call=call, release=AsyncMock(),
                                  verify_identity=AsyncMock(), controller_status=AsyncMock(return_value=self.info))
        self.homes = StartPositions(root/"home.json",AsyncMock(return_value=self.ch),AsyncMock(),
                                    self.feedback,lambda *_:True)
        self.homes.presets["right"] = {"identity":self.identity,"kind":"CONFIGURED_JOINT_TARGET",
                                       "q_deg":[0,0,-90,0,90,0],"side":"right"}

    def feedback(self, side):
        if side!="right":
            return {}
        return {"connected":True,"valid":self.mode==4,"packet_valid":True,"robot_type":150,
                "user":0,"tool":0,"robot_mode":self.mode,"error_status":int(self.mode==9),
                "host_monotonic_s":time.monotonic(),"robot_id":"right","frame_id":"f","connection_generation":1,
                "joint_position_deg":[0,0,-90,0,90,0],"pose_mm_deg":[0,0,0,180,0,0]}

    async def run_reset(self):
        self.homes.begin("right","vr","fixture",clear_only=True)
        await self.homes.task
        return self.homes.states["right"]

    async def test_reset_rechecks_all_codes_without_enable_or_move(self):
        state=self.homes.snapshot()["arms"]["right"]
        self.assertTrue(state["can_clear_alarms"])
        self.assertFalse(state["can_return"])
        self.assertEqual((await self.run_reset())["state"],"ALARMS_CLEARED")
        self.assertEqual(self.methods.count("RecoveryClearError"),1)
        self.assertLess(self.methods.index("Stop"),self.methods.index("RecoveryClearError"))
        self.assertEqual(self.methods[-1],"GetErrorID")
        self.assertTrue(self.homes.snapshot()["arms"]["right"]["can_return"])

    async def test_estop_rejects_reset(self):
        self.info["emergencyStop"]=True
        self.assertEqual((await self.run_reset())["state"],"ERROR")
        self.assertNotIn("RecoveryClearError",self.methods)

    async def test_persistent_alarm_is_reported_once(self):
        self.persistent=True
        result=await self.run_reset()
        self.assertEqual(result["state"],"ERROR")
        self.assertIn("3, 5",result["reason"])
        self.assertEqual(self.methods.count("RecoveryClearError"),1)
