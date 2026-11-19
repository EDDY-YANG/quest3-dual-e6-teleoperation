"""Offline fixtures for the new signal path. These inputs are not robot or headset measurements."""
import asyncio
import json
import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from aiohttp.test_utils import TestClient, TestServer
from gripper_control import Grippers
from gripper_uno import settings
from asset_registry import load_catalog
import server

ROOT=Path(__file__).resolve().parents[1]

def message(seq,left,right,blocked=False):
    def hand(v,b):
        if v is None:return None
        return {"is_controller":True,"tracking_valid":True,"grip_held":False,"gripper_ui_blocked":b,
                "pose":{"position_m":[0,1,0],"quaternion_xyzw":[0,0,0,1],"emulated_position":False},
                "gamepad":{"mapping":"xr-standard","connected":True,"axes":[],"buttons":[
                    {"pressed":x>.7,"touched":x>0,"value":x} for x in (v,0)]}}
    return {"type":"frame","schema":1,"seq":seq,"reference_space":"local-floor","reference_epoch":1,
            "xr_frame_time_ms":seq*33,"client_send_time_ms":seq*33,"visibility":"visible",
            "input_policy":"continuous-pose-grip-v2","gripper_policy":"analog-trigger-ui-v1",
            "diagnostics":{"requested_mode":"immersive-ar"},"hands":{"left":hand(left,blocked),"right":hand(right,False)}}

class Geometry(unittest.TestCase):
    def test_distal_gap_linear_and_rod_endpoints_stay_on_pins(self):
        g=Grippers(ROOT,{});m=g.arms["left"]["model"]
        self.assertAlmostEqual(math.hypot(m.cx,m.cy),25.,places=4)
        for i in range(101):
            a=i/100;p=m.pose(a);self.assertAlmostEqual(p["model_gap_mm"],2*m.travel*a+m.closed_gap,places=6)
            co,si=math.cos(p["rod_rad"]),math.sin(p["rod_rad"])
            x,y=m.sx,m.sy
            self.assertAlmostEqual(co*x-si*y+p["rod_offset_mm"][0],m.sx-p["slide_mm"],places=6)
            self.assertAlmostEqual(si*x+co*y+p["rod_offset_mm"][1],m.sy,places=6)
            cx=m.cx*math.cos(p["crank_rad"])-m.cy*math.sin(p["crank_rad"])
            cy=m.cx*math.sin(p["crank_rad"])+m.cy*math.cos(p["crank_rad"])
            self.assertAlmostEqual(math.hypot(x-p["slide_mm"]-cx,y-cy),m.rod,places=6)
        self.assertTrue(58.5<m.pose(1)["model_gap_mm"]<58.8)
        for bad in (-1,1.01,float("nan"),True):
            with self.assertRaises(ValueError):m.pose(bad)

    def test_catalog_mounts_and_unmodified_per_arm_instances(self):
        c=load_catalog(ROOT)
        for s in ("left","right"):
            a=c["instances"][s]
            self.assertEqual((a['arm'],a['controller'],a['tool']),('e6','quest-touch-plus','gripper-v1'))
            self.assertEqual(a['mount']['parent'],'Link6')
            self.assertAlmostEqual(a['mount']['xyz_m'][2],.019)
        g=Grippers(ROOT,{})
        for a,b in ((0,1),(.5,1),(1,0)):
            v=g.preview({'left':a,'right':b})['arms']
            self.assertAlmostEqual(v['left']['target_open_fraction'],1-a)
            self.assertAlmostEqual(v['right']['target_open_fraction'],1-b)
            self.assertIsNone(v['left']['measured_gap_mm'])
        self.assertIsNone(g.state('left')['target_open_fraction'])

    def test_ui_and_stale_do_not_turn_a_click_or_lost_pose_into_motion(self):
        g=Grippers(ROOT,{})
        def tick(n,raw,ui=False,delay=0):
            f=message(n,raw,0,ui);f["host_monotonic_s"]=n
            g.tick({"frame":f,"session":"test","connected":True},now=n+delay)
            return g.state('left')
        self.assertEqual(tick(1,.7)['reason'],'RELEASE_TRIGGER')
        self.assertEqual(tick(2,0)['target_open_fraction'],1)
        self.assertAlmostEqual(tick(3,.6)['target_open_fraction'],.4)
        self.assertAlmostEqual(tick(4,1,True)['target_open_fraction'],.4)
        self.assertAlmostEqual(tick(5,.9)['target_open_fraction'],.4)
        self.assertAlmostEqual(tick(6,0)['target_open_fraction'],.4)
        self.assertAlmostEqual(tick(7,.3)['target_open_fraction'],.7)
        self.assertIsNone(tick(8,.7,delay=.3)['trigger'])
        self.assertEqual(tick(9,.8)['reason'],'RELEASE_TRIGGER')
    def test_duplicate_ports_and_unknown_endpoints_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'runtime').mkdir();f=root/'runtime/grippers.local.json'
            a={"enabled":True,"port":"COM88","closed_us":1250,"open_us":1700,"slew_us_s":100}
            f.write_text(json.dumps({"schema":1,"arms":{"left":a,"right":a}}))
            with self.assertRaises(ValueError):settings(root)
            f.write_text(json.dumps({"schema":1,"arms":{"right":{**a,"closed_us":None}}}))
            with self.assertRaises(ValueError):settings(root)

class WebIntegration(unittest.IsolatedAsyncioTestCase):
    async def test_actual_ws_to_both_display_endpoints_and_pure_preview(self):
        with tempfile.TemporaryDirectory() as tmp,patch("gripper_control.settings",return_value={}),patch("dobot_safe_sdk.BoundDobotDashboard",side_effect=AssertionError("No robot I/O")):
            app=server.make_app(recording_enabled=False,recordings_root=Path(tmp))
            async with TestClient(TestServer(app)) as client:
                headers={"Host":f"127.0.0.1:{server.PORT}"}
                response=await client.get("/api/session",headers=headers);token=(await response.json())["token"]
                async with client.ws_connect("/ws/quest?token="+token,headers=headers) as ws:
                    for n,(l,r) in enumerate(((0,0),(.25,.75)),1):
                        await ws.send_json(message(n,l,r))
                        for _ in range(20):
                            d=await (await client.get("/api/monitor/status",headers=headers)).json()
                            if d['grippers']['arms']['left']['input_seq']==n:break
                            await asyncio.sleep(.01)
                    for route in ("/api/headset/status","/api/monitor/status","/api/status"):
                        d=await (await client.get(route,headers=headers)).json();arms=d["grippers"]["arms"]
                        self.assertAlmostEqual(arms["left"]["target_open_fraction"],.75)
                        self.assertAlmostEqual(arms["right"]["target_open_fraction"],.25)
                        self.assertEqual(arms["left"]["drive"]["mode"],"virtual")
                        self.assertIsNone(arms["left"]["measured_gap_mm"])
                    preview=await (await client.get("/api/grippers/preview?left=1&right=1",headers=headers)).json()
                    self.assertEqual(preview['arms']['left']['target_open_fraction'],0)
                    live=await (await client.get("/api/headset/status",headers=headers)).json()
                    self.assertEqual(live['grippers']['arms']['left']['target_open_fraction'],.75)
                await asyncio.sleep(.06)
                stale=await (await client.get("/api/headset/status",headers=headers)).json()
                self.assertIsNone(stale["grippers"]["arms"]["left"]["trigger"])
                for path in ("/api/assets","/api/assets/gripper-v1/gripper.glb","/api/assets/quest-touch-plus/left.glb","/api/assets/e6/Link6.STL"):
                    resp=await client.get(path,headers=headers);self.assertEqual(resp.status,200);await resp.read()
                self.assertEqual((await client.get("/api/assets/gripper-v1/grippers.local.json",headers=headers)).status,404)
        self.assertFalse(list(Path(tmp).glob("*.jsonl")))

