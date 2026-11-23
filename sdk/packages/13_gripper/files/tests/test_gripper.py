"""Offline fixtures for the new signal path. These inputs are not robot or headset measurements."""
import asyncio
import json
import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch
from aiohttp.test_utils import TestClient, TestServer
from gripper_control import Grippers
from gripper_uno import settings, PortSettings, PositionPI, UnoWire
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
    def test_release_overrides_incomplete_closure_without_grip(self):
        import time
        pi=PositionPI(100,900,0.,60.,120,50,60)
        for mode in (None,pi):
            cfg=PortSettings("OFFLINE",400,1300,0,True,mode)
            worker=Mock()
            reply={"status":"PWM_ACK","ack_lease":1,"ack_at":time.monotonic(),"measured_gap_mm":30. if mode else None}
            worker.snapshot.side_effect=lambda:dict(reply)
            g=Grippers(ROOT,{"right":cfg},lambda *_:worker)
            g.set_independent("right",("offline",1))
            for seq,(raw,target) in enumerate([(0,1300),(.5,850),(1,400),(.015,1300),(.04,1300),(.06,1246)],1):
                f=message(seq,None,raw);f["host_monotonic_s"]=time.monotonic()
                g.tick({"frame":f,"session":"offline","connected":True},permits={"right"},owners=())
                st=g.state("right")
                self.assertEqual(st["target_pulse_us"],target)
                self.assertEqual(cfg.pulse_for(worker.offer.call_args.args[2]),target)
                self.assertEqual(st["measured_gap_mm"],30. if mode else None)
                if mode and target==1300:self.assertEqual(st["target_gap_mm"],60.)
            reply["ack_at"]=time.monotonic()-1
            self.assertIsNone(g.state("right")["measured_gap_mm"])
            g.tick({"connected":False})
            self.assertIsNone(worker.offer.call_args.args[0])

    def test_pi_requires_real_calibration_and_matching_ack(self):
        pi_data=dict(enabled=True,open_adc=100,closed_adc=900,open_gap_mm=60.,closed_gap_mm=0.,
                     kp_us=120,ki_us_s=50,max_trim_us=60)
        self.assertIsNone(PositionPI.from_config(None,900))
        with self.assertRaises(ValueError): PositionPI.from_config({**pi_data,"closed_adc":100},900)
        pi=PositionPI.from_config(pi_data,900)
        port=Mock(); wire=UnoWire(port,"right",7)
        port.readline.return_value=b"ERR\n"
        with self.assertRaises(OSError): wire.arm(PortSettings("TEST",400,1300,0,True,pi))
        port.readline.return_value=b"ARMEDPI 7\n";wire.arm(PortSettings("TEST",400,1300,0,True,pi))
        port.readline.return_value=b"ACKPI 7 1 850 880 500\n"
        self.assertEqual(wire.set_pwm(850)["measured_gap_mm"],30.)
        port.readline.return_value=b"ACKPI 7 2 1300 1299 500\n"
        with self.assertRaises(OSError): wire.set_pwm(1300)
        port.readline.return_value=b"ACKPI 7 3 850 850 0\n"
        with self.assertRaises(OSError): wire.set_pwm(850)

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

    def test_calibrated_pulse_below_1000_requires_per_arm_opt_in(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "runtime").mkdir()
            path = root / "runtime/grippers.local.json"
            calibrated = {"enabled": True, "port": "COM11", "open_us": 1300,
                          "closed_us": 400, "slew_us_s": 400}

            def write(arm):
                path.write_text(json.dumps({"schema": 1, "arms": {"right": arm}}))

            write(calibrated)
            with self.assertRaises(ValueError):
                settings(root)
            write({**calibrated, "min_pulse_us": 400})
            cfg = settings(root)["right"]
            self.assertEqual((cfg.port, cfg.open_us, cfg.closed_us), ("COM11", 1300, 400))
            write({**calibrated, "min_pulse_us": 400, "closed_us": 399})
            with self.assertRaises(ValueError):
                settings(root)

class WebIntegration(unittest.IsolatedAsyncioTestCase):
    async def test_start_opens_before_follow_then_waits_for_a_new_trigger_pull(self):
        import time
        from teleop_group import TeleopGroup
        cfg=PortSettings("TEST",400,1300,0,True)
        channel=Mock()
        drive={"status":"PWM_ACK","ack_us":1300,"ack_at":time.monotonic(),"ack_seq":8,"ack_lease":0}
        channel.snapshot.side_effect=lambda: dict(drive)
        grippers=Grippers(ROOT,{"right":cfg},lambda *_:channel)
        group=TeleopGroup(on_stop=grippers.stop_independent)
        trial=Mock(state="PREPARED",armed=False)
        async def arm():
            trial.state,trial.armed="READY",True
        async def disarm():
            trial.state,trial.armed="DISARMED",False
        trial.arm=arm
        trial.disarm=disarm
        trial.assert_group_ready=lambda: self.assertTrue(trial.armed and trial.state=="READY")
        trial.fault_stop=lambda reason: disarm()
        trial.snapshot.return_value={"stop_evidence":{"feedback_confirmed":True}}
        group.controllers["right"]=trial
        group.set_prepared(("right",))
        q={"connected":True,"session":"offline-start-test"}
        def frame(seq,trigger,ui):
            f=message(seq,None,trigger)
            f["host_monotonic_s"]=time.monotonic()
            f["hands"]["right"]["gripper_ui_blocked"]=ui
            q["frame"]=f
            grippers.tick(q,permits={"right"},owners=())
        frame(10,1,True)
        start=asyncio.create_task(group.arm(before_follow=lambda:grippers.open_for_start(
            ("right",),lambda:q,lambda:group.state=="ARMING")))
        try:
            for _ in range(20):
                await asyncio.sleep(.005)
                if start.done(): await start
                if grippers.start_open:break
            frame(11,1,True)
            lease,_,fraction,_,allow_first=channel.offer.call_args.args
            self.assertEqual((fraction,allow_first),(1.,True))
            self.assertFalse(group.motion_allowed("right"))
            self.assertFalse(start.done())
            drive.update(ack_lease=lease,ack_seq=11,ack_at=time.monotonic(),ack_us=850)
            await asyncio.sleep(.025)
            self.assertFalse(start.done())
            drive.update(ack_us=1300,ack_at=time.monotonic())
            await asyncio.wait_for(start,.5)
            self.assertTrue(group.motion_allowed("right"))
            for seq,t,ui,pulse in ((12,1,True,1300),(13,.7,False,1300),(14,0,False,1300),
                                   (15,.01,False,1300),(16,.5,False,850)):
                frame(seq,t,ui)
                self.assertEqual(cfg.pulse_for(channel.offer.call_args.args[2]),pulse)
            await group.stop()
            self.assertFalse(grippers.direct_sessions)
        finally:
            if not start.done():
                start.cancel()
                await asyncio.gather(start,return_exceptions=True)
            await group.close()

    async def test_independent_right_trigger_without_robot_arm_or_grip(self):
        import time
        cfg = PortSettings("EMULATED",400,1300,0,True)
        channel = Mock()
        channel.snapshot.return_value = {"mode":"uno","status":"TEST_CHANNEL"}
        channel.close.return_value = None
        grippers = Grippers(ROOT, {"right":cfg}, lambda side,config: channel)
        mode = 5
        def feedback(monitor,side,reader=None):
            return {"side":side,"connected":side=="right","valid":side=="right" and mode in (4,5,7),
                    "host_monotonic_s":time.monotonic(),"robot_mode":mode,"error_status":0}
        with tempfile.TemporaryDirectory() as tmp, patch("server.Grippers",return_value=grippers), \
                patch("server.RobotMonitor.snapshot",feedback), \
                patch("dobot_safe_sdk.BoundDobotDashboard",side_effect=AssertionError("Unexpected E6 command")):
            app=server.make_app(recording_enabled=False,recordings_root=Path(tmp))
            async with TestClient(TestServer(app)) as client:
                h={"Host":f"127.0.0.1:{server.PORT}"}
                token=(await (await client.get("/api/session",headers=h)).json())["token"]
                async with client.ws_connect("/ws/quest?token="+token,headers=h) as ws:
                    seq=0
                    async def frame(value,ui=False,grip=0):
                        nonlocal seq
                        seq+=1
                        f=message(seq,None,value)
                        hand=f["hands"]["right"]
                        hand["gripper_ui_blocked"]=ui
                        hand["grip_held"]=bool(grip)
                        hand["gamepad"]["buttons"][1].update(value=grip,pressed=bool(grip))
                        await ws.send_json(f)
                        for _ in range(20):
                            await asyncio.sleep(.005)
                            if grippers.state("right")["input_seq"]==seq: return
                        self.fail("Right frame did not reach the gripper")
                    async def action(name):
                        await ws.send_json({"type":"control_action","action":name,"id":name+str(seq)})
                        while True:
                            reply=await ws.receive_json(timeout=1)
                            if reply.get("type")=="action_result":return reply
                    await frame(1,True)  # Click captured by the HUD
                    self.assertFalse(grippers.state("right")["independent_enabled"])
                    self.assertTrue((await action("gripper_enable_right"))["ok"])
                    await frame(0)
                    for raw,expect,grip in [(0,1300,0),(.5,850,0),(1,400,0),(.25,1075,1)]:
                        await frame(raw,grip=grip)
                        v=grippers.state("right")
                        self.assertEqual(v["target_pulse_us"],expect)
                        self.assertEqual(cfg.pulse_for(channel.offer.call_args.args[2]),expect)
                    self.assertEqual(v["model_pose"]["crank_fraction"],.75)
                    await frame(1,True)
                    self.assertEqual(grippers.state("right")["target_pulse_us"],1075)
                    self.assertTrue((await action("stop"))["ok"])
                    self.assertFalse(grippers.direct_sessions)
                    await frame(0)
                    self.assertTrue((await action("gripper_enable_right"))["ok"])
                    await frame(0)
                    mode=11
                    await frame(.25)
                    self.assertFalse(grippers.direct_sessions)
                    self.assertIsNone(channel.offer.call_args.args[0])
                    mode=5
                    await frame(0)
                    self.assertTrue((await action("gripper_enable_right"))["ok"])
                self.assertFalse(grippers.direct_sessions)

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

