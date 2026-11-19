"""Test the Python production wire driver against actual AVR firmwares running in AVR8js."""
import argparse,json,os,subprocess,sys
import time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from gripper_uno import UnoWire,PortSettings,UnoChannel
from gripper_control import Grippers

class EmulatedUart:
    def __init__(self,image,module):
        env=dict(os.environ)
        if module:env["AVR8JS_PATH"]=str(module.resolve())
        self.p=subprocess.Popen(["node",str(Path(__file__).with_name("gripper_avr_uart.cjs")),str(image.resolve())],
              stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,env=env)
    def request(self,data):
        self.p.stdin.write(json.dumps(data)+"\n");self.p.stdin.flush()
        return json.loads(self.p.stdout.readline())
    def write(self,data):
        self.reply=self.request({"line":data.decode("ascii").strip()})["reply"]
    def readline(self):return (self.reply+"\n").encode("ascii")
    def reset_input_buffer(self):self.reply=""
    def close(self):
        self.p.stdin.close()
        if self.p.wait(timeout=10)!=0:raise RuntimeError("AVR emulator failed")

def verify(side,image,module):
    uart=EmulatedUart(image,module)
    try:
        assert not uart.request({"ms":50})["pulses"]
        wire=UnoWire(uart,side,nonce=1999999999);wire.identify()
        other="R" if side=="left" else "L"
        assert uart.request({"line":f"ARM {other} 11 1300 1700 800"})["reply"]=="ERR"
        cfg=PortSettings("TEST",1300,1700,800);wire.arm(cfg)
        assert not uart.request({"ms":30})["pulses"]
        wire.set_pwm(cfg.open_us)
        open_pulses=uart.request({"ms":80})["pulses"]
        assert len(open_pulses)>=2 and abs(open_pulses[-1]["width_us"]-cfg.open_us)<9
        grippers=Grippers(Path(__file__).resolve().parents[1],{})
        results=[]
        for trigger in (0.5,1.0):
            pose=grippers.preview({"left":trigger,"right":trigger})["arms"][side]["model_pose"]
            target=round(cfg.closed_us+pose["crank_fraction"]*(cfg.open_us-cfg.closed_us))
            for _ in range(8):
                wire.set_pwm(target)
                pulses=uart.request({"ms":70})["pulses"]
            assert pulses and abs(pulses[-1]["width_us"]-target)<9,(side,trigger,target,pulses)
            results.append({"trigger":trigger,"command_us":target,"emulated_D9_us":pulses[-1]["width_us"]})
        after=uart.request({"ms":430})
        assert all(p["at_us"]<after["now_us"]-80000 for p in after["pulses"])
        assert uart.request({"line":f"SET {wire.nonce} 100 1300"})["reply"]=="ERR"
        wire.identify();wire.arm(cfg);wire.set_pwm(cfg.open_us)
        assert uart.request({"line":f"SET {wire.nonce} {wire.seq} {cfg.open_us}"})["reply"]=="ERR"
        wire.off();assert not uart.request({"ms":50})["pulses"]
        return {"side":side,"open_pulse_us":open_pulses[-1]["width_us"],"graded":results,"stale_and_wrong_side_rejected":True,"watchdog_pulses_stopped":True}
    finally:uart.close()
def integrated(left,right,module):
    from server import QuestState
    q=QuestState();q.session="isolated-software-test";q.connected=True
    images={"left":left,"right":right}
    config={"left":PortSettings("EMULATED-L",1300,1700,800),"right":PortSettings("EMULATED-R",1740,1320,800)}
    def factory(side,cfg):
        return UnoChannel(side,cfg,boot_delay=0,port_factory=lambda *_args,**_kwargs:EmulatedUart(images[side],module))
    g=Grippers(Path(__file__).resolve().parents[1],config,factory);sequence=0
    def accept(l,r):
        nonlocal sequence
        now=time.monotonic();sequence+=1
        def hand(t):
            return {"is_controller":True,"tracking_valid":True,"grip_held":False,"gripper_ui_blocked":False,
                    "pose":{"position_m":[0,1,0],"quaternion_xyzw":[0,0,0,1],"emulated_position":False},
                    "gamepad":{"connected":True,"mapping":"xr-standard","axes":[],"buttons":[{"value":v,"pressed":v>.7,"touched":v>0} for v in (t,0)]}}
        q.accept({"type":"frame","schema":1,"seq":sequence,"reference_epoch":1,"reference_space":"local-floor","visibility":"visible",
                  "input_policy":"continuous-pose-grip-v2","gripper_policy":"analog-trigger-ui-v1","xr_frame_time_ms":sequence*30,
                  "client_send_time_ms":sequence*30,"hands":{"left":hand(l),"right":hand(r)}},received=now)
        g.tick(q.snapshot(now),permits=("left","right"),now=now)
    try:
        for values,expected in [((0,0),(1700,1320)),((.5,1),(1528,1740))]:
            until=time.monotonic()+5
            while time.monotonic()<until:
                accept(*values);time.sleep(.03)
                ack=tuple(g.state(s)["drive"].get("ack_us") for s in ("left","right"))
                if ack==expected:break
            assert ack==expected,(expected,ack,[g.state(s)["drive"] for s in ("left","right")])
        q.connected=False;g.tick(q.snapshot(),permits=("left","right"));time.sleep(.15)
        assert all(g.state(s)["hardware_rearm_required"] for s in ("left","right"))
        return {"scope":"validated input frames -> real backend and both serial workers -> two emulated compiled Unos",
                "last_ack_us":list(ack),"tracking_loss_latched":True,"physical_hardware":False}
    finally:
        for channel in g.channels.values():channel.close()

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("--left",type=Path,required=True);p.add_argument("--right",type=Path,required=True);p.add_argument("--avr8js-path",type=Path)
    a=p.parse_args()
    print(json.dumps({"boards":[verify("left",a.left,a.avr8js_path),verify("right",a.right,a.avr8js_path)],
                      "integration":integrated(a.left,a.right,a.avr8js_path)},indent=2))

