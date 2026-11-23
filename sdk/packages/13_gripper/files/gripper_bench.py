"""Windows bench commissioning of ONE unloaded Uno gripper. No E6 SDK client or robot connection."""
import argparse
import json
import time
from pathlib import Path
from gripper_uno import PortSettings, UnoWire, settings

ROOT=Path(__file__).resolve().parent

def main():
    cli=argparse.ArgumentParser(description=__doc__)
    sub=cli.add_subparsers(dest="action",required=True)
    for name in ("ports","init","check"):
        sub.add_parser(name)
    for name in ("identify","adc","jog"):
        p=sub.add_parser(name)
        p.add_argument("--side",choices=["left","right"],required=True)
        p.add_argument("--port",required=True)
        if name=="jog":
            p.add_argument("--start-us",required=True,type=int,help="Independently established unloaded start pulse")
            p.add_argument("--limit-us",required=True,type=int,help="Operator-chosen single-direction guard")
            p.add_argument("--step-us",default=5,type=int)
            p.add_argument("--slew-us-s",default=100,type=int)
    args=cli.parse_args()
    if args.action=="init":
        path=ROOT/"runtime/grippers.local.json";path.parent.mkdir(exist_ok=True)
        try:
            with path.open("x",encoding="utf8") as f:f.write((ROOT/"grippers.example.json").read_text("utf8"))
            print("Created",path)
        except FileExistsError:print("Kept existing",path)
        return
    if args.action=="check":
        print(json.dumps({"configured_sides":sorted(settings(ROOT)), "serial_opened":False}));return
    import serial
    from serial.tools import list_ports
    if args.action=="ports":
        for p in list_ports.comports():print(p.device,p.description)
        return
    if args.action=="jog":
        if not (1000<=args.start_us<=2000 and 1000<=args.limit_us<=2000 and
                abs(args.start_us-args.limit_us)>=20 and 1<=args.step_us<=10 and 20<=args.slew_us_s<=500):
            cli.error("Use a verified 1000..2000 us interval at least 20 us wide, step 1..10 us, slew 20..500 us/s")
        import msvcrt
        print("Unloaded bench only. Remove the test object. Confirm the E6 is stationary, the SDK is stopped, and servo power can be cut.")
        print("First pulse:",args.start_us,"us. Travel guard:",args.limit_us,"us. There is no position or force sensor.")
        if input("Type JOG "+args.side.upper()+" to enable this side: ").strip()!="JOG "+args.side.upper():
            return
    with serial.Serial(args.port,115200,timeout=.18,write_timeout=.1) as port:
        time.sleep(1.8);port.reset_input_buffer();wire=UnoWire(port,args.side)
        try:
            wire.identify();print("Confirmed E6G1",args.side,"; servo output off.")
            if args.action=="identify":return
            if args.action=="adc":
                port.write(b"ADC?\n")
                raw=port.readline().decode("ascii",errors="replace").split()
                if len(raw)!=2 or raw[0]!="ADC" or not raw[1].isdigit() or int(raw[1])>1023:
                    raise OSError("This board did not return a calibrated-input ADC reading")
                print("Raw feedback ADC",raw[1]);return
            cfg=PortSettings(args.port,args.limit_us,args.start_us,args.slew_us_s)
            target=args.start_us;direction=1 if args.limit_us>args.start_us else -1
            lower,upper=sorted((args.start_us,args.limit_us))
            wire.arm(cfg);wire.set_pwm(target)
            print("+: one step toward limit, -: one step back, k: keep enabled, space/q: OFF. Four seconds idle -> OFF.")
            active=time.monotonic()
            while time.monotonic()-active<4:
                if msvcrt.kbhit():
                    key=msvcrt.getwch().lower()
                    if key in ("q"," ","\x03"):break
                    if key in ("+","=","-","k"):
                        active=time.monotonic()
                        if key!="k":
                            target=max(lower,min(upper,target+direction*args.step_us*(-1 if key=="-" else 1)))
                            print("Requested pulse",target,"us",flush=True)
                wire.set_pwm(target);time.sleep(.045)
        finally:
            wire.off()
    print("Pulse generation disabled. Measure the physical opening independently.")

if __name__=="__main__":main()

