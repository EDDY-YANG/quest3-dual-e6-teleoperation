"""Two independent, optional Uno serial owners. No COM probing and no automatic retry of a failed lease."""
import json
import re
import secrets
import threading
import time
from dataclasses import dataclass
from pathlib import Path

@dataclass(frozen=True)
class PortSettings:
    port: str
    closed_us: int
    open_us: int
    slew_us_s: int

def settings(root):
    file=Path(root)/"runtime/grippers.local.json"
    if not file.is_file():return {}
    data=json.loads(file.read_text(encoding="utf-8-sig"))
    if data.get("schema")!=1 or not isinstance(data.get("arms"),dict) or set(data["arms"])-{"left","right"}:raise ValueError("Invalid gripper configuration")
    result={};ports=set()
    for side,s in data["arms"].items():
        if s.get("enabled") is not True:continue
        port=s.get("port");closed=s.get("closed_us");opened=s.get("open_us");speed=s.get("slew_us_s")
        if not isinstance(port,str) or not re.fullmatch(r"COM[1-9][0-9]{0,2}",port,re.I) or port.upper() in ports:
            raise ValueError("Each Uno needs its own explicit COM port")
        if not all(type(v) is int and 1000<=v<=2000 for v in (closed,opened)) or abs(closed-opened)<20:
            raise ValueError("Enter separately calibrated open/closed pulses (1000..2000 us)")
        if type(speed) is not int or not 20<=speed<=1000:raise ValueError("Set a bench-verified slew limit in us/s")
        ports.add(port.upper());result[side]=PortSettings(port.upper(),closed,opened,speed)
    return result

class UnoWire:
    def __init__(self,port,side,nonce=None):
        self.port=port;self.side="L" if side=="left" else "R"
        self.nonce=nonce if nonce is not None else secrets.randbelow(2000000000)+1
        self.seq=0

    def exchange(self,command,expected):
        self.port.write((command+"\n").encode("ascii"))
        reply=self.port.readline().decode("ascii",errors="replace").strip()
        if reply!=expected:raise OSError("UNO reply timeout or mismatch for "+command.split()[0])

    def identify(self):
        self.exchange("ID?",f"E6G1 {self.side}")

    def arm(self,config):
        self.exchange(f"ARM {self.side} {self.nonce} {config.closed_us} {config.open_us} {config.slew_us_s}",f"ARMED {self.nonce}")

    def set_pwm(self,pulse):
        self.seq+=1
        self.exchange(f"SET {self.nonce} {self.seq} {pulse}",f"ACK {self.nonce} {self.seq} {pulse}")

    def off(self):
        self.port.write(b"OFF\n")

class UnoChannel:
    """Latest reference wins. Blocking serial operations never run in the XR/control event loop."""
    def __init__(self,side,config,*,port_factory=None,boot_delay=1.8):
        self.side=side;self.config=config;self.lock=threading.Lock();self.quit=threading.Event()
        self.latest=None;self.thread=None;self.failed_lease=None;self.stream=0
        self.port_factory=port_factory;self.boot_delay=boot_delay
        self.info={"mode":"uno","status":"DISARMED","ack_us":None,"ack_seq":None,"ack_at":None}

    def offer(self,lease,seq,crank_fraction,expires,neutral=False):
        with self.lock:
            if lease is None:
                self.latest=None
                return
            if self.latest is None or self.latest[0]!=lease:self.stream+=1
            pulse=round(self.config.closed_us+crank_fraction*(self.config.open_us-self.config.closed_us))
            self.latest=(lease,self.stream,seq,pulse,expires,neutral)
        if self.thread is None:
            self.thread=threading.Thread(target=self._run,daemon=True,name="gripper-uno-"+self.side);self.thread.start()

    def pending(self):
        with self.lock:v=self.latest
        return v if v and v[4]>time.monotonic() and v[0]!=self.failed_lease else None

    def state(self,**kwargs):
        with self.lock:self.info.update(kwargs)

    def snapshot(self):
        with self.lock:return dict(self.info)

    def _run(self):
        port=wire=key=None
        driving=False
        def close():
            nonlocal port,wire,key,driving
            if port is not None:
                try:
                    if wire:wire.off()
                except OSError:pass
                finally:
                    try:port.close()
                    except OSError:pass
            port=wire=key=None
            driving=False
        try:
            while not self.quit.is_set():
                demand=self.pending()
                if not demand or (key and demand[:2]!=key):
                    if port is not None:
                        close(); self.state(status="DISARMED",ack_us=None,ack_seq=None,ack_at=None)
                    self.quit.wait(.015);continue
                try:
                    if port is None:
                        self.state(status="CONNECTING",ack_us=None,ack_seq=None,ack_at=None)
                        factory=self.port_factory
                        if factory is None:
                            from serial import Serial
                            factory=Serial
                        key=demand[:2];port=factory(self.config.port,115200,timeout=.15,write_timeout=.08)
                        wire=UnoWire(port,self.side)
                        if self.quit.wait(self.boot_delay):break  # Uno USB reset/bootloader
                        current=self.pending()
                        if not current or current[:2]!=key:continue
                        port.reset_input_buffer();wire.identify()
                    fresh=self.pending()
                    if not fresh or fresh[:2]!=key:continue
                    pulse=fresh[3]
                    if not driving:
                        if not fresh[5]:
                            self.state(status="RELEASE_TO_ENABLE");self.quit.wait(.02);continue
                        wire.arm(self.config)
                        fresh=self.pending()
                        if not fresh or fresh[:2]!=key or not fresh[5]:
                            close();continue
                        pulse=self.config.open_us
                        driving=True
                    wire.set_pwm(pulse);self.state(status="PWM_ACK",error=None,ack_us=pulse,ack_seq=fresh[2],ack_at=time.monotonic())
                    self.quit.wait(.045)
                except (ImportError,OSError,ValueError) as exc:
                    self.failed_lease=demand[0];close()
                    self.state(status="FAULT_REARM_REQUIRED",error=str(exc)[:150],ack_us=None,ack_seq=None,ack_at=None)
                    self.quit.wait(.05)
        finally:close()

    def close(self):
        self.quit.set()
        if self.thread:self.thread.join(timeout=2.5)
