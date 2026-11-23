"""Two independent, optional Uno serial owners. No COM probing and no automatic retry of a failed lease."""
import json
import math
import re
import secrets
import threading
import time
from dataclasses import dataclass
from typing import Optional
from pathlib import Path

@dataclass(frozen=True)
class PositionPI:
    closed_adc: int
    open_adc: int
    closed_gap_mm: float
    open_gap_mm: float
    kp_us: int
    ki_us_s: int
    max_trim_us: int

    @classmethod
    def from_config(cls, data, span):
        if data is None:
            return None
        if not isinstance(data, dict) or type(data.get("enabled")) is not bool:
            raise ValueError("position_pi requires an explicit enabled flag")
        if not data["enabled"]:
            return None
        a,b=(data.get(k) for k in ("closed_adc","open_adc"))
        if not all(type(v) is int and 1<=v<=1022 for v in (a,b)) or a==b:
            raise ValueError("PI needs distinct measured closed_adc/open_adc inside the ADC rails")
        c,d=(data.get(k) for k in ("closed_gap_mm","open_gap_mm"))
        if not all(type(v) in (int,float) and math.isfinite(v) for v in (c,d)) or not 0<=c<d:
            raise ValueError("PI needs measured closed/open jaw gaps in mm")
        kp,ki,trim=(data.get(k) for k in ("kp_us","ki_us_s","max_trim_us"))
        if (not all(type(v) is int for v in (kp,ki,trim)) or not 1<=kp<=32767 or
                not 0<=ki<=32767 or not 1<=trim<=span):
            raise ValueError("Invalid PI gains or correction bound")
        return cls(a,b,c,d,kp,ki,trim)

    def gap(self, fraction):
        return self.closed_gap_mm+fraction*(self.open_gap_mm-self.closed_gap_mm)

    def measured_gap(self, adc):
        margin=2+abs(self.open_adc-self.closed_adc)//50
        if not 0<adc<1023 or not min(self.closed_adc,self.open_adc)-margin<=adc<=max(self.closed_adc,self.open_adc)+margin:
            raise OSError("Position feedback outside calibrated sensor range")
        return self.gap((adc-self.closed_adc)/(self.open_adc-self.closed_adc))

@dataclass(frozen=True)
class PortSettings:
    port: str
    closed_us: int
    open_us: int
    slew_us_s: int
    independent_trigger: bool = False
    position_pi: Optional[PositionPI] = None

    def pulse_for(self, fraction):
        return round(self.closed_us + fraction * (self.open_us - self.closed_us))

def settings(root):
    file=Path(root)/"runtime/grippers.local.json"
    if not file.is_file():return {}
    data=json.loads(file.read_text(encoding="utf-8-sig"))
    if data.get("schema")!=1 or not isinstance(data.get("arms"),dict) or set(data["arms"])-{"left","right"}:raise ValueError("Invalid gripper configuration")
    result={};ports=set()
    for side,s in data["arms"].items():
        if s.get("enabled") is not True:continue
        port=s.get("port");closed=s.get("closed_us");opened=s.get("open_us");speed=s.get("slew_us_s")
        minimum = s.get("min_pulse_us", 1000)
        if not isinstance(port,str) or not re.fullmatch(r"COM[1-9][0-9]{0,2}",port,re.I) or port.upper() in ports:
            raise ValueError("Each Uno needs its own explicit COM port")
        if type(minimum) is not int or not 400 <= minimum <= 1000:
            raise ValueError("min_pulse_us must be an integer from 400 to 1000")
        if not all(type(v) is int and minimum <= v <= 2000 for v in (closed, opened)) or abs(closed-opened)<20:
            raise ValueError(f"Enter separately calibrated open/closed pulses ({minimum}..2000 us)")
        independent = s.get("independent_trigger", False)
        if type(independent) is not bool:
            raise ValueError("independent_trigger must be true or false")
        if type(speed) is not int or not (20 <= speed <= 1000 or (independent and speed == 0)):
            raise ValueError("Set a slew limit of 20..1000 us/s, or 0 for direct independent trigger control")
        pi=PositionPI.from_config(s.get("position_pi"),abs(opened-closed))
        if pi and (not independent or speed!=0):
            raise ValueError("Position PI requires independent_trigger and slew_us_s=0")
        ports.add(port.upper());result[side]=PortSettings(port.upper(),closed,opened,speed,independent,pi)
    return result

class UnoWire:
    def __init__(self,port,side,nonce=None):
        self.port=port;self.side="L" if side=="left" else "R"
        self.nonce=nonce if nonce is not None else secrets.randbelow(2000000000)+1
        self.seq=0
        self.config=None

    def exchange(self,command,expected):
        self.port.write((command+"\n").encode("ascii"))
        reply=self.port.readline().decode("ascii",errors="replace").strip()
        if reply!=expected:raise OSError("UNO reply timeout or mismatch for "+command.split()[0])

    def identify(self):
        # USB reset timing varies. Only the no-output identity query is repeatable.
        expected=f"E6G1 {self.side}"
        deadline=time.monotonic()+.9
        while True:
            self.port.write(b"ID?\n")
            reply=self.port.readline().decode("ascii",errors="replace").strip()
            if reply==expected:
                return
            if reply or time.monotonic()>=deadline:
                raise OSError(f"Expected {expected}, received {reply or 'no board reply'}")

    def arm(self,config):
        pi=config.position_pi
        if pi:
            self.exchange(f"ARMPI {self.side} {self.nonce} {config.closed_us} {config.open_us} "
                          f"{pi.closed_adc} {pi.open_adc} {pi.kp_us} {pi.ki_us_s} {pi.max_trim_us}",f"ARMEDPI {self.nonce}")
        else:
            self.exchange(f"ARM {self.side} {self.nonce} {config.closed_us} {config.open_us} {config.slew_us_s}",f"ARMED {self.nonce}")
        self.config=config

    def set_pwm(self,pulse):
        self.seq+=1
        cmd=f"SET {self.nonce} {self.seq} {pulse}"
        if not self.config or not self.config.position_pi:
            self.exchange(cmd,f"ACK {self.nonce} {self.seq} {pulse}")
            return {}
        self.port.write((cmd+"\n").encode("ascii"))
        reply=self.port.readline().decode("ascii",errors="replace").split()
        if len(reply)!=6 or reply[:4]!=["ACKPI",str(self.nonce),str(self.seq),str(pulse)]:
            raise OSError("Position PI setpoint or sensor acknowledgement failed")
        applied,adc=map(int,reply[4:])
        low,high=sorted((self.config.closed_us,self.config.open_us))
        if not low<=applied<=high or abs(applied-pulse)>self.config.position_pi.max_trim_us or (pulse in (low,high) and applied!=pulse):
            raise OSError("Position PI output outside calibrated pulse limits")
        return {"applied_pwm_us":applied,"feedback_adc":adc,"measured_gap_mm":self.config.position_pi.measured_gap(adc)}

    def off(self):
        self.port.write(b"OFF\n")

class UnoChannel:
    """Latest reference wins. Blocking serial operations never run in the XR/control event loop."""
    def __init__(self,side,config,*,port_factory=None,boot_delay=1.8):
        self.side=side;self.config=config;self.lock=threading.Lock();self.quit=threading.Event()
        self.wake=threading.Event()
        self.latest=None;self.thread=None;self.failed_lease=None;self.stream=0
        self.port_factory=port_factory;self.boot_delay=boot_delay
        self.info={"mode":"uno","status":"DISARMED","ack_us":None,"ack_seq":None,"ack_at":None}

    def offer(self,lease,seq,crank_fraction,expires,neutral=False):
        with self.lock:
            if lease is None:
                if self.latest is not None:
                    self.latest=None
                    self.wake.set()
                return
            if self.latest is None or self.latest[0]!=lease:self.stream+=1
            pulse=self.config.pulse_for(crank_fraction)
            previous=self.latest
            self.latest=(lease,self.stream,seq,pulse,expires,neutral)
            if previous is None or previous[:4]!=self.latest[:4] or previous[5]!=neutral:
                self.wake.set()
        if self.thread is None:
            self.thread=threading.Thread(target=self._run,daemon=True,name="gripper-uno-"+self.side);self.thread.start()

    def pending(self):
        with self.lock:v=self.latest
        return v if v and v[4]>time.monotonic() and v[0]!=self.failed_lease else None

    def state(self,**kwargs):
        if kwargs.get("status")!="PWM_ACK":
            kwargs.update(measured_gap_mm=None,feedback_adc=None,applied_pwm_us=None)
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
                self.wake.clear()
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
                    feedback=wire.set_pwm(pulse)
                    self.state(status="PWM_ACK",error=None,ack_us=pulse,ack_seq=fresh[2],
                               ack_lease=fresh[0],ack_at=time.monotonic(),**feedback)
                    if self.config.independent_trigger:
                        self.wake.wait(.08)  # New frames wake immediately; idle frames keep the Uno watchdog alive.
                    else:
                        self.quit.wait(.045)
                except (ImportError,OSError,ValueError) as exc:
                    self.failed_lease=demand[0];close()
                    self.state(status="FAULT_REARM_REQUIRED",error=str(exc)[:150],fault_lease=demand[0],ack_us=None,ack_seq=None,ack_at=None)
                    self.quit.wait(.05)
        finally:close()

    def close(self):
        self.quit.set()
        self.wake.set()
        if self.thread:self.thread.join(timeout=2.5)
