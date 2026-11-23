"""One trigger stream per hand. Reflected states are references; a PWM acknowledgement is not jaw feedback."""
import asyncio
import json
import math
import time
from pathlib import Path
from asset_registry import load_catalog, asset_path
from gripper_kinematics import ParallelJaw, finite_unit
from gripper_uno import UnoChannel, settings

SIDES=("left","right")
MAX_AGE=.2
# Near the released stop, small analog noise must not turn full-open into a new squeeze.
OPEN_TRIGGER = .02
LEAVE_OPEN_TRIGGER = .05
class Grippers:
    def __init__(self,root,output_settings=None,channel_factory=UnoChannel):
        self.catalog=load_catalog(root); self.arms={};self.channels={};self.task=None
        self.config_error=None
        try:
            cfg=settings(root) if output_settings is None else output_settings
        except (OSError, ValueError, TypeError, AttributeError) as exc:
            self.config_error=str(exc)[:160];cfg={}
        self.direct = {side:config for side,config in cfg.items() if config.independent_trigger}
        self.direct_sessions = {}
        self.start_open = {}
        for side in SIDES:
            asset=self.catalog["instances"][side]["tool"];desc=self.catalog["assets"][asset]
            model=ParallelJaw(json.loads(asset_path(root,asset,desc["mechanism"]).read_text(encoding="utf-8")))
            self.arms[side]={"model":model,"asset":asset,"phase":"WAIT_RELEASE","context":None,"last_seq":None,
                             "last_open":None,"pose":model.pose(0),"raw":None,"reason":"WAITING","permit":False,
                             "lease":0,"hardware_released":False,"rearm_required":False,"log_at":0,"log_mark":None,"log_gap":None,
                             "open_latched":False,"last_logged_measurement":None}
            if side in cfg:self.channels[side]=channel_factory(side,cfg[side])

    def set_independent(self, side, context=None):
        if side not in self.direct:
            raise ValueError("Independent trigger is not configured for this side")
        if context is not None:
            self.direct_sessions[side] = context
            return
        self.direct_sessions.pop(side, None)
        self.start_open.pop(side,None)
        self.channels[side].offer(None,None,None,None)
        self.arms[side].update(permit=False, hardware_released=False, phase="WAIT_RELEASE")

    def stop_independent(self):
        for side in tuple(self.direct_sessions):
            self.set_independent(side)

    def model_pose(self, side, opening):
        model = self.arms[side]["model"]
        cfg=self.direct.get(side)
        return model.pose_at_crank_fraction(opening) if cfg and not cfg.position_pi else model.pose(opening)

    async def open_for_start(self, participants, quest_provider, starting):
        """A fresh open acknowledgement precedes the arm's follow permission."""
        sides = [s for s in participants if s in self.direct]
        if not sides:
            return
        q = quest_provider()
        frame = q.get("frame") or {}
        context = (q.get("session"),frame.get("reference_epoch"))
        seq = frame.get("seq")
        if not context[0] or type(context[1]) is not int or type(seq) is not int:
            raise ValueError("缺少当前手柄输入，未启动夹爪")
        began = time.monotonic()
        for side in sides:
            if self.channels[side].snapshot().get("status")=="FAULT_REARM_REQUIRED":
                self.set_independent(side)
            self.set_independent(side,context)
            self.start_open[side] = "WAIT_OPEN_ACK"
        try:
            acknowledged=set()
            while True:
                for side in sides:
                    if not starting() or self.direct_sessions.get(side)!=context:
                        raise ValueError("夹爪启动已取消或追踪失效")
                    drive=self.channels[side].snapshot()
                    if (drive.get("status")=="FAULT_REARM_REQUIRED" and self.arms[side]["permit"] and
                            drive.get("fault_lease")==self.arms[side]["lease"]):
                        raise ValueError(f"{side} Uno 未确认全开：{drive.get('error')}")
                    if (drive.get("status")=="PWM_ACK" and drive.get("ack_us")==self.direct[side].open_us and
                            drive.get("ack_lease")==self.arms[side]["lease"] and
                            (drive.get("ack_at") or 0)>=began and (drive.get("ack_seq") or 0)>seq):
                        acknowledged.add(side)
                if len(acknowledged)==len(sides):
                    self.start_open.update(dict.fromkeys(sides,"WAIT_TRIGGER_RELEASE"))
                    return
                if time.monotonic()-began>5:
                    raise ValueError("未收到当前启动请求的夹爪全开 ACK；请检查 Uno 连接")
                await asyncio.sleep(.01)
        except BaseException:
            for side in sides:
                self.set_independent(side)
            raise

    def tick(self,quest,permits=(),now=None,record=None,inhibited=False,owners=None):
        now=time.monotonic() if now is None else now
        f=quest.get("frame") or {};t=f.get("host_monotonic_s");ctx=(quest.get("session"),f.get("reference_epoch"))
        fresh=(quest.get("connected") is True and type(t) in (int,float) and 0<=now-t<MAX_AGE
               and f.get("visibility")=="visible" and f.get("input_policy")=="continuous-pose-grip-v2"
               and f.get("gripper_policy")=="analog-trigger-ui-v1")
        for side in SIDES:
            a=self.arms[side];hand=(f.get("hands") or {}).get(side) or {};gp=hand.get("gamepad") or {};buttons=gp.get("buttons") or []
            trigger=buttons[0].get("value") if buttons and isinstance(buttons[0],dict) else None
            valid=(fresh and bool(ctx[0]) and type(ctx[1]) is int and hand.get("is_controller") is True and hand.get("tracking_valid") is True
                   and (hand.get("pose") or {}).get("emulated_position") is False and gp.get("mapping")=="xr-standard"
                   and gp.get("connected") is True and finite_unit(trigger) and type(hand.get("gripper_ui_blocked")) is bool)
            active_permit=side in permits and not inhibited
            owner=side in (permits if owners is None else owners)
            if side in self.direct:
                if side in self.direct_sessions and (not valid or not active_permit or
                        self.direct_sessions[side]!=ctx or
                        (f.get("diagnostics") or {}).get("requested_mode")!="immersive-ar"):
                    self.set_independent(side)
                owner=side in self.direct_sessions
                active_permit=owner
            if owner and not a["permit"]:
                a["lease"]+=1;a["hardware_released"]=False;a["rearm_required"]=False
                a["open_latched"]=True
            if owner and a["permit"] and a["hardware_released"] and (not active_permit or not valid or a["context"]!=ctx):
                a["rearm_required"]=True
            if not owner:
                a["hardware_released"]=False;a["rearm_required"]=False
            a["permit"]=owner
            if a["context"]!=ctx:
                a.update(context=ctx,phase="WAIT_RELEASE",last_seq=None,hardware_released=False)
            if not valid or inhibited:
                a.update(phase="WAIT_RELEASE",raw=None,hardware_released=False,open_latched=False,reason="INHIBITED" if inhibited else "NO_FRESH_INPUT")
            else:
                a["raw"]=float(trigger)
                if self.start_open.get(side)=="WAIT_TRIGGER_RELEASE" and trigger<=.05 and not hand["gripper_ui_blocked"]:
                    self.start_open[side]="WAIT_NEW_PULL"
                elif self.start_open.get(side)=="WAIT_NEW_PULL":
                    if hand["gripper_ui_blocked"]:
                        self.start_open[side]="WAIT_TRIGGER_RELEASE"
                    elif trigger>.05:
                        self.start_open.pop(side)
                        a["phase"]="TRACK"
                if side in self.direct and active_permit and not a["hardware_released"] and not hand["gripper_ui_blocked"]:
                    a["phase"]="WAIT_RELEASE"
                if side in self.start_open:
                    # The Start click owns the opening action, even if its ray is still on the HUD.
                    a.update(last_open=1.0,hardware_released=True,reason=self.start_open[side])
                    a["pose"]=self.model_pose(side,1.0)
                    a["open_latched"]=True
                elif hand["gripper_ui_blocked"]:
                    a.update(phase="UI_RELEASE",reason="HUD_PRIORITY")
                elif a["phase"]=="UI_RELEASE":
                    if trigger<=.05:a["phase"]="NEW_PRESS"
                    a["reason"]="WAIT_NEW_TRIGGER"
                elif a["phase"]=="NEW_PRESS" and trigger<=.05:
                    a["reason"]="WAIT_NEW_TRIGGER"
                elif a["phase"]=="WAIT_RELEASE" and trigger>.05:
                    a["reason"]="RELEASE_TRIGGER"
                else:
                    a["phase"]="TRACK";a["reason"]="TRACKING"
                    opening=1-float(trigger)
                    if side in self.direct:
                        if trigger<=OPEN_TRIGGER:
                            a["open_latched"]=True
                        elif trigger>LEAVE_OPEN_TRIGGER:
                            a["open_latched"]=False
                        if a["open_latched"]:
                            opening=1.0
                    if a["last_open"]!=opening:
                        a["pose"]=self.model_pose(side, opening);a["last_open"]=opening
                    if active_permit and trigger<=.05:a["hardware_released"]=True
                a["last_seq"]=f.get("seq")
            channel=self.channels.get(side)
            if channel:
                if valid and active_permit and a["hardware_released"] and not a["rearm_required"] and a["last_open"] is not None:
                    fraction = a["last_open"] if side in self.direct else a["pose"]["crank_fraction"]
                    neutral = side in self.start_open or (a["reason"]=="TRACKING" and trigger<=.05)
                    channel.offer(a["lease"],a["last_seq"],fraction,t+MAX_AGE,neutral)
                else:channel.offer(None,None,None,None)
            if record:
                st=self.state(side);mark=(a["reason"],st["drive"]["status"]);g=st["target_open_fraction"]
                measured=st["measured_gap_mm"]
                moved=measured is not None and (a["last_logged_measurement"] is None or abs(measured-a["last_logged_measurement"])>=.5)
                if mark!=a["log_mark"] or (now-a["log_at"]>=.1 and g is not None and (moved or a["log_gap"] is None or abs(g-a["log_gap"])>=.015)):
                    record("gripper_reference",{"side":side,"asset_revision":self.catalog["revision"],"seq":a["last_seq"],"reference_epoch":ctx[1],
                                                 "trigger":a["raw"],"target_open_fraction":g,"model_gap_mm":st["model_gap_mm"],"reason":a["reason"],"drive":st["drive"],
                                                 "target_gap_mm":st["target_gap_mm"],"measured_gap_mm":measured})
                    a.update(log_mark=mark,log_gap=g,log_at=now,last_logged_measurement=measured)

    def state(self,side):
        a=self.arms[side];ch=self.channels.get(side)
        target_us = self.direct[side].pulse_for(a["last_open"]) if side in self.direct and a["last_open"] is not None else None
        pi=self.direct[side].position_pi if side in self.direct else None
        drive=ch.snapshot() if ch else {"mode":"blocked" if self.config_error else "virtual",
            "status":"CONFIG_ERROR" if self.config_error else "VIRTUAL_ONLY","error":self.config_error,"ack_us":None}
        measured=None
        if (pi and drive.get("status")=="PWM_ACK" and drive.get("ack_lease")==a["lease"] and a["permit"] and
                side in self.direct_sessions and 0<=time.monotonic()-(drive.get("ack_at") or 0)<MAX_AGE):
            measured=drive.get("measured_gap_mm")
        elif pi:
            drive={**drive,"measured_gap_mm":None,"feedback_adc":None,"applied_pwm_us":None}
        return {"asset":a["asset"],"input_seq":a["last_seq"],"trigger":a["raw"],"reason":a["reason"],"hardware_rearm_required":bool(ch and a["rearm_required"]),
                "independent_trigger":side in self.direct,"independent_enabled":side in self.direct_sessions,"target_pulse_us":target_us,
                "target_open_fraction":a["last_open"],"model_gap_mm":a["pose"]["model_gap_mm"] if a["last_open"] is not None else None,
                "position_pi_configured":pi is not None,"target_gap_mm":pi.gap(a["last_open"]) if pi and a["last_open"] is not None else None,
                "model_pose":a["pose"],"measured_gap_mm":measured,"drive":drive}

    def snapshot(self):
        return {"schema":1,"asset_revision":self.catalog["revision"],"arms":{s:self.state(s) for s in SIDES}}

    def preview(self,triggers):
        out={}
        for s in SIDES:
            t=triggers[s]
            if not finite_unit(t): raise ValueError("trigger must be in [0,1]")
            p=self.model_pose(s,1-t)
            out[s]={"asset":self.arms[s]["asset"],"reason":"OFFLINE_PREVIEW","trigger":t,"target_open_fraction":1-t,
                    "model_gap_mm":p["model_gap_mm"],"model_pose":p,"measured_gap_mm":None,"drive":{"mode":"virtual","status":"PREVIEW","ack_us":None}}
        return {"schema":1,"asset_revision":self.catalog["revision"],"arms":out}

    async def close(self):
        if self.task:
            self.task.cancel()
            try:await self.task
            except asyncio.CancelledError:pass
        await asyncio.gather(*(asyncio.to_thread(ch.close) for ch in self.channels.values()))
