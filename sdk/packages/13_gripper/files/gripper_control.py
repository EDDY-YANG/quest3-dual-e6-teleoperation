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
class Grippers:
    def __init__(self,root,output_settings=None,channel_factory=UnoChannel):
        self.catalog=load_catalog(root); self.arms={};self.channels={};self.task=None
        self.config_error=None
        try:
            cfg=settings(root) if output_settings is None else output_settings
        except (OSError, ValueError, TypeError, AttributeError) as exc:
            self.config_error=str(exc)[:160];cfg={}
        for side in SIDES:
            asset=self.catalog["instances"][side]["tool"];desc=self.catalog["assets"][asset]
            model=ParallelJaw(json.loads(asset_path(root,asset,desc["mechanism"]).read_text(encoding="utf-8")))
            self.arms[side]={"model":model,"asset":asset,"phase":"WAIT_RELEASE","context":None,"last_seq":None,
                             "last_open":None,"pose":model.pose(0),"raw":None,"reason":"WAITING","permit":False,
                             "lease":0,"hardware_released":False,"rearm_required":False,"log_at":0,"log_mark":None,"log_gap":None}
            if side in cfg:self.channels[side]=channel_factory(side,cfg[side])

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
            if owner and not a["permit"]:
                a["lease"]+=1;a["hardware_released"]=False;a["rearm_required"]=False
            if owner and a["permit"] and a["hardware_released"] and (not active_permit or not valid or a["context"]!=ctx):
                a["rearm_required"]=True
            if not owner:
                a["hardware_released"]=False;a["rearm_required"]=False
            a["permit"]=owner
            if a["context"]!=ctx:
                a.update(context=ctx,phase="WAIT_RELEASE",last_seq=None,hardware_released=False)
            if not valid or inhibited:
                a.update(phase="WAIT_RELEASE",raw=None,hardware_released=False,reason="INHIBITED" if inhibited else "NO_FRESH_INPUT")
            else:
                a["raw"]=float(trigger)
                if hand["gripper_ui_blocked"]:
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
                    if a["last_open"]!=opening:
                        a["pose"]=a["model"].pose(opening);a["last_open"]=opening
                    if active_permit and trigger<=.05:a["hardware_released"]=True
                a["last_seq"]=f.get("seq")
            channel=self.channels.get(side)
            if channel:
                if valid and active_permit and a["hardware_released"] and not a["rearm_required"] and a["last_open"] is not None:
                    channel.offer(a["lease"],a["last_seq"],a["pose"]["crank_fraction"],t+MAX_AGE,a["reason"]=="TRACKING" and trigger<=.05)
                else:channel.offer(None,None,None,None)
            if record:
                st=self.state(side);mark=(a["reason"],st["drive"]["status"]);g=st["target_open_fraction"]
                if mark!=a["log_mark"] or (now-a["log_at"]>=.1 and g is not None and (a["log_gap"] is None or abs(g-a["log_gap"])>=.015)):
                    record("gripper_reference",{"side":side,"asset_revision":self.catalog["revision"],"seq":a["last_seq"],"reference_epoch":ctx[1],
                                                 "trigger":a["raw"],"target_open_fraction":g,"model_gap_mm":st["model_gap_mm"],"reason":a["reason"],"drive":st["drive"],"measured_gap_mm":None})
                    a.update(log_mark=mark,log_gap=g,log_at=now)

    def state(self,side):
        a=self.arms[side];ch=self.channels.get(side)
        return {"asset":a["asset"],"input_seq":a["last_seq"],"trigger":a["raw"],"reason":a["reason"],"hardware_rearm_required":bool(ch and a["rearm_required"]),
                "target_open_fraction":a["last_open"],"model_gap_mm":a["pose"]["model_gap_mm"] if a["last_open"] is not None else None,
                "model_pose":a["pose"],"measured_gap_mm":None,"drive":ch.snapshot() if ch else
                {"mode":"blocked" if self.config_error else "virtual","status":"CONFIG_ERROR" if self.config_error else "VIRTUAL_ONLY",
                 "error":self.config_error,"ack_us":None}}

    def snapshot(self):
        return {"schema":1,"asset_revision":self.catalog["revision"],"arms":{s:self.state(s) for s in SIDES}}

    def preview(self,triggers):
        out={}
        for s in SIDES:
            t=triggers[s]
            if not finite_unit(t): raise ValueError("trigger must be in [0,1]")
            p=self.arms[s]["model"].pose(1-t)
            out[s]={"asset":self.arms[s]["asset"],"reason":"OFFLINE_PREVIEW","trigger":t,"target_open_fraction":1-t,
                    "model_gap_mm":p["model_gap_mm"],"model_pose":p,"measured_gap_mm":None,"drive":{"mode":"virtual","status":"PREVIEW","ack_us":None}}
        return {"schema":1,"asset_revision":self.catalog["revision"],"arms":out}

    async def close(self):
        if self.task:
            self.task.cancel()
            try:await self.task
            except asyncio.CancelledError:pass
        await asyncio.gather(*(asyncio.to_thread(ch.close) for ch in self.channels.values()))
