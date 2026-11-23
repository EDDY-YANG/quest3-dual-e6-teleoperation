"""Measured or explicitly configured per-arm starts and joint return jobs.

No startup connection or movement. Configured targets are not measured captures.
Motion uses controller joint trajectories, not IK/ServoJ target teleportation.
"""
import asyncio
import ipaddress
import json
import math
import os
import re
import secrets
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

from tool_orientation_cone import tilt_deg, INITIAL_TILT_DEG, ANGLE_EPS_DEG
from kinematics_validation import pose_error
from ranged_ik_profile import E6_NOMINAL_ABSOLUTE_LIMIT_RAD, vector
from robot_stop_state import stop_feedback

SIDES = ("left", "right")
RETURN_SPEED_PERCENT = 100
RETURN_ACCEL_PERCENT = 30
SEND_BOUND_DEG = 50.0
SEGMENT_DEG = 45.0
START_TILT_DEG = INITIAL_TILT_DEG
ARRIVAL_DEG = 0.5
ARRIVAL_MM = 1.0
STABLE_SPAN_S = 0.15
WATCHDOG_S = 0.2
CLIENT_LEASE_S = 2.0


def six_payload(reply):
    return list(vector([float(x) for x in reply["payload"].split(",")], 6, "controller reply"))


def check_joints(q):
    q = vector(q, 6, "joints")
    if any(abs(x) > math.degrees(bound) + 1e-7 for x, bound in zip(q, E6_NOMINAL_ABSOLUTE_LIMIT_RAD)):
        raise ValueError("START_ABSOLUTE_JOINT_LIMIT")
    return list(q)


def down_angle(pose):
    # Explicit commissioning convention: level base, tool +Z towards base -Z.
    return tilt_deg(pose)


def joint_path(start, goal):
    start, goal = check_joints(start), check_joints(goal)
    steps = max(1, math.ceil(max(abs(a-b) for a, b in zip(start, goal)) / SEGMENT_DEG))
    return [[a+(b-a)*i/steps for a, b in zip(start, goal)] for i in range(1, steps+1)]


async def neighbor_identity(ip, source):
    """Read the neighbor on the explicitly selected Windows source interface."""
    ip, source = str(ipaddress.IPv4Address(ip)), str(ipaddress.IPv4Address(source))
    if os.name != "nt":
        raise ValueError("START_MAC_IDENTITY_REQUIRES_WINDOWS")
    script = (
        "$ErrorActionPreference='Stop';"
        f"$a=@(Get-NetIPAddress -AddressFamily IPv4 -IPAddress '{source}');"
        "if($a.Count -ne 1){throw 'Ambiguous source interface'};"
        f"$n=@(Get-NetNeighbor -InterfaceIndex $a[0].InterfaceIndex -IPAddress '{ip}' "
        "| Where-Object {$_.State -notin @('Unreachable','Incomplete')});"
        "if($n.Count -ne 1){throw 'No unique robot neighbor'};"
        "@{mac=$n[0].LinkLayerAddress;interface_index=$a[0].InterfaceIndex}|ConvertTo-Json -Compress"
    )
    process = await asyncio.create_subprocess_exec(
        "powershell.exe", "-NoProfile", "-NonInteractive", "-Command", script,
        stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE,
        creationflags=0x08000000,
    )
    try:
        stdout, _ = await asyncio.wait_for(process.communicate(), 4)
    except asyncio.TimeoutError:
        process.kill()
        await process.communicate()
        raise ValueError("START_MAC_IDENTITY_TIMEOUT") from None
    if process.returncode:
        raise ValueError("START_MAC_IDENTITY_UNAVAILABLE")
    value = json.loads(stdout.decode("utf-8-sig"))
    mac = str(value["mac"]).replace(":", "-").upper()
    if not re.fullmatch(r"(?:[0-9A-F]{2}-){5}[0-9A-F]{2}", mac) or mac in ("00-00-00-00-00-00", "FF-FF-FF-FF-FF-FF"):
        raise ValueError("START_MAC_IDENTITY_INVALID")
    return {"ip": ip, "source_ip": source, "mac": mac}


@dataclass
class ReturnChannel:
    feedback: object
    call: object
    identity: dict
    release: object
    connected: object = lambda: True
    verify_identity: object = None
    controller_status: object = None


def empty_alarm_ids(ids):
    if isinstance(ids,list):
        return all(empty_alarm_ids(i) for i in ids)
    return type(ids) is int and ids == 0


class StartPositions:
    def __init__(self, path, acquire, quiesce, feedback, owner_valid, event=None, *, presets_path=None):
        self.path, self.acquire, self.quiesce = Path(path), acquire, quiesce
        self.feedback, self.owner_valid = feedback, owner_valid
        self.event = event or (lambda *args: None)
        self.saved, self.load_error = {}, None
        self.presets, self.preset_error = {}, None
        self.active_side = self.operation_id = self.owner = self.source = None
        self.heartbeat_at = 0
        self.task = None
        self.channel = None
        self.cancel = asyncio.Event()
        self.fault_latched = False
        self.audit = None
        self.audit_path = None
        self.audit_error = None
        self.recover_collision = False
        self.clear_only = False
        self.states = {s: {"state": "NO_START", "reason": "尚未保存该臂开始位"} for s in SIDES}
        try:
            if self.path.exists():
                data = json.loads(self.path.read_text(encoding="utf-8"))
                if data.get("schema") != 1 or not isinstance(data.get("arms"), dict):
                    raise ValueError("START_STORE_SCHEMA_INVALID")
                for side, record in data["arms"].items():
                    self._validate_record(side, record)
                self.saved = data["arms"]
                for side in self.saved:
                    self.states[side] = {"state": "SAVED", "reason": "开始位已保存，等待连接核对"}
        except (ValueError, OSError, KeyError, TypeError) as exc:
            self.saved = {}
            self.load_error = str(exc)
        try:
            if presets_path is not None and Path(presets_path).exists():
                data = json.loads(Path(presets_path).read_text(encoding="utf-8"))
                if not isinstance(data, dict) or data.get("schema") != 1 or not isinstance(data.get("arms"), dict):
                    raise ValueError("START_PRESET_SCHEMA_INVALID")
                for side, record in data["arms"].items():
                    self._validate_binding(side, record)
                    if record.get("kind") != "CONFIGURED_JOINT_TARGET" or "pose_mm_deg" in record:
                        raise ValueError("START_PRESET_MUST_BE_EXPLICIT_JOINT_TARGET")
                identities = [r["identity"]["mac"] for r in data["arms"].values()]
                if len(identities) != len(set(identities)):
                    raise ValueError("START_SAME_ROBOT_ASSIGNED_TO_BOTH_ARMS")
                self.presets = data["arms"]
                for side in self.presets:
                    if side not in self.saved:
                        self.states[side] = {"state": "PRESET", "reason": "关节开始位预设已保存；等待回位前核对"}
        except (ValueError, OSError, KeyError, TypeError) as exc:
            self.presets = {}
            self.preset_error = str(exc)

    @staticmethod
    def _validate_binding(side, record):
        if side not in SIDES or not isinstance(record, dict) or record.get("side") != side:
            raise ValueError("START_SIDE_MISMATCH")
        check_joints(record["q_deg"])
        if record.get("user") != 0 or record.get("tool") != 0:
            raise ValueError("START_UNSUPPORTED_FRAME")
        for name in ("user_value_mm_deg", "tool_value_mm_deg"):
            vector(record[name], 6, name)
        identity = record["identity"]
        for name in ("ip", "source_ip"):
            ipaddress.IPv4Address(identity[name])
        if not re.fullmatch(r"(?:[0-9A-F]{2}-){5}[0-9A-F]{2}", identity["mac"]):
            raise ValueError("START_MAC_IDENTITY_INVALID")

    @staticmethod
    def _validate_record(side, record):
        StartPositions._validate_binding(side, record)
        vector(record["pose_mm_deg"], 6, "start pose")
        if down_angle(record["pose_mm_deg"]) > START_TILT_DEG:
            raise ValueError("START_TOOL_NOT_DOWN")

    def _target(self, side):
        # An explicit later measured capture replaces the active preset, while
        # the separately stored preset remains available as configuration.
        # Do not silently revert to a preset when that capture store is corrupt.
        if self.load_error:
            return None
        return self.saved.get(side) or self.presets.get(side)

    def _target_source(self, side):
        if self._target(side) is None:
            return None
        return "MEASURED_CAPTURE" if side in self.saved else "CONFIGURED_JOINT_TARGET" if side in self.presets else None

    @property
    def busy(self):
        return self.active_side is not None

    def _set(self, side, state, reason, **values):
        self.states[side] = {**self.states[side], "state": state, "reason": reason, **values}
        self.event("start_position", {"side": side, **self.states[side]})
        if self.audit is not None:
            self.audit["events"].append({"at": time.monotonic(), "side": side, "state": state, "reason": reason})

    async def _write_audit(self):
        if self.audit_path and self.audit is not None:
            path, payload = self.audit_path, json.dumps(self.audit, ensure_ascii=False, indent=2)+"\n"
            def write():
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(payload, encoding="utf-8")
            await asyncio.to_thread(write)

    def _packet(self, side, context=None, enabled=False):
        p = self.feedback(side)
        age = time.monotonic() - p.get("host_monotonic_s", -math.inf)
        if not p.get("connected") or not p.get("valid") or not 0 <= age <= WATCHDOG_S:
            raise ValueError("START_FEEDBACK_STALE_OR_INVALID")
        if p.get("robot_type") != 150 or p.get("user") != 0 or p.get("tool") != 0 or p.get("error_status") != 0:
            raise ValueError("START_ROBOT_OR_FRAME_INVALID")
        if p.get("robot_mode") not in ((5, 7) if enabled else (4, 5, 7)):
            raise ValueError("START_ROBOT_NOT_ENABLED_OR_USABLE")
        check_joints(p["joint_position_deg"])
        vector(p["pose_mm_deg"], 6, "feedback pose")
        actual = [p.get(k) for k in ("robot_id", "frame_id", "connection_generation")]
        if any(x is None for x in actual) or (context is not None and actual != context):
            raise ValueError("START_FEEDBACK_CONTEXT_CHANGED")
        return p, actual

    def _owner_guard(self):
        if self.cancel.is_set():
            raise ValueError("START_RETURN_CANCELLED")
        if self.source == "pc" and time.monotonic()-self.heartbeat_at > CLIENT_LEASE_S:
            raise ValueError("START_PC_HEARTBEAT_LOST")
        if not self.owner_valid(self.source, self.owner):
            raise ValueError("START_OWNER_INPUT_LOST")

    def _stop_packet(self, side, context=None):
        p = stop_feedback(self.feedback(side))
        actual = [p.get(k) for k in ("robot_id", "frame_id", "connection_generation")]
        if any(x is None for x in actual) or (context is not None and actual != context):
            raise ValueError("START_FEEDBACK_CONTEXT_CHANGED")
        return p, actual

    async def _stable(self, side, context, after, goal=None, pose=None, enabled=False, watch=False, timeout=3, allow_protected=False):
        deadline, samples = time.monotonic()+timeout, []
        while time.monotonic() < deadline:
            if watch:
                self._owner_guard()
            p, _ = self._stop_packet(side, context) if allow_protected else self._packet(side, context, enabled)
            stamp = p["host_monotonic_s"]
            idle = p["robot_mode"] in ((5,) if enabled else (4, 5, 11) if allow_protected else (4, 5))
            arrived = goal is None or max(abs(a-b) for a, b in zip(p["joint_position_deg"], goal)) <= ARRIVAL_DEG
            if pose is not None:
                ep, er = pose_error(p["pose_mm_deg"], pose)
                arrived = arrived and ep <= ARRIVAL_MM and er <= 1
            if stamp > after and idle and arrived:
                if not samples or stamp > samples[-1]["host_monotonic_s"]:
                    if samples and max(abs(a-b) for a, b in zip(p["joint_position_deg"], samples[0]["joint_position_deg"])) > .05:
                        samples = []
                    samples.append(p)
                    if len(samples) >= 3 and stamp-samples[0]["host_monotonic_s"] >= STABLE_SPAN_S:
                        return p
            else:
                samples = []
            await asyncio.sleep(.02)
        raise ValueError("START_IDLE_OR_ARRIVAL_NOT_CONFIRMED")

    async def _crosscheck(self, channel, side, context):
        q = six_payload(await channel.call("GetAngle"))
        pose = six_payload(await channel.call("GetPose", user=0, tool=0))
        fk = six_payload(await channel.call("PositiveKin", *q, user=0, tool=0))
        p, _ = self._packet(side, context)
        if max(abs(a-b) for a, b in zip(q, p["joint_position_deg"])) > .1:
            raise ValueError("START_ANGLE_FEEDBACK_MISMATCH")
        if any(a > b for a, b in zip(pose_error(pose, p["pose_mm_deg"]), (.1, .1))):
            raise ValueError("START_POSE_FEEDBACK_MISMATCH")
        if any(a > .05 for a in pose_error(fk, pose)):
            raise ValueError("START_CONTROLLER_FK_MISMATCH")
        return p

    async def capture(self, side):
        if side not in SIDES or self.busy or self.fault_latched:
            raise ValueError("START_OPERATION_BUSY_OR_FAULT")
        self.active_side = side
        self.audit = None
        channel = None
        self._set(side, "CAPTURING", "读取静止实测开始位")
        try:
            await self.quiesce(capture=True)
            channel = await self.acquire(side)
            _, context = self._packet(side)
            await self._stable(side, context, time.monotonic())
            p = await self._crosscheck(channel, side, context)
            if down_angle(p["pose_mm_deg"]) > START_TILT_DEG:
                raise ValueError("START_TOOL_NOT_DOWN: 工具+Z须朝基座-Z，偏差不超过5度")
            record = {k: p[k] for k in ("user", "tool", "user_value_mm_deg", "tool_value_mm_deg")}
            record.update(side=side, identity=channel.identity, q_deg=list(p["joint_position_deg"]),
                          pose_mm_deg=list(p["pose_mm_deg"]), saved_at=datetime.now(timezone.utc).isoformat(),
                          down_reference="BASE_MINUS_Z_LEVEL_BASE_REQUIRED")
            self._validate_record(side, record)
            updated = {**self.saved, side: record}
            if any(s != side and r["identity"]["mac"] == channel.identity["mac"]
                   for s, r in {**self.presets, **updated}.items()):
                raise ValueError("START_SAME_ROBOT_ASSIGNED_TO_BOTH_ARMS")
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.path.with_suffix(".tmp")
            temporary.write_text(json.dumps({"schema": 1, "arms": updated}, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
            temporary.replace(self.path)
            self.saved = updated
            self.load_error = None
            self._set(side, "SAVED", "已保存实测开始位")
        except Exception as exc:
            self._set(side, "ERROR", str(exc))
            raise
        finally:
            if channel:
                await channel.release()
            self.active_side = None
        return self.snapshot()

    def begin(self, side, source, owner, *, recover_collision=False, clear_only=False):
        if side not in SIDES or source not in ("pc", "vr") or not re.fullmatch(r"[A-Za-z0-9_-]{1,80}", owner or ""):
            raise ValueError("START_INVALID_REQUEST")
        if self.busy or self.fault_latched:
            raise ValueError("START_OPERATION_BUSY_OR_FAULT")
        record = self._target(side)
        if record is None:
            raise ValueError("START_NOT_SAVED")
        self.active_side, self.source, self.owner = side, source, owner
        self.recover_collision = recover_collision
        self.clear_only = clear_only
        self.operation_id = secrets.token_hex(12)
        self.audit_error = None
        self.audit = {"schema": 1, "operation_id": self.operation_id, "side": side, "source": source,
                      "saved_target": record, "target_source": self._target_source(side),
                      "collision_recovery_requested": recover_collision, "clear_only": clear_only, "events": [], "segments": []}
        self.audit_path = self.path.parent / "start_return_logs" / (
            datetime.now().strftime("%Y-%m-%d-%H-%M-%S")+"-"+side+"-"+self.operation_id+".json")
        self.heartbeat_at = time.monotonic()
        self.cancel = asyncio.Event()
        self._set(side, "PREPARING_RETURN", "正在暂停遥操作并核对开始位", stop_confirmed=None, progress=0)
        self.task = asyncio.create_task(self._run(side), name=f"return-start-{side}")
        return {"operation_id": self.operation_id, **self.snapshot()}

    def heartbeat(self, operation_id, owner):
        if self.busy and operation_id == self.operation_id and owner == self.owner and self.source == "pc":
            self.heartbeat_at = time.monotonic()
            return
        raise ValueError("START_HEARTBEAT_OWNER_MISMATCH")

    async def _clear_collision(self, channel, side, context):
        if channel.controller_status is None:
            raise ValueError("COLLISION_RECOVERY_STATUS_UNAVAILABLE")
        self._owner_guard()
        info = await channel.controller_status()
        if (info.get("remoteControl") != "tcp" or info.get("emergencyStop") is not False
                or info.get("isMotion") is not False or info.get("isCollision") != 1):
            raise ValueError("COLLISION_RECOVERY_REQUIRES_TCP_NO_ESTOP_AND_STATIONARY")
        alarms = json.loads((await channel.call("GetErrorID"))["payload"])
        if not isinstance(alarms,list) or not empty_alarm_ids(alarms):
            self._set(side, "ALARM_REVIEW", f"请先排除故障，当前报警 {alarms}", alarm_ids=alarms)
            raise ValueError(f"COLLISION_RECOVERY_OTHER_CONTROLLER_ALARM_REQUIRES_INSPECTION: {alarms}；排除原因后点击仅清警")
        check_joints(self.feedback(side)["joint_position_deg"])
        self.audit["collision_recovery"] = {"before": info, "error_ids": alarms, "attempts": 1}
        await self._write_audit()
        self._set(side, "RECOVERING", "正在清除本臂碰撞保护；日志保留")
        def clear_guard():
            self._owner_guard()
            current, _ = self._stop_packet(side, context)
            if current["robot_mode"] != 11:
                raise ValueError("COLLISION_RECOVERY_STATE_CHANGED")
        self.audit["collision_recovery"]["clear_reply"] = await channel.call("RecoveryClearError", guard=clear_guard)
        await channel.call("Stop")
        after, deadline = time.monotonic(), time.monotonic()+3
        # Clearing protection may restart feedback. Only before motion, after a
        # new Stop, bind a new generation of the same verified robot/frame.
        while True:
            self._owner_guard()
            try:
                p, recovered = self._packet(side)
                if recovered[:-1] != context[:-1]:
                    raise ValueError("COLLISION_RECOVERY_ROBOT_OR_FRAME_CHANGED")
                if p["host_monotonic_s"] > after and p["robot_mode"] in (4, 5):
                    break
            except ValueError as exc:
                if "ROBOT_OR_FRAME_CHANGED" in str(exc):
                    raise
            if time.monotonic() >= deadline:
                raise ValueError("COLLISION_RECOVERY_NOT_CLEARED")
            await asyncio.sleep(.02)
        await self._stable(side, recovered, after, watch=True)
        info_after = await channel.controller_status()
        if (info_after.get("isCollision") != 0 or info_after.get("emergencyStop") is not False
                or info_after.get("isMotion") is not False or info_after.get("remoteControl") != "tcp"):
            raise ValueError("COLLISION_RECOVERY_NOT_CLEARED")
        self.audit["collision_recovery"].update(after=info_after, feedback_context=recovered)
        await self._write_audit()
        return recovered

    def _alarm_packet(self, side):
        """Evidence for a reset request only. These packets cannot authorize motion."""
        p = self.feedback(side)
        age = time.monotonic() - p.get("host_monotonic_s", -math.inf)
        if (not p.get("connected") or p.get("packet_valid") is not True or not 0 <= age <= WATCHDOG_S or
                p.get("robot_type") != 150 or p.get("user") != 0 or p.get("tool") != 0 or
                p.get("robot_mode") not in (9, 11)):
            raise ValueError("清警需要新鲜的本臂报警反馈，请重新检查")
        check_joints(p["joint_position_deg"])
        context = tuple(p.get(k) for k in ("robot_id","frame_id","connection_generation"))
        if None in context:
            raise ValueError("清警反馈缺少机器人身份")
        return p, context

    async def _reset_alarms(self, side, channel):
        self._owner_guard()
        if channel.identity != self._target(side)["identity"] or not channel.controller_status or not channel.verify_identity:
            raise ValueError("清警设备身份未确认")
        await channel.verify_identity()
        initial, context = self._alarm_packet(side)
        def guard():
            self._owner_guard()
            packet, current = self._alarm_packet(side)
            if current != context or max(abs(a-b) for a,b in zip(packet["joint_position_deg"],initial["joint_position_deg"])) > .05:
                raise ValueError("清警期间机械臂状态变化")
        async def check_stationary_controller():
            info = await channel.controller_status()
            if (info.get("remoteControl")!="tcp" or info.get("emergencyStop") is not False or
                    info.get("isMotion") is not False or info.get("prjState")!="stopped" or
                    info.get("powerState")!="on"):
                raise ValueError("先检查急停、TCP模式、电源和程序停止状态")
        await check_stationary_controller()
        await channel.call("Stop",guard=guard)
        # Require distinct feedback after Stop, with the same six stationary joints.
        after = time.monotonic()
        await asyncio.sleep(.12)
        guard()
        if self.feedback(side)["host_monotonic_s"] <= after:
            raise ValueError("控制器停止后缺少新的反馈")
        before = json.loads((await channel.call("GetErrorID"))["payload"])
        if not isinstance(before, list):
            raise ValueError("控制器报警列表不可读")
        self.audit["alarm_reset"] = {"before":before, "attempts":1}
        await self._write_audit()
        await check_stationary_controller()
        await channel.verify_identity()
        self.audit["alarm_reset"]["reply"] = await channel.call("RecoveryClearError",guard=guard)
        cleared_at = time.monotonic()
        deadline = cleared_at+3
        after_ids = before
        while True:
            self._owner_guard()
            after_ids = json.loads((await channel.call("GetErrorID"))["payload"])
            self.audit["alarm_reset"]["after"] = after_ids
            if isinstance(after_ids,list) and empty_alarm_ids(after_ids):
                try:
                    packet, recovered = self._packet(side)
                except ValueError:
                    pass
                else:
                    if tuple(recovered[:2]) != context[:2]:
                        raise ValueError("清警后反馈身份发生变化")
                    if packet["robot_mode"] in (4,5) and packet["host_monotonic_s"]>cleared_at:
                        await check_stationary_controller()
                        self._set(side,"ALARMS_CLEARED","报警已清除且控制器已静止；确认工作区后再点回位",alarm_ids=[])
                        return
            if time.monotonic() >= deadline:
                raise ValueError(f"清警未通过复核；报警 {after_ids}，请查看控制器")
            await asyncio.sleep(.15)

    async def _run(self, side):
        channel = None
        context = None
        try:
            await self._write_audit()  # Verify local recording before any motion.
            self._owner_guard()
            if self.clear_only:
                await self.quiesce(capture=True)
                channel = await self.acquire(side)
                self.channel = channel
                await self._reset_alarms(side,channel)
                return
            if self.recover_collision:
                await self.quiesce(capture=False, recover=True)
            else:
                await self.quiesce(capture=False)
            self._owner_guard()
            channel = await self.acquire(side)
            self.channel = channel
            _, context = self._stop_packet(side) if self.recover_collision else self._packet(side)
            record = self._target(side)
            if record["identity"] != channel.identity:
                raise ValueError("START_ROBOT_IDENTITY_CHANGED")
            if any(s != side and self._target(s) and self._target(s)["identity"]["mac"] == channel.identity["mac"]
                   for s in SIDES):
                raise ValueError("START_SAME_ROBOT_ASSIGNED_TO_BOTH_ARMS")
            # Clear previously queued movement before taking ownership of return.
            await channel.call("Stop", guard=self._owner_guard)
            stationary = await self._stable(side, context, time.monotonic(), watch=True,
                                            allow_protected=self.recover_collision)
            if self.recover_collision and stationary["robot_mode"] == 11:
                context = await self._clear_collision(channel, side, context)
            p = await self._crosscheck(channel, side, context)
            self.audit["measured_start"] = p
            for field in ("user_value_mm_deg", "tool_value_mm_deg"):
                if max(abs(a-b) for a, b in zip(record[field], p[field])) > 1e-6:
                    raise ValueError("START_TOOL_OR_USER_CALIBRATION_CHANGED")
            goal = check_joints(record["q_deg"])
            target_fk = six_payload(await channel.call("PositiveKin", *goal, user=0, tool=0))
            if down_angle(target_fk) > START_TILT_DEG:
                raise ValueError("START_TARGET_FK_NOT_DOWN")
            if side in self.saved and any(a > .1 for a in pose_error(target_fk, record["pose_mm_deg"])):
                raise ValueError("START_SAVED_FK_CHANGED")
            self.audit["resolved_target"] = {"q_deg": goal, "pose_mm_deg": target_fk,
                                             "pose_source": "CONTROLLER_POSITIVE_KIN_NOT_MEASURED_ARRIVAL"}
            await self._write_audit()
            self._owner_guard()
            if p["robot_mode"] == 4:
                def enable_guard():
                    self._owner_guard()
                    latest, _ = self._packet(side, context)
                    if latest["robot_mode"] != 4:
                        raise ValueError("START_ENABLE_CONTEXT_CHANGED")
                self._set(side, "ENABLING", "回位请求已核对；正在使能本臂")
                self.audit["enable_requested"] = True
                await channel.call("EnableRobot", guard=enable_guard)
                await self._stable(side, context, time.monotonic(), enabled=True, watch=True)
                p = await self._crosscheck(channel, side, context)
                for field in ("user_value_mm_deg", "tool_value_mm_deg"):
                    if max(abs(a-b) for a, b in zip(record[field], p[field])) > 1e-6:
                        raise ValueError("START_TOOL_OR_USER_CALIBRATION_CHANGED")
                self.audit["enabled_feedback"] = p
                await self._write_audit()
            self._owner_guard()
            path = joint_path(p["joint_position_deg"], goal)
            baseline = int((await channel.call("GetCurrentCommandID"))["payload"])
            # Explicit user-requested full speed ratio; acceleration is separate.
            await channel.call("SpeedFactor", RETURN_SPEED_PERCENT, guard=self._owner_guard)
            for index, target in enumerate(path):
                def send_guard():
                    self._owner_guard()
                    latest, _ = self._packet(side, context, enabled=True)
                    if latest["robot_mode"] != 5:
                        raise ValueError("START_PREVIOUS_SEGMENT_NOT_IDLE")
                    if max(abs(a-b) for a, b in zip(target, latest["joint_position_deg"])) > SEND_BOUND_DEG:
                        raise ValueError("START_SEND_EXCEEDS_50_DEG")
                self._set(side, "RETURNING", "关节轨迹回开始位；途中不保证TCP朝下",
                          segment=index+1, segments=len(path), progress=index/len(path))
                reply = await channel.call("MovJ", *target, coordinateMode=1,
                                           a=RETURN_ACCEL_PERCENT, v=RETURN_SPEED_PERCENT, cp=0, guard=send_guard)
                command_id = int(reply["payload"])
                if command_id <= baseline:
                    raise ValueError("START_QUEUE_ID_NOT_ADVANCING")
                baseline = command_id
                ack_at = time.monotonic()
                deadline = ack_at+30
                while True:
                    self._owner_guard()
                    self._packet(side, context, enabled=True)
                    current = int((await channel.call("GetCurrentCommandID"))["payload"])
                    if current >= command_id:
                        break
                    if time.monotonic() >= deadline:
                        raise ValueError("START_QUEUE_PROGRESS_TIMEOUT")
                    await asyncio.sleep(.05)
                arrived = await self._stable(side, context, ack_at, goal=target,
                                   pose=target_fk if index == len(path)-1 else None,
                                   enabled=True, watch=True, timeout=max(.1, deadline-time.monotonic()))
                self.audit["segments"].append({"target_q_deg": target, "command_id": command_id,
                                               "ack_at": ack_at, "arrival_feedback": arrived})
            self.audit["final_tool_tilt_deg"] = down_angle(arrived["pose_mm_deg"])
            if self.audit["final_tool_tilt_deg"] > START_TILT_DEG + ANGLE_EPS_DEG:
                raise ValueError("START_FINAL_TILT_EXCEEDS_5_DEG")
            self._set(side, "AT_START", "已实测确认回到开始位；重检后重新开始遥操作",
                      progress=1, stop_confirmed=True)
        except Exception as exc:
            self._set(side, "STOPPING", str(exc))
            if channel and context:
                try:
                    await self._stop_channel(channel, side, context)
                    self._set(side, "STOPPED" if self.cancel.is_set() else "ERROR", str(exc), stop_confirmed=True)
                except Exception as stop_exc:
                    self.fault_latched = True
                    self._set(side, "FAULT", f"{exc}; STOP_NOT_CONFIRMED: {stop_exc}", stop_confirmed=False)
            else:
                self._set(side, "ERROR", str(exc), stop_confirmed=None)
        finally:
            self.audit["result"] = dict(self.states[side])
            try:
                await self._write_audit()
            except OSError as exc:
                self.audit_error = str(exc)
            if channel and not self.fault_latched:
                await channel.release()
                self.channel = None
            self.active_side = None

    async def _stop_channel(self, channel, side, context, *, explicit_recovery=False):
        method = "Stop"
        if explicit_recovery and not channel.connected():
            if channel.verify_identity is not None:
                await channel.verify_identity()
            method = "RecoverStop"
        await channel.call(method)
        await self._stable(side, context, time.monotonic(), allow_protected=True)

    async def stop(self):
        self.cancel.set()
        if self.task and not self.task.done():
            await self.task
        if self.fault_latched and self.channel:
            side = next(s for s in SIDES if self.states[s]["state"] == "FAULT")
            _, context = self._stop_packet(side)
            await self._stop_channel(self.channel, side, context, explicit_recovery=True)
            await self.channel.release()
            self.channel = None
            self.fault_latched = False
            self._set(side, "STOPPED", "停止已由实测确认", stop_confirmed=True)
            if self.audit is not None:
                self.audit["result"] = dict(self.states[side])
                try:
                    await self._write_audit()
                except OSError as exc:
                    self.audit_error = str(exc)
        return self.snapshot()

    def snapshot(self):
        arms = {}
        for side in SIDES:
            blocked = None
            protected = False
            try:
                packet, _ = self._stop_packet(side)
                protected = packet["robot_mode"] == 11
                idle = packet["robot_mode"] in (4, 5)
                usable = packet["robot_mode"] in (4, 5, 7, 11)
            except (ValueError, KeyError, TypeError):
                idle = usable = False
                blocked = "未连接或反馈无效；先连接双臂只读监控"
            target = self._target(side)
            clear_only = False
            try:
                alarm_packet, _ = self._alarm_packet(side)
                clear_only = (alarm_packet["robot_mode"] == 9 or alarm_packet.get("error_status") == 1 or
                              bool(self.states[side].get("alarm_ids")))
            except (ValueError, KeyError, TypeError):
                pass
            if target is None:
                blocked = "尚未保存该臂开始位" if not self.preset_error else "开始位预设无效："+self.preset_error
            if self.load_error:
                blocked = "实测开始位文件无效；修复或重新保存后再回位"
            if self.busy:
                blocked = "正在处理开始位任务；STOP可中断回位"
            if self.fault_latched:
                blocked = "点击回位会先恢复停止确认，失败不会运动"
            arms[side] = {**self.states[side], "saved": self.saved.get(side),
                          "preset": self.presets.get(side), "target": target,
                          "target_source": self._target_source(side),
                          "state": "COLLISION_PROTECTION" if protected and not self.busy else self.states[side]["state"],
                          "recovery_required": protected or self.fault_latched,
                          "blocked_reason": "确认工作区安全后点击，将先清除本臂碰撞保护再回位" if protected and not self.busy else blocked,
                          "can_capture": idle and not self.busy and not self.fault_latched,
                          "clear_only":clear_only,
                          "can_clear_alarms":clear_only and target is not None and not self.busy and not self.fault_latched,
                          "can_return": usable and target is not None and not self.busy and not clear_only}
        return {"schema": 1, "active_side": self.active_side, "operation_id": self.operation_id,
                "source": self.source if self.busy else None, "busy": self.busy,
                "fault_latched": self.fault_latched, "arms": arms, "load_error": self.load_error,
                "preset_error": self.preset_error,
                "audit_path": str(self.audit_path) if self.audit_path else None, "audit_error": self.audit_error,
                "settings": {"speed_percent": RETURN_SPEED_PERCENT, "global_speed_percent": RETURN_SPEED_PERCENT,
                             "acceleration_percent": RETURN_ACCEL_PERCENT, "send_bound_deg": SEND_BOUND_DEG,
                             "segment_deg": SEGMENT_DEG, "start_down_tolerance_deg": START_TILT_DEG,
                             "down_reference": "BASE_MINUS_Z_LEVEL_BASE_REQUIRED",
                             "path": "JOINT_SEGMENTS_NO_CARTESIAN_DOWN_CONSTRAINT",
                             "collision_validation": "NOT_PERFORMED", "right_teleoperation": "SELECTED_SINGLE_ARM",
                             "enable_on_explicit_return": True, "direction_calibration_required": False,
                             "clear_collision_on_explicit_return": True, "preserve_diagnostic_logs": True}}

    async def close(self):
        await self.stop()
        if self.channel:
            await self.channel.release()
            self.channel = None

