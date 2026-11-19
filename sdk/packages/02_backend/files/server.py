"""Local Quest input, read-only diagnostics and explicit left-Z commissioning.

Startup does not connect to a robot. The separate prepare/arm/stop endpoints
expose experimental motion through LeftZTrial; only those explicit operations
can enable or command a robot. LAN routes expose Quest input, not robot APIs.
"""

import asyncio
import ipaddress
import json
import math
import os
import re
import secrets
import struct
import time
from collections import deque
from contextlib import suppress
from datetime import datetime, timezone, timedelta
from pathlib import Path

from aiohttp import web
from quest_lan import QuestLan
from headset_bridge import headset_status, input_ready, home_input_ready, validate_action, validate_calibration_action
from teleop_recording import RecordedDriver, TeleopRecorder, TeleopRecorderGroup
from teleop_group import TeleopGroup
from web_assets import robot_asset_response
from asset_registry import register_asset_routes
from gripper_control import Grippers
from robot_monitor import RobotMonitor
from device_settings import load_device_config, validate as validate_device_config
from start_positions import StartPositions, ReturnChannel, neighbor_identity
from tool_orientation_cone import configuration as cone_configuration

ROOT = Path(__file__).resolve().parent
PORT = 8765
ADB_PORT = 5038  # Dedicated ADB server; do not restart another project's server.
STALE_SECONDS = 0.5
READ_ONLY_COMMANDS = ("RobotMode()", "GetAngle()", "GetPose()", "GetErrorID()")
CHINA = timezone(timedelta(hours=8))
LAN_KEY = web.AppKey("quest_lan", QuestLan)
START_POSITIONS_KEY = web.AppKey("start_positions", StartPositions)
TELEOP_GROUP_KEY = web.AppKey("teleop_group", TeleopGroup)


def now_iso():
    return datetime.now(CHINA).isoformat(timespec="milliseconds")


def numbers(value, length=None, limit=1000):
    if not isinstance(value, list) or (length is not None and len(value) != length):
        raise ValueError("Invalid vector length")
    if len(value) > 16 or any(
        type(x) not in (int, float) or not math.isfinite(x) or abs(x) > limit for x in value
    ):
        raise ValueError("Invalid numeric vector")
    return value


def validate_frame(data):
    if not isinstance(data, dict) or data.get("type") != "frame" or data.get("schema") != 1:
        raise ValueError("Expected WebXR frame schema 1")
    if type(data.get("seq")) is not int or data["seq"] < 0:
        raise ValueError("Invalid sequence")
    if data.get("reference_space") not in ("local-floor", "local"):
        raise ValueError("Unknown reference space")
    if type(data.get("reference_epoch")) is not int or data["reference_epoch"] < 0:
        raise ValueError("Missing reference epoch")
    if data.get("visibility") not in ("visible", "visible-blurred", "hidden"):
        raise ValueError("Invalid session visibility")
    for field in ("xr_frame_time_ms", "client_send_time_ms"):
        numbers([data.get(field)], 1, 1e15)
    hands = data.get("hands")
    if not isinstance(hands, dict) or set(hands) != {"left", "right"}:
        raise ValueError("Both hand slots are required; absent hands must be null")
    for hand in hands.values():
        if hand is None:
            continue
        if not isinstance(hand, dict) or type(hand.get("is_controller")) is not bool:
            raise ValueError("Invalid input source")
        pose = hand.get("pose")
        policy = data.get("input_policy")
        if policy in ("grip-gated-v1", "continuous-pose-grip-v2"):
            if (
                type(hand.get("tracking_valid")) is not bool
                or type(hand.get("grip_held")) is not bool
            ):
                raise ValueError("Missing Grip gate state")
            if policy == "grip-gated-v1" and not hand["grip_held"] and pose is not None:
                raise ValueError("Idle messages must not contain a pose")
            if hand["grip_held"] and (not hand["tracking_valid"] or pose is None):
                raise ValueError("Held Grip requires tracked pose")
            if (
                policy == "continuous-pose-grip-v2"
                and hand["tracking_valid"]
                and (not isinstance(pose, dict) or pose.get("emulated_position") is not False)
            ):
                raise ValueError(
                    "Continuous tracked input requires its real pose, even when released"
                )
        if pose is not None:
            if not isinstance(pose, dict):
                raise ValueError("Invalid pose object")
            numbers(pose.get("position_m"), 3)
            q = numbers(pose.get("quaternion_xyzw"), 4, 1.1)
            if not 0.95 <= sum(x * x for x in q) <= 1.05:
                raise ValueError("Quaternion must be unit length; do not silently repair it")
            if type(pose.get("emulated_position")) is not bool:
                raise ValueError("Missing tracking quality flag")
        gamepad = hand.get("gamepad")
        ray = hand.get("target_ray")
        if ray is not None:
            if not isinstance(ray, dict) or set(ray) != {"quaternion_xyzw", "emulated_position"}:
                raise ValueError("Invalid target-ray pose")
            ray_q = numbers(ray.get("quaternion_xyzw"), 4, 1.1)
            if abs(sum(x*x for x in ray_q)-1) > .001 or type(ray.get("emulated_position")) is not bool:
                raise ValueError("Invalid target-ray rotation/tracking")
        if gamepad is not None:
            if not isinstance(gamepad, dict):
                raise ValueError("Invalid gamepad object")
            if type(gamepad.get("connected")) is not bool:
                raise ValueError("Invalid gamepad connection flag")
            numbers(gamepad.get("axes"), limit=1.001)
            buttons = gamepad.get("buttons")
            if not isinstance(buttons, list) or len(buttons) > 16:
                raise ValueError("Invalid buttons")
            for b in buttons:
                numbers([b.get("value")], 1, 1.001)
                if (
                    b["value"] < 0
                    or type(b.get("pressed")) is not bool
                    or type(b.get("touched")) is not bool
                ):
                    raise ValueError("Invalid button state")
    return data


class QuestState:
    def __init__(self):
        self.connected = False
        self.session = None
        self.user_agent = None
        self.reset()

    def reset(self):
        self.last = None
        self.last_received = None
        self.seq = -1
        self.epoch = None
        self.buffer = deque(maxlen=600)  # About 20 s at 30 Hz, RAM only.
        self.receive_times = deque(maxlen=90)
        self.stats = {
            side: {"tracked_frames": 0, "pressed": set(), "released_after_press": set()}
            for side in ("left", "right")
        }

    def accept(self, data, received=None):
        data = validate_frame(data)
        if data["seq"] <= self.seq:
            raise ValueError("Repeated or out-of-order sequence")
        if self.epoch is not None and data["reference_epoch"] != self.epoch:
            self.reset()  # Never combine measurements from different XR origins.
        self.seq, self.epoch = data["seq"], data["reference_epoch"]
        self.last_received = time.monotonic() if received is None else received
        self.receive_times.append(self.last_received)
        self.last = {**data, "host_received_at": now_iso(), "host_monotonic_s": self.last_received}
        self.buffer.append(self.last)
        for side, hand in data["hands"].items():
            pose_valid = self.hand_valid(hand, data)
            idle_tracked = bool(
                data.get("input_policy") == "grip-gated-v1"
                and data["visibility"] == "visible"
                and hand
                and hand["is_controller"]
                and hand.get("tracking_valid") is True
                and hand.get("grip_held") is False
            )
            if not pose_valid and not idle_tracked:
                continue
            if pose_valid:
                self.stats[side]["tracked_frames"] += 1
            gp = hand.get("gamepad")
            if gp and gp["connected"]:
                for i, button in enumerate(gp["buttons"]):
                    if button["pressed"]:
                        self.stats[side]["pressed"].add(i)
                    elif i in self.stats[side]["pressed"]:
                        self.stats[side]["released_after_press"].add(i)

    @staticmethod
    def hand_valid(hand, frame):
        return bool(
            frame
            and frame["visibility"] == "visible"
            and hand
            and hand["is_controller"]
            and hand.get("pose")
            and not hand["pose"]["emulated_position"]
            and (
                frame.get("input_policy") != "continuous-pose-grip-v2"
                or hand.get("tracking_valid") is True
            )
        )

    def snapshot(self, current=None):
        current = time.monotonic() if current is None else current
        age = None if self.last_received is None else current - self.last_received
        fresh = self.connected and age is not None and 0 <= age < STALE_SECONDS
        status = (
            "WAITING_FOR_QUEST"
            if self.last is None
            else ("LIVE" if fresh else "STALE_OR_DISCONNECTED")
        )
        if fresh and self.last["visibility"] != "visible":
            status = "XR_NOT_VISIBLE"
        times = self.receive_times
        hz = (
            (len(times) - 1) / (times[-1] - times[0])
            if fresh and len(times) > 1 and times[-1] > times[0]
            else 0
        )
        return {
            "status": status,
            "connected": self.connected,
            "age_ms": None if age is None else round(age * 1000),
            "receive_hz": round(hz, 1),
            "session": self.session,
            "user_agent": self.user_agent,
            "frame": self.last if fresh else None,
            "valid": {
                side: fresh
                and self.hand_valid((self.last or {}).get("hands", {}).get(side), self.last)
                for side in ("left", "right")
            },
            "stats": {
                s: {k: sorted(v) if isinstance(v, set) else v for k, v in a.items()}
                for s, a in self.stats.items()
            },
            "physical_acceptance": "NOT_PERFORMED_AUTOMATICALLY",
        }


async def adb_command(*args):
    executable = ROOT / "vendor/platform-tools/adb.exe"
    if not executable.exists():
        raise RuntimeError("ADB missing; run setup script")
    process = await asyncio.create_subprocess_exec(
        str(executable),
        "-P",
        str(ADB_PORT),
        *args,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE,
    )
    try:
        stdout, stderr = await asyncio.wait_for(process.communicate(), 12)
    except asyncio.TimeoutError:
        process.kill()
        await process.communicate()
        raise RuntimeError("ADB command timed out")
    if process.returncode:
        raise RuntimeError(
            stderr.decode("utf-8", "replace").strip() or f"ADB exit {process.returncode}"
        )
    return stdout.decode("utf-8", "replace").strip()


async def adb_devices():
    raw = await adb_command("devices", "-l")
    devices = []
    for line in raw.splitlines():
        if not line or line.startswith(("List of devices", "*")):
            continue
        fields = line.split()
        if len(fields) >= 2:
            devices.append(
                {"serial": fields[0], "state": fields[1], "details": " ".join(fields[2:])}
            )
    return devices


async def send_readonly(reader, writer, command):
    if command not in READ_ONLY_COMMANDS:
        raise ValueError("Blocked: command is outside the read-only allowlist")
    started = time.monotonic()
    writer.write(command.encode("ascii"))
    await asyncio.wait_for(writer.drain(), 2)
    raw = b""
    deadline = time.monotonic() + 2
    while b";" not in raw:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(f"Incomplete E6 reply: {raw.decode('utf-8', 'replace')!r}")
        chunk = await asyncio.wait_for(reader.read(1024), remaining)
        if not chunk:
            raise ConnectionError("E6 closed the connection before a complete reply")
        raw += chunk
        if len(raw) > 8192:
            raise ValueError("Oversized E6 reply")
        # Observed V4 mode rejection has a tab terminator, not the normal semicolon.
        if raw.strip() == b"Control Mode Is Not Tcp":
            return {
                "command": command,
                "error_id": None,
                "protocol_error": "NOT_TCP_MODE",
                "raw_reply": raw.decode().strip(),
                "request_reply_ms": round((time.monotonic() - started) * 1000, 3),
            }
    text = raw.decode("utf-8", "strict")
    match = re.fullmatch(
        r"\s*(-?\d+)\s*,\s*\{([^;\r\n]*)\},\s*([A-Za-z][A-Za-z0-9]*)\([^();\r\n]*\);\s*",
        text,
    )
    if match is None or match[3] != command.split("(", 1)[0]:
        raise ValueError("Malformed E6 reply or command echo mismatch")
    return {
        "command": command,
        "error_id": int(match[1]),
        "raw_reply": text,
        "request_reply_ms": round((time.monotonic() - started) * 1000, 3),
    }


async def close_writer(writer):
    if writer is not None:
        writer.close()
        with suppress(OSError, asyncio.TimeoutError):
            await asyncio.wait_for(writer.wait_closed(), 1)


def parse_feedback_header(data):
    """Validate the V4 1440-byte envelope; decode only the documented stable header.

    Layout reference: Dobot-Arm/TCP-IP-Python-V4/main/dobot_api.py, MyType.
    This does not validate the remaining model/firmware-specific payload fields.
    """
    if len(data) != 1440 or struct.unpack_from("<H", data, 0)[0] != 1440:
        raise ValueError("Unexpected V4 feedback size")
    if struct.unpack_from("<Q", data, 48)[0] != 0x0123456789ABCDEF:
        raise ValueError("V4 feedback marker mismatch; refusing to decode")
    mode = struct.unpack_from("<Q", data, 24)[0]
    names = {
        1: "INITIALIZING",
        2: "BRAKE_OPEN",
        3: "POWER_OFF",
        4: "DISABLED",
        5: "ENABLED_IDLE",
        6: "DRAG",
        7: "RUNNING",
        8: "SINGLE_MOVE",
        9: "ERROR",
        10: "PAUSED",
        11: "COLLISION",
    }
    return {
        "packet_bytes": len(data),
        "header_valid": True,
        "robot_mode": mode,
        "robot_mode_name": names.get(mode, "UNKNOWN"),
        "controller_timestamp_raw": struct.unpack_from("<Q", data, 32)[0],
        "scope": "V4_HEADER_ONLY_NOT_FULL_PAYLOAD_VALIDATION",
    }


def e6_targets(cfg, sides):
    available = validate_device_config(cfg)
    if not sides or any(side not in available for side in sides):
        raise ValueError("DEVICE_NOT_CONFIGURED: edit runtime/devices.local.json for the selected arm")
    if cfg["left_e6_ip"] == cfg["right_e6_ip"]:
        local_ips = [cfg.get(f"{side}_e6_local_ip") for side in ("left", "right")]
        if None in local_ips or len(set(local_ips)) != 2:
            raise ValueError("Duplicate robot IPs require distinct explicit local interface IPs")
    targets = [(cfg[f"{side}_e6_ip"], cfg.get(f"{side}_e6_local_ip")) for side in sides]
    for ip, local_ip in targets:
        for value in [ip] if local_ip is None else [ip, local_ip]:
            addr = ipaddress.ip_address(value)
            if addr.version != 4 or not addr.is_private or addr.is_loopback or addr.is_unspecified:
                raise ValueError("Robot and local interface IPs must be private IPv4 addresses")
    return targets


async def inspect_e6(ip, local_ip=None):
    result = {
        "ip": ip,
        "local_ip": local_ip,
        "checked_at": now_iso(),
        "dashboard_tcp": False,
        "feedback_tcp": False,
        "feedback_header": None,
        "replies": [],
        "errors": [],
    }
    connection_options = {"local_addr": (local_ip, 0)} if local_ip else {}
    writer = None
    try:
        reader, writer = await asyncio.wait_for(
            asyncio.open_connection(ip, 29999, limit=8192, **connection_options), 2
        )
        result["dashboard_tcp"] = True
        result["dashboard_local_ip"] = writer.get_extra_info("sockname")[0]
        for command in READ_ONLY_COMMANDS:
            reply = await send_readonly(reader, writer, command)
            result["replies"].append(reply)
            if reply.get("protocol_error") == "NOT_TCP_MODE":
                break
    except (
        OSError,
        asyncio.TimeoutError,
        asyncio.IncompleteReadError,
        asyncio.LimitOverrunError,
        ValueError,
    ) as exc:
        result["errors"].append(f"29999: {type(exc).__name__}: {exc}")
    finally:
        await close_writer(writer)
    writer = None
    try:
        reader, writer = await asyncio.wait_for(
            asyncio.open_connection(ip, 30004, **connection_options), 2
        )
        result["feedback_tcp"] = True
        result["feedback_local_ip"] = writer.get_extra_info("sockname")[0]
        packet = await asyncio.wait_for(reader.readexactly(1440), 2)
        result["feedback_header"] = parse_feedback_header(packet)
    except (OSError, asyncio.TimeoutError, asyncio.IncompleteReadError, ValueError) as exc:
        result["errors"].append(f"30004: {type(exc).__name__}: {exc}")
    finally:
        await close_writer(writer)
    result["protocol_queries_ok"] = len(result["replies"]) == 4 and all(
        r["error_id"] == 0 for r in result["replies"]
    )
    return result


@web.middleware
async def local_only(request, handler):
    if request.remote not in ("127.0.0.1", "::1") or request.host not in (
        f"localhost:{PORT}",
        f"127.0.0.1:{PORT}",
    ):
        raise web.HTTPForbidden(text="Local USB/loopback service only")
    origin = request.headers.get("Origin")
    if origin and origin not in (f"http://localhost:{PORT}", f"http://127.0.0.1:{PORT}"):
        raise web.HTTPForbidden(text="Unexpected origin")
    try:
        response = await handler(request)
    except (ValueError, RuntimeError) as exc:
        return web.json_response({"error": str(exc)}, status=400)
    response.headers["Cache-Control"] = "no-store"
    response.headers["X-Content-Type-Options"] = "nosniff"
    return response


def make_app(*, recording_enabled=True, recordings_root=None, ranged_profile_path=None, start_positions_path=None,
             calibration_log_dir=None, start_presets_path=None, trial_runtime_dir=None):
    app = web.Application(middlewares=[local_only], client_max_size=16384)
    ranged_profile = None
    direction_calibration = None
    from direction_calibration import DirectionCalibration
    from operator_frame import OperatorFrame
    operator_frame = OperatorFrame()
    direction_calibrations = {side: DirectionCalibration(side, operator_frame) for side in ("left", "right")}
    if ranged_profile_path is not None:
        from ranged_ik_profile import RangedIKProfile
        ranged_profile = RangedIKProfile.load(ranged_profile_path)
        direction_calibration = direction_calibrations["left"]
    quest = QuestState()
    grippers = Grippers(ROOT)
    token = secrets.token_urlsafe(24)
    adb_state = {"devices": [], "checked_at": None}
    robot_state = {"checked_at": None, "robots": [], "note": "NOT_CHECKED"}
    e6_lock = asyncio.Lock()
    usb_lock = asyncio.Lock()
    trial_boxes = {side: {"controller": None, "reader": None, "driver": None} for side in ("left", "right")}
    group = TeleopGroup()
    app[TELEOP_GROUP_KEY] = group
    trial_lock = asyncio.Lock()
    monitors = RobotMonitor()
    calibration_audit_error = None
    calibration_audit_dir = Path(calibration_log_dir) if calibration_log_dir else ROOT / "runtime/calibration_events"

    async def audit_calibration(side, action, ok, message):
        nonlocal calibration_audit_error
        if not recording_enabled and calibration_log_dir is None:
            return
        q = quest.snapshot()
        frame = q.get("frame") or {}
        hand = (frame.get("hands") or {}).get(side) or {}
        payload = {"at": now_iso(), "side": side, "action": action, "ok": ok, "message": message,
                   "quest_status": q.get("status"), "quest_age_ms": q.get("age_ms"),
                   "attempt_input": {"seq": frame.get("seq"), "reference_epoch": frame.get("reference_epoch"),
                                     "position_m": (hand.get("pose") or {}).get("position_m"),
                                     "tracking_valid": hand.get("tracking_valid"), "grip_held": hand.get("grip_held")},
                   "calibration": direction_calibrations[side].snapshot(q, home_feedback(side)),
                   "operator_frame": operator_frame.snapshot(q)}
        line = json.dumps(payload, ensure_ascii=False)+"\n"
        path = calibration_audit_dir / (datetime.now(CHINA).strftime("%Y-%m-%d")+"-calibration-actions.jsonl")
        def write():
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("a", encoding="utf-8") as stream:
                stream.write(line)
        try:
            await asyncio.to_thread(write)
        except OSError as exc:
            calibration_audit_error = str(exc)
    recorder = TeleopRecorderGroup({
        side: TeleopRecorder(recordings_root or Path(os.environ.get("USERPROFILE", str(Path.home())))
                            / "Documents" / "E6_Teleop_Recordings", ROOT, quest.snapshot,
                            lambda side=side: trial_boxes[side]["controller"], enabled=recording_enabled, side=side)
        for side in ("left", "right")
    }, lambda: group.participants)
    group.event = recorder.capture

    async def start_recording_monitor(app):
        await recorder.start()
        await group.start()

    app.on_startup.append(start_recording_monitor)

    def home_feedback(side):
        return monitors.snapshot(side, trial_boxes[side]["reader"])

    def home_owner_valid(source, owner):
        return source == "pc" or (quest.session == owner and home_input_ready(quest.snapshot(), homes.active_side))

    async def quiesce_for_home(*, capture, recover=False):
        async with trial_lock:
            if recover:
                stopped = await group.stop("EXPLICIT_COLLISION_RECOVERY_AND_RETURN")
                if not (stopped.get("stop_evidence") or {}).get("feedback_confirmed"):
                    raise ValueError("START_GROUP_STOP_NOT_CONFIRMED")
                return
            trials = list(group.controllers.values())
            if group.state == "FAULT" or any(t.state == "FAULT" and not (t.stop_evidence or {}).get("feedback_confirmed") for t in trials):
                raise ValueError("START_GROUP_STOP_NOT_CONFIRMED")
            if any(t.armed or t.state in ("ARMING", "STOPPING") for t in trials):
                if capture:
                    raise ValueError("START_STOP_TELEOP_BEFORE_CAPTURE")
                stopped = await group.stop("RETURN_TO_START_REQUESTED")
                if not (stopped.get("stop_evidence") or {}).get("feedback_confirmed"):
                    raise ValueError("START_GROUP_STOP_NOT_CONFIRMED")

    async def acquire_home_channel(side):
        from dobot_safe_sdk import BoundDobotDashboard, require_success
        async with trial_lock:
            cfg = load_device_config(ROOT)
            ip, source_ip = e6_targets(cfg, [side])[0]
            trial = trial_boxes[side]["controller"]
            if trial is None:
                await monitors.connect(side, ip, source_ip)
            for _ in range(40):
                candidate = home_feedback(side)
                if candidate.get("valid") or (candidate.get("packet_valid") and candidate.get("robot_mode") == 11):
                    break
                await asyncio.sleep(.05)
            p = home_feedback(side)
            readable = p.get("valid") or (p.get("packet_valid") and p.get("robot_mode") == 11)
            if not readable or p.get("local_ip") != source_ip or p.get("ip") != ip:
                raise ValueError("START_SOURCE_FEEDBACK_NOT_READY")
            identity = await neighbor_identity(ip, source_ip)
            async def verify_identity():
                if await neighbor_identity(ip, source_ip) != identity:
                    raise ValueError("START_STOP_RECOVERY_ROBOT_IDENTITY_CHANGED")
            async def controller_status():
                from communication_probe import read_json
                response = await asyncio.to_thread(read_json, ip, 22000, "/protocol/exchange", source_ip)
                if not response.get("ok"):
                    raise ValueError("COLLISION_RECOVERY_CONTROLLER_STATUS_UNAVAILABLE")
                return response["data"]
            if trial and trial.driver.connected:
                async def release():
                    pass
                return ReturnChannel(trial.feedback, trial._call, identity, release,
                                     connected=lambda: trial.driver.connected, verify_identity=verify_identity,
                                     controller_status=controller_status)
            driver = RecordedDriver(await asyncio.to_thread(BoundDobotDashboard, ip, source_ip), recorder.for_side(side))
            io_lock = asyncio.Lock()

            async def call(method, *args, guard=None, **kwargs):
                async with io_lock:
                    if guard:
                        guard()
                    return require_success(await asyncio.to_thread(getattr(driver, method), *args, **kwargs))

            async def release():
                driver.close()

            return ReturnChannel(trial.feedback if trial else monitors.readers[side], call, identity, release,
                                 connected=lambda: driver.connected, verify_identity=verify_identity,
                                 controller_status=controller_status)

    homes = StartPositions(start_positions_path or ROOT / "runtime/start_positions.json",
                           acquire_home_channel, quiesce_for_home, home_feedback, home_owner_valid,
                           event=recorder.capture,
                           presets_path=start_presets_path or (ROOT / "runtime/start_position_presets.local.json"
                                                              if start_positions_path is None else None))
    app[START_POSITIONS_KEY] = homes

    async def drive_gripper_references():
        while True:
            now = time.monotonic()
            allowed = set()
            for side in grippers.channels:
                trial = trial_boxes[side]["controller"]
                if trial is None or not group.motion_allowed(side) or trial.state not in ("READY","FOLLOWING"):
                    continue
                feedback = trial.feedback.snapshot()
                stamp = feedback.get("host_monotonic_s")
                age = now - stamp if type(stamp) in (int, float) else math.inf
                if (feedback.get("valid") is True and feedback.get("connected") is True and 0 <= age < .2
                        and feedback.get("robot_mode") in (5,7) and feedback.get("error_status") == 0):
                    allowed.add(side)
            inhibited = homes.busy or homes.fault_latched or group.state in ("STOPPING","FAULT","ARMING")
            owners = group.participants if group.state == "ARMED" else ()
            grippers.tick(quest.snapshot(now), allowed, now, recorder.capture, inhibited, owners)
            await asyncio.sleep(.033)

    async def gripper_lifetime(app):
        grippers.task = asyncio.create_task(drive_gripper_references(), name="gripper-references")
        try:
            yield
        finally:
            await grippers.close()
    app.cleanup_ctx.append(gripper_lifetime)

    async def preview_grippers(request):
        try:
            values = {side: float(request.query[side]) for side in ("left","right")}
            return web.json_response(grippers.preview(values))
        except (KeyError, ValueError):
            raise web.HTTPBadRequest(text="Supply left and right trigger values in [0,1]")

    async def asset_preview(request):
        return web.FileResponse(ROOT/"web/gripper_preview.html")

    def require_home_idle():
        if homes.busy or homes.fault_latched:
            raise web.HTTPConflict(text="回位正在进行或停止未确认；请先使用STOP")

    async def begin_home_return(side, source, owner):
        if homes.busy:
            raise web.HTTPConflict(text="已有回位任务；等待完成或使用STOP")
        if homes.fault_latched:
            await homes.stop()
        return homes.begin(side, source, owner, recover_collision=True)

    async def home_action(request):
        data = await request.json()
        if not isinstance(data, dict) or set(data) != {"action", "side", "client_id"}:
            raise ValueError("Expected action, side, client_id")
        side, action, owner = data["side"], data["action"], data["client_id"]
        if side not in ("left", "right") or not isinstance(owner, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,80}", owner):
            raise ValueError("Invalid start-position request")
        recorder.request("start_"+str(action), "pc_console")
        if action == "capture":
            return web.json_response(await homes.capture(side))
        if action == "return":
            if group.state in ("ARMING", "STOPPING"):
                raise web.HTTPConflict(text="等待统一启动/停止完成后再回位")
            return web.json_response(await begin_home_return(side, "pc", owner))
        raise ValueError("Unknown start-position action")

    async def home_heartbeat(request):
        data = await request.json()
        if not isinstance(data, dict) or set(data) != {"operation_id", "client_id"}:
            raise ValueError("Invalid heartbeat")
        homes.heartbeat(data["operation_id"], data["client_id"])
        return web.json_response({"ok": True})

    def calibration_states():
        q = quest.snapshot()
        quiet = not homes.busy and not homes.fault_latched and (
            group.state not in ("ARMING", "STOPPING", "FAULT") and all(
                not trial.armed and trial.state not in ("ARMING", "STOPPING")
                and (trial.state != "FAULT" or (trial.stop_evidence or {}).get("feedback_confirmed"))
                for trial in group.controllers.values()))
        views, allowed = {}, {}
        operator = operator_frame.snapshot(q)
        for side, calibration in direction_calibrations.items():
            view = calibration.snapshot(q, home_feedback(side))
            views[side] = view
            sample_ready = quiet and view["capture_ready"]
            allowed[side] = {
                "begin": sample_ready,
                "capture_x": sample_ready and view["state"] == "WAIT_X",
                "cancel": quiet and view["state"] == "WAIT_X",
                "clear": quiet and view["state"] in ("CALIBRATED", "INVALID"),
                "record_forward": quiet and operator["capture_ready"][side],
            }
        return views, allowed, operator

    def headset_snapshot():
        q = quest.snapshot()
        connected = [s for s in ("left", "right") if home_feedback(s).get("connected")]
        primary = (group.participants or tuple(connected) or ("left",))[0]
        views, allowed, operator = calibration_states()
        heads = {s: headset_status(q, trial_boxes[s]["controller"], s) for s in ("left", "right")}
        robots, controls = {}, {}
        for side, head in heads.items():
            robots[side] = head["robot"] if trial_boxes[side]["controller"] else home_feedback(side)
            controls[side] = head["control"]
            if direction_calibration is not None:
                controls[side]["direction_calibration"] = views[side]
                head["capabilities"]["arm"] = head["capabilities"]["arm"] and views[side]["control_ready"]
        data = {**heads[primary], "recording": recorder.snapshot(), "robot": robots[primary],
                "robots": robots, "control": controls[primary], "controls": controls}
        data["group"] = {**group.snapshot(), "connected_sides": connected,
                         "arm_ready": {s: heads[s]["capabilities"]["arm"] for s in ("left", "right")}}
        data["teleop_side"] = "both" if len(group.participants) == 2 else primary
        data["single_arm"] = len(group.participants) < 2
        data["scope"] = "AUTO_CONNECTED_SINGLE_OR_DUAL_TELEOP"
        data.update(direction_calibrations=views, calibration_capabilities=allowed, operator_frame=operator,
                    calibration_log_error=calibration_audit_error)
        try:
            group.require_stopped()
            prepare_allowed = True
        except ValueError:
            prepare_allowed = False
        data["capabilities"].update(
            prepare=prepare_allowed and q.get("connected") is True,
            arm=bool(group.participants) and group.state == "PREPARED" and
                all(heads[s]["capabilities"]["arm"] for s in group.participants),
            stop=bool(group.controllers),
        )
        data["capabilities"].update({"calibration_"+action: permitted for action, permitted in allowed[primary].items()})
        data["start_positions"] = homes.snapshot()
        data["grippers"] = grippers.snapshot()
        for arm in ("left", "right"):
            data["capabilities"][f"return_{arm}_start"] = (
                group.state not in ("ARMING", "STOPPING") and
                home_input_ready(q, arm) and data["start_positions"]["arms"][arm]["can_return"])
        if homes.busy or homes.fault_latched:
            data["capabilities"].update(prepare=False, arm=False, stop=True)
            for name in list(data["capabilities"]):
                if name.startswith("calibration_"):
                    data["capabilities"][name] = False
        return data

    async def monitor_status(request):
        head = headset_snapshot()
        return web.json_response({
            "type": "pc_monitor_status", "schema": 1,
            "server_monotonic_s": time.monotonic(),
            "robots": {side: home_feedback(side) for side in ("left", "right")},
            "grippers": head["grippers"], "controls": head["controls"], "teleop_side": head["teleop_side"],
            "single_arm": head["single_arm"], "group": head["group"],
            "start_positions": head["start_positions"],
            "direction_calibrations": head["direction_calibrations"], "operator_frame": head["operator_frame"],
            "calibration_log_error": calibration_audit_error,
            "quest": quest.snapshot(), "recording": recorder.snapshot(),
            "lan": app[LAN_KEY].snapshot(),
            "scope": "AUTO_CONNECTED_SINGLE_OR_DUAL_TELEOP",
            "configured_tool_orientation_cone": cone_configuration() if ranged_profile else None,
            "configured_ranged_ik": ranged_profile.snapshot() if ranged_profile else None,
        })

    async def monitor_connect(request):
        async with trial_lock:
            require_home_idle()
            side = request.query.get("side", "both")
            if side not in ("left", "right", "both"):
                raise ValueError("side must be left, right or both")
            sides = ["left", "right"] if side == "both" else [side]
            cfg = load_device_config(ROOT)
            targets = e6_targets(cfg, sides)
            for arm, (ip, local_ip) in zip(sides, targets):
                if trial_boxes[arm]["reader"] is not None:
                    continue  # Reuse the existing control reader, never a second socket.
                await monitors.connect(arm, ip, local_ip)
            return await monitor_status(request)

    async def monitor_disconnect(request):
        async with trial_lock:
            require_home_idle()
            side = request.query.get("side", "both")
            if side not in ("left", "right", "both"):
                raise ValueError("Invalid side")
            for arm in (["left", "right"] if side == "both" else [side]):
                await monitors.disconnect(arm)
            return await monitor_status(request)

    async def monitor_page(request):
        return web.FileResponse(ROOT / "web/pc_monitor.html")

    async def headset_status_handler(request):
        return web.json_response(headset_snapshot())

    async def capture_recording_diagnostic(request):
        # Exercise real local capture without changing a robot state or arming.
        if not recording_enabled:
            raise web.HTTPConflict(text="Recording is explicitly disabled")
        if any(trial.armed or trial.state in ("ARMING", "STOPPING") for trial in group.controllers.values()):
            raise web.HTTPConflict(text="An active trial already records automatically")
        recorder.begin("READ_ONLY_DIAGNOSTIC")
        recorder.capture("diagnostic_scope", {"motion_requested": False})
        recorder.finish_deadline = time.monotonic() + 2
        return web.json_response(recorder.snapshot())

    async def hud_preview_page(request):
        return web.FileResponse(ROOT / "web/hud_preview.html")

    async def status(request):
        head = headset_snapshot()
        return web.json_response(
            {
                "project_id": "quest3-dual-e6-readonly-20261029",
                "mode": "DUAL_START_RETURN_ACTIVE" if homes.busy else (
                    (head["teleop_side"].upper()+"_RANGEDIK_SE3_AVAILABLE" if ranged_profile else head["teleop_side"].upper()+"_INCREMENTAL_SE3_AVAILABLE")
                    if group.controllers else "READ_ONLY_NO_MOTION"),
                "grippers": head["grippers"],
                "left_trial": head["controls"]["left"],
                "right_trial": head["controls"]["right"],
                "teleop": group.snapshot(), "group": head["group"],
                "teleop_side": head["teleop_side"],
                "quest": quest.snapshot(),
                "adb": adb_state,
                "e6": robot_state,
                "lan": app[LAN_KEY].snapshot(),
                "recording": recorder.snapshot(),
                "configured_ranged_ik": ranged_profile.snapshot() if ranged_profile else None,
                "configured_tool_orientation_cone": cone_configuration() if ranged_profile else None,
                "start_positions": homes.snapshot(),
            }
        )

    async def connect_usb(request):
        if usb_lock.locked():
            raise web.HTTPConflict(text="USB setup already running")
        async with usb_lock:
            adb_state.clear()
            adb_state.update({"checked_at": now_iso(), "devices": await adb_devices()})
            candidates = [
                d for d in adb_state["devices"] if d["state"] == "device" and ":" not in d["serial"]
            ]
            if not candidates:
                adb_state["message"] = (
                    "No authorized USB device. Connect Quest, enable Developer Mode and accept USB debugging."
                )
                return web.json_response(adb_state)
            quests = []
            for device in candidates:
                model = await adb_command(
                    "-s", device["serial"], "shell", "getprop", "ro.product.model"
                )
                device["model"] = model
                if "quest" in model.lower():
                    quests.append(device)
            if len(quests) != 1:
                raise ValueError(
                    "Expected exactly one authorized USB Quest. Disconnect other headsets; non-Quest devices are not used."
                )
            serial = quests[0]["serial"]
            await adb_command("-s", serial, "reverse", f"tcp:{PORT}", f"tcp:{PORT}")
            launch = await adb_command(
                "-s",
                serial,
                "shell",
                "am",
                "start",
                "-a",
                "android.intent.action.VIEW",
                "-d",
                f"http://localhost:{PORT}/quest",
            )
            adb_state.update(
                {
                    "selected": quests[0],
                    "forwarded": True,
                    "browser_launch_result": launch,
                    "message": "Put on Quest, open its Browser, then click Start AR or Start VR. This page reads inputs only.",
                }
            )
            return web.json_response(adb_state)

    async def check_e6(request):
        require_home_idle()
        if e6_lock.locked():
            raise web.HTTPConflict(text="Read-only robot check already running")
        async with e6_lock, trial_lock:
            cfg = load_device_config(ROOT)
            side = request.query.get("side", "both")
            if side not in ("left", "right", "both"):
                raise ValueError("side must be left, right or both")
            sides = ["left", "right"] if side == "both" else [side]
            if any(s in monitors.readers for s in sides):
                raise web.HTTPConflict(text="Passive feedback monitoring is active. Use the monitor page or disconnect its passive readers before a separate port check.")
            if any(trial_boxes[side]["controller"] is not None for side in sides):
                raise web.HTTPConflict(
                    text="Left control owns the robot connection. Read left_trial.feedback "
                    "or use communication_probe.py instead of opening another command socket."
                )
            targets = e6_targets(cfg, sides)
            results = await asyncio.gather(*(inspect_e6(ip, local_ip) for ip, local_ip in targets))
            previous = {r["side"]: r for r in robot_state["robots"]}
            for arm, result in zip(sides, results):
                result["side"] = arm
                previous[arm] = result
            robot_state.update(
                {
                    "checked_at": now_iso(),
                    "robots": [previous[s] for s in ("left", "right") if s in previous],
                    "note": "Queried fixed read-only commands; IP labels do not verify robot serial identity.",
                }
            )
            return web.json_response(robot_state)

    async def websocket(request):
        if request.query.get("token") != token:
            raise web.HTTPForbidden(text="Invalid local session token")
        if quest.connected:
            raise web.HTTPConflict(text="Another Quest page is already connected")
        ws = web.WebSocketResponse(heartbeat=3, max_msg_size=16384)
        await ws.prepare(request)
        quest.reset()
        quest.connected, quest.session = True, secrets.token_hex(8)
        quest.transport = "LAN_TLS" if request.secure else "LOOPBACK_HTTP"
        quest.peer_ip = request.remote
        quest.user_agent = request.headers.get("User-Agent", "")[:300]
        owner_session = quest.session
        send_lock = asyncio.Lock()
        action_task = stop_task = None
        seen_actions = deque(maxlen=128)

        async def send_hud(data):
            async with send_lock:
                if not ws.closed:
                    await ws.send_json(data)

        async def publish_status():
            try:
                while not ws.closed:
                    await send_hud(headset_snapshot())
                    await asyncio.sleep(0.2)
            except (ConnectionError, RuntimeError):
                pass

        async def run_action(request_id, action, *, is_calibration=False, calibration_side="left", control_side="left"):
            result = {"type": "action_result", "id": request_id, "action": action, "ok": False}
            try:
                if not is_calibration:
                    recorder.request(action, "paired_headset")
                if quest.session != owner_session or not quest.connected or ws.closed:
                    raise ValueError("输入会话已改变，请重新连接；操作不会自动重放")
                if is_calibration and action in ("capture_y", "apply"):
                    raise ValueError("新版只采X，完成X后自动应用；无需+Y或单独应用，请刷新头显页面")
                if action == "arm":
                    require_prepared_request(control_side)
                    if not all(input_ready(quest.snapshot(), s) for s in group.participants):
                        raise ValueError("请保持AR可见，所有已准备侧的手柄有效并松开Grip")
                capabilities = headset_snapshot()["capabilities"]
                capability = "calibration_" + action if is_calibration else action
                permitted = (headset_snapshot()["calibration_capabilities"][calibration_side].get(action, False)
                             if is_calibration else capabilities.get(capability, False))
                if not permitted:
                    raise ValueError("当前状态不允许此操作；先准备/重检，查看机器人及追踪状态")
                if is_calibration:
                    async with trial_lock:
                        if not headset_snapshot()["calibration_capabilities"][calibration_side].get(action, False):
                            raise ValueError("校准上下文已过期，请重新准备")
                        if action == "record_forward":
                            operator_view = operator_frame.capture(quest.snapshot(), calibration_side)
                            direction_calibrations[calibration_side].last_error = None
                            calibration_view = direction_calibrations[calibration_side].snapshot(quest.snapshot(), home_feedback(calibration_side))
                            message = operator_view["message"]
                        else:
                            calibration_view = direction_calibrations[calibration_side].apply_action(
                                action, quest.snapshot(), home_feedback(calibration_side))
                            message = calibration_view["message"]
                        recorder.capture("direction_calibration_action", {
                            "action": action, "side": calibration_side, "result": calibration_view,
                            "operator_frame": operator_frame.snapshot(quest.snapshot()),
                        })
                    result.update(ok=True, message=message, side=calibration_side,
                                  direction_calibration=calibration_view,
                                  revision=calibration_view["revision"])
                    await audit_calibration(calibration_side, action, True, message)
                    await send_hud(result)
                    return
                if action == "prepare":
                    await prepare_trial(None, control_side)
                elif action == "arm":
                    await arm_trial(None, control_side)
                elif action in ("return_left_start", "return_right_start"):
                    side = "left" if action == "return_left_start" else "right"
                    result.update(ok=True, message="回位请求已接受；请保持AR可见并松开Grip，查看实测到位状态",
                                  start_positions=await begin_home_return(side, "vr", owner_session))
                    await send_hud(result)
                    return
                else:
                    await stop_left_trial(None)
                result.update(
                    ok=True,
                    message={
                        "prepare": "机器人预检完成；点击开始遥操作",
                        "arm": "已允许：先松开Grip，再握持进行小幅位移与转角操作",
                        "stop": "停止请求已完成，请查看静止反馈确认",
                    }[action],
                )
            except Exception as exc:
                recorder.rejected(action, exc)
                result["message"] = getattr(exc, "text", None) or str(exc)
                if is_calibration:
                    direction_calibrations[calibration_side].last_error = result["message"]
                    await audit_calibration(calibration_side, action, False, result["message"])
            await send_hud(result)

        publisher = asyncio.create_task(publish_status(), name="quest-hud-status")
        try:
            async for message in ws:
                if message.type == web.WSMsgType.TEXT:
                    try:
                        data = json.loads(message.data)
                        if isinstance(data, dict) and data.get("type") in ("control_action", "calibration_action"):
                            is_calibration = data["type"] == "calibration_action"
                            request_id, action = (validate_calibration_action(data) if is_calibration else validate_action(data))
                            if request_id in seen_actions:
                                await send_hud(
                                    {
                                        "type": "action_result",
                                        "id": request_id,
                                        "ok": False,
                                        "message": "重复操作已拒绝",
                                    }
                                )
                                continue
                            seen_actions.append(request_id)
                            busy = action_task is not None and not action_task.done()
                            stopping = stop_task is not None and not stop_task.done()
                            if stopping or (busy and action != "stop"):
                                await send_hud(
                                    {
                                        "type": "action_result",
                                        "id": request_id,
                                        "ok": False,
                                        "message": "上一操作正在处理；STOP可优先请求",
                                    }
                                )
                                continue
                            task = asyncio.create_task(
                                run_action(request_id, action, is_calibration=is_calibration,
                                           calibration_side=data.get("side", "left"),
                                           control_side=data.get("side", "left")), name=f"quest-action-{action}"
                            )
                            if action == "stop":
                                stop_task = task
                            else:
                                action_task = task
                        else:
                            quest.accept(data)
                            recorder.capture("quest_frame", quest.last)
                    except (ValueError, TypeError, KeyError, AttributeError) as exc:
                        await ws.send_json({"error": str(exc)})
                        await ws.close(code=1007, message=b"Invalid input frame")
                        break
        finally:
            if quest.session == owner_session:
                quest.connected = False
            publisher.cancel()
            with suppress(asyncio.CancelledError):
                await publisher
            # Never cancel a robot transaction running in a worker thread. The
            # existing watchdog sees disconnected input and revokes the trial.
            for task in (action_task, stop_task):
                if task is not None:
                    with suppress(Exception):
                        await task
        return ws

    async def save_observations(request):
        state = quest.snapshot()
        if state["status"] != "LIVE" or not any(state["valid"].values()):
            raise web.HTTPConflict(text="No fresh tracked controller data to save")
        cutoff = time.monotonic() - 10
        samples = [x for x in quest.buffer if x["host_monotonic_s"] >= cutoff]
        output = ROOT / "observations"
        output.mkdir(exist_ok=True)
        name = datetime.now(CHINA).strftime("%Y-%m-%d-%H-%M-%S-%f-Quest-input.json")
        data = {
            "recorded_at": now_iso(),
            "mode": "READ_ONLY_NO_MOTION",
            "physical_acceptance": "NOT_PERFORMED_AUTOMATICALLY",
            "frame_convention": "WebXR: metres, quaternion xyzw, +X right, +Y up, -Z forward; not robot coordinates",
            "timing": "XR frame time and PC receive time have different clock origins; receive age is not end-to-end latency",
            "adb_snapshot": dict(adb_state),
            "transport": getattr(quest, "transport", None),
            "peer_ip": getattr(quest, "peer_ip", None),
            "session": quest.session,
            "stats": state["stats"],
            "samples": samples,
        }
        (output / name).write_text(
            json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8"
        )
        return web.json_response({"file": str(output / name), "sample_count": len(samples)})

    async def page(request):
        name = request.match_info.get("name", "index.html")
        if name not in (
            "index.html",
            "quest.html",
            "style.css",
            "dashboard.js",
            "quest.js",
            "grip_gate.mjs",
            "quest_hud.mjs",
            "robot_view.mjs",
            "direction_calibration.mjs",
            "tracking_warning.mjs",
            "e6_urdf_view.mjs",
            "pc_monitor.mjs",
            "pc_monitor.css",
            "pc_monitor_state.mjs",
            "controller_model_view.mjs",
            "asset_store.mjs", "gripper_model.mjs", "gripper_ui_input.mjs", "gripper_preview.mjs",
            "start_positions.mjs",
            "hud_preview.html",
        ):
            raise web.HTTPNotFound()
        return web.FileResponse(ROOT / "web" / name)

    async def quest_page(request):
        return web.FileResponse(ROOT / "web/quest.html")

    async def robot_asset(request):
        return robot_asset_response(ROOT, request)

    async def session_info(request):
        return web.json_response({"token": token})

    register_asset_routes(app, ROOT)
    app.add_routes(
        [
            web.get("/", page),
            web.get("/quest", quest_page),
            web.get("/static/{name}", page),
            web.get("/static/robot-assets/{package}/{asset}", robot_asset),
            web.get("/api/status", status),
            web.get("/api/headset/status", headset_status_handler),
            web.post("/api/recording/capture", capture_recording_diagnostic),
            web.get("/hud-preview", hud_preview_page),
            web.get("/asset-preview", asset_preview),
            web.get("/api/grippers/preview", preview_grippers),
            web.get("/monitor", monitor_page),
            web.get("/api/monitor/status", monitor_status),
            web.post("/api/monitor/connect", monitor_connect),
            web.post("/api/monitor/disconnect", monitor_disconnect),
            web.post("/api/start-positions/action", home_action),
            web.post("/api/start-positions/heartbeat", home_heartbeat),
            web.get("/api/session", session_info),
            web.get("/ws/quest", websocket),
            web.post("/api/quest/connect-usb", connect_usb),
            web.post("/api/e6/check", check_e6),
            web.post("/api/quest/save", save_observations),
        ]
    )
    lan = QuestLan(ROOT, session_info, websocket)
    app[LAN_KEY] = lan

    async def renew_lan(request):
        if quest.connected:
            raise web.HTTPConflict(text="End the current XR input session before renewing pairing")
        return await lan.renew()

    app.router.add_post("/api/quest/lan/renew", renew_lan)
    app.add_routes(
        [
            web.get("/api/quest/lan", lan.status_handler),
            web.post("/api/quest/lan/enable", lan.enable_handler),
        ]
    )
    app.on_cleanup.append(lan.cleanup)

    async def prepare_one(side):
        box = trial_boxes[side]
        current = box["controller"]
        if current:
            if current.state == "FAULT" and not (current.stop_evidence or {}).get("feedback_confirmed"):
                raise ValueError(f"{side}: Stop has not been confirmed")
            if current.state == "PREPARED":
                await current.prepare()
                return current.snapshot()
            await current.close()
            await box["reader"].stop()
            box["driver"].close()
            group.controllers.pop(side, None)
            box.update(controller=None, reader=None, driver=None)
        await monitors.disconnect(side)
        from dobot_safe_sdk import BoundDobotDashboard
        from e6_feedback import FeedbackReader
        from left_z_trial import LeftZTrial

        cfg = load_device_config(ROOT)
        ip, local_ip = e6_targets(cfg, [side])[0]
        driver = RecordedDriver(await asyncio.to_thread(BoundDobotDashboard, ip, local_ip), recorder.for_side(side))
        reader = FeedbackReader(ip, local_ip, f"{side}@{ip}-via-{local_ip}", publish_hz=50)
        trial = LeftZTrial(driver, reader, quest.snapshot, ranged_profile=ranged_profile,
                          runtime_dir=trial_runtime_dir or ROOT / "runtime", side=side,
                          direction_calibration=direction_calibrations[side] if ranged_profile else None,
                          motion_permit=lambda: group.motion_allowed(side))
        try:
            await reader.start()
            for _ in range(40):
                if reader.snapshot().get("valid"):
                    break
                await asyncio.sleep(.05)
            await trial.prepare()
            await trial.start()
        except Exception:
            await trial.close()
            await reader.stop()
            driver.close()
            raise
        box.update(controller=trial, reader=reader, driver=driver)
        group.controllers[side] = trial
        return trial.snapshot()

    async def detected_sides():
        cfg = load_device_config(ROOT)
        configured = validate_device_config(cfg)
        if not configured:
            raise ValueError("DEVICE_NOT_CONFIGURED: initialize runtime/devices.local.json")
        for side in configured:
            if trial_boxes[side]["reader"] is None:
                ip, source = e6_targets(cfg, [side])[0]
                try:
                    await monitors.connect(side, ip, source)
                except (OSError, ValueError):
                    pass
        deadline = time.monotonic()+2
        while time.monotonic() < deadline:
            if all(home_feedback(s).get("connected") for s in configured):
                break
            await asyncio.sleep(.05)
        return tuple(s for s in configured if home_feedback(s).get("connected"))

    def require_prepared_request(side):
        if side not in ("left", "right", "both", "auto"):
            raise web.HTTPBadRequest(text="Unknown teleoperation side")
        requested = group.participants if side == "auto" else ("left", "right") if side == "both" else (side,)
        if not requested or requested != group.participants:
            raise web.HTTPConflict(text="Requested arms do not match the prepared group; recheck first")

    def group_response():
        data = group.snapshot()
        data["controls"] = {s: t.snapshot() for s, t in group.controllers.items()}
        if len(group.participants) == 1:
            return {**group.controllers[group.participants[0]].snapshot(), "group": data}
        return {**data, "group": data, "side": "both"}

    async def prepare_trial(request, side=None):
        side = side or request.query.get("side", "auto")
        if side not in ("left", "right", "both", "auto"):
            raise web.HTTPBadRequest(text="Unknown teleoperation side")
        async with trial_lock:
            require_home_idle()
            try:
                group.require_stopped()
            except ValueError as exc:
                raise web.HTTPConflict(text=str(exc)) from exc
            sides = await detected_sides() if side == "auto" else ("left", "right") if side == "both" else (side,)
            if not sides:
                raise ValueError("未发现已连接机械臂；检查电源、网线和源地址")
            group.state, group.reason = "NOT_PREPARED", "CHECKING_ALL_PARTICIPANTS"
            results = await asyncio.gather(*(prepare_one(s) for s in sides), return_exceptions=True)
            errors = {s: str(result) for s, result in zip(sides, results) if isinstance(result, BaseException)}
            if errors:
                group.participants = tuple(sides)
                group.reason = "PREFLIGHT_FAILED: "+str(errors)
                raise ValueError(group.reason)
            group.set_prepared(sides)
            return web.json_response(group_response())

    async def prepare_left_trial(request):
        return await prepare_trial(request, "left")

    async def arm_trial(request, side=None):
        side = side or request.query.get("side", "auto")
        if request is not None:
            recorder.request("arm", "pc_console")
        try:
            async with trial_lock:
                require_home_idle()
                require_prepared_request(side)
                await group.arm()
                return web.json_response(group_response())
        except Exception as exc:
            recorder.rejected("arm", exc)
            raise

    async def arm_left_trial(request):
        return await arm_trial(request, "left")

    async def stop_left_trial(request):
        if request is not None:
            recorder.request("stop", "pc_console")
        outcomes = await asyncio.gather(group.stop(), homes.stop(), return_exceptions=True)
        errors = [str(result) for result in outcomes if isinstance(result, BaseException)]
        result = group.snapshot()
        if errors or homes.fault_latched:
            result["state"] = "FAULT"
            result["stop_evidence"] = {**(result.get("stop_evidence") or {}),
                                       "feedback_confirmed": False, "home_errors": errors}
        recorder.finish_deadline = time.monotonic() + 1
        return web.json_response({**result, "controls": headset_snapshot()["controls"],
                                  "start_positions": homes.snapshot()})

    async def cleanup_trial(app):
        await group.close()
        await homes.close()
        await monitors.close()
        for box in trial_boxes.values():
            if box["controller"]:
                await box["controller"].close()
            if box["reader"]:
                await box["reader"].stop()
            if box["driver"]:
                box["driver"].close()
        await recorder.close()

    app.add_routes(
        [
            web.post("/api/left-trial/prepare", prepare_left_trial),
            web.post("/api/left-trial/arm", arm_left_trial),
            web.post("/api/left-trial/stop", stop_left_trial),
            web.post("/api/teleop/prepare", prepare_trial),
            web.post("/api/teleop/arm", arm_trial),
            web.post("/api/teleop/stop", stop_left_trial),
        ]
    )
    app.on_cleanup.append(cleanup_trial)
    return app


if __name__ == "__main__":
    print(
        f"Quest/E6 console: http://127.0.0.1:{PORT} | No automatic robot connections or motion commands.",
        flush=True,
    )
    web.run_app(make_app(ranged_profile_path=os.environ.get("E6_RANGEDIK_PROFILE")),
                host="127.0.0.1", port=PORT, access_log=None, print=None)
