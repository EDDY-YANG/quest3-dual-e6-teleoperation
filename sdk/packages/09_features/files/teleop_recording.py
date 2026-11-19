"""Local episode recording with a bounded non-blocking queue and batched writes.

Quest frames are captured on receipt; existing robot feedback is sampled at
20 Hz. No robot socket is created. Only an explicit arm request opens a file.
Synthetic tests must disable recording or pass a temporary recordings root.
"""

import asyncio
import copy
import hashlib
import itertools
import json
import queue
import re
import threading
import time
from collections import Counter, deque
from datetime import datetime, timedelta, timezone
from pathlib import Path

CST = timezone(timedelta(hours=8))


def timestamp():
    return datetime.now(CST).isoformat(timespec="milliseconds")


def exception_chain(exc):
    chain, seen = [], set()
    while exc is not None and id(exc) not in seen:
        seen.add(id(exc))
        chain.append({"type": type(exc).__name__, "message": str(exc)})
        exc = exc.__cause__ or exc.__context__
    return chain


def _analyze_lines(lines):
    """Stream the saved log; ACK, measured arrival and stop remain separate."""
    counts, commands = Counter(), Counter()
    first_ns = last_ns = None
    first_xyz = last_xyz = last_q = target_q = target_xyz = target_pose = None
    failures, acknowledgements, stop_results = [], 0, []
    drops = 0
    pre_roll_records = 0
    tracking_window_deg = None
    for line in lines:
        item = json.loads(line)
        if item.get("pre_roll"):
            pre_roll_records += 1
            continue  # A preceding attempt's events are context, never this attempt's success.
        kind, data = item["kind"], item.get("data") or {}
        if kind == "control_snapshot":
            value = (data.get("limits") or {}).get("tracking_window_deg")
            if type(value) in (int, float) and 0 < value <= 180:
                tracking_window_deg = value
        counts[kind] += 1
        clock = item["pc_perf_ns"]
        first_ns = clock if first_ns is None else min(first_ns, clock)
        last_ns = clock if last_ns is None else max(last_ns, clock)
        if kind == "command_return":
            commands[data["method"]] += 1
            if data["method"] == "ServoJ" and data.get("reply_error_id") == 0:
                target_q = data.get("args", [])[:6]
        if kind == "controller_event":
            if data.get("kind") == "TARGET_ACK":
                acknowledgements += 1
                target_q = data.get("q_deg")
                target_xyz = data.get("z_mm")
                target_pose = data.get("pose_mm_deg")
            if data.get("kind") == "STOP_RESULT":
                evidence = data.get("evidence") or {}
                stop_results.append(evidence)
                normal = evidence.get("reason") in (
                    "GRIP_RELEASED",
                    "OPERATOR_DISARMED",
                    "SERVICE_CLEANUP",
                )
                if not evidence.get("feedback_confirmed") or not normal:
                    failures.append(data)
        if kind in ("command_exception", "action_rejected"):
            failures.append(data)
        if kind == "robot_feedback" and data.get("valid"):
            xyz = data.get("position_mm")
            if xyz:
                first_xyz = xyz if first_xyz is None else first_xyz
                last_xyz = xyz
            last_q = data.get("joint_position_deg") or last_q
        if kind == "session_end":
            drops = data.get("dropped_records", 0)
    error = max(abs(a - b) for a, b in zip(last_q, target_q)) if last_q and target_q else None
    return {
        "scope": "SAVED_OBSERVATIONS_NOT_PHYSICAL_SAFETY_ACCEPTANCE",
        "record_counts": dict(counts),
        "pre_roll_records": pre_roll_records,
        "command_returns": dict(commands),
        "target_ack_events": acknowledgements,
        "observed_span_s": None if first_ns is None else round((last_ns - first_ns) / 1e9, 6),
        "first_actual_xyz_mm": first_xyz,
        "last_actual_xyz_mm": last_xyz,
        "observed_xyz_change_mm": [b - a for a, b in zip(first_xyz, last_xyz)]
        if first_xyz and last_xyz
        else None,
        "last_target_z_mm": target_xyz,
        "last_target_pose_mm_deg": target_pose,
        "last_target_joint_deg": target_q,
        "last_actual_joint_deg": last_q,
        "last_observed_joint_error_deg": error,
        "last_observed_within_0_03_deg": None if error is None else error <= 0.03,
        "last_observed_within_tracking_window_10_deg": None if error is None else error <= 10,
        "observed_tracking_window_deg": tracking_window_deg,
        "last_observed_within_tracking_window": (
            None if error is None or tracking_window_deg is None else error <= tracking_window_deg
        ),
        "failure_events": failures[-20:],
        "stop_results": stop_results[-20:],
        "dropped_records": drops,
        "sampling": "Every accepted Quest frame; existing robot/control state at up to 20 Hz. Missing samples are not reconstructed.",
    }


def analyze_log(path):
    with Path(path).open(encoding="utf-8") as stream:
        return _analyze_lines(stream)


class _Episode:
    def __init__(self, root, side="left"):
        root.mkdir(parents=True, exist_ok=True)
        if side not in ("left", "right"):
            raise ValueError("Invalid recording arm side")
        name = datetime.now(CST).strftime("%Y-%m-%d-%H-%M-%S") + "-" + side
        for suffix in range(100):
            self.folder = root / (name if suffix == 0 else f"{name}-v{suffix + 1}")
            try:
                self.folder.mkdir()
                break
            except FileExistsError:
                continue
        else:
            raise FileExistsError("Too many recording names in the same second")
        self.path = self.folder / "events.jsonl"
        self.stream = self.path.open("x", encoding="utf-8", buffering=65536)
        self.queue = queue.Queue(maxsize=2048)
        self.finished = threading.Event()
        self.dropped = self.written = 0
        self.error = None
        self.end_reason = None
        self.thread = threading.Thread(target=self._write, name="e6-record-writer", daemon=True)
        self.thread.start()

    def put(self, item):
        if self.error:
            self.dropped += 1
            return
        try:
            self.queue.put_nowait(item)
        except queue.Full:
            self.dropped += 1

    def _line(self, item):
        self.stream.write(
            json.dumps(item, ensure_ascii=True, separators=(",", ":"), allow_nan=False) + "\n"
        )
        self.written += 1

    def _write(self):
        last_flush = time.perf_counter()
        try:
            while not self.finished.is_set() or not self.queue.empty():
                try:
                    self._line(self.queue.get(timeout=0.1))
                except queue.Empty:
                    pass
                if time.perf_counter() - last_flush >= 0.5:
                    self.stream.flush()
                    last_flush = time.perf_counter()
            self._line(
                {
                    "kind": "session_end",
                    "pc_time": timestamp(),
                    "pc_perf_ns": time.perf_counter_ns(),
                    "data": {"reason": self.end_reason, "dropped_records": self.dropped},
                }
            )
            self.stream.close()
            result = analyze_log(self.path)
            result.update(writer_error=self.error, saved_at=timestamp())
            (self.folder / "analysis.json").write_text(
                json.dumps(result, indent=2), encoding="utf-8"
            )
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
            self.stream.close()


class TeleopRecorder:
    def __init__(self, root, source_root, quest_provider, trial_provider, *, enabled=True, side="left"):
        self.side = side
        self.root, self.source_root = Path(root), Path(source_root)
        self.quest_provider, self.trial_provider = quest_provider, trial_provider
        self.enabled = enabled
        self.active = self.last = None
        self.recent = deque(maxlen=512)
        self.closed = deque(maxlen=16)
        self.lock = threading.RLock()
        self.task = None
        self.finish_deadline = None
        self.last_error = None
        self._event_keys = deque(maxlen=256)

    def capture(self, kind, data):
        if not self.enabled:
            return
        try:
            item = {
                "kind": kind,
                "pc_time": timestamp(),
                "pc_perf_ns": time.perf_counter_ns(),
                "data": copy.deepcopy(data),
            }
            with self.lock:
                self.recent.append(item)
                if self.active:
                    self.active.put(item)
        except Exception as exc:
            self.last_error = str(exc)

    def begin(self, source):
        if not self.enabled:
            return
        with self.lock:
            if self.active and self.finish_deadline is None:
                self.capture("repeated_start_request", {"source": source})
                return
            if self.active:
                self.finish("NEW_EXPLICIT_ATTEMPT")
            try:
                side = getattr(self.trial_provider(), "side", self.side)
                episode = _Episode(self.root, side)
            except OSError as exc:
                self.last_error = str(exc)
                raise RuntimeError(
                    "Cannot create the requested local recording; trial was not started"
                ) from exc
            self.active = self.last = episode
            self.finish_deadline = None
            names = (
                "server.py",
                "teleop_group.py",
                "robot_stop_state.py",
                "left_z_trial.py",
                "dobot_safe_sdk.py",
                "e6_feedback.py",
                "headset_bridge.py",
                "web/quest.js",
                "web/grip_gate.mjs",
                "relative_pose.py",
                "tool_orientation_cone.py",
                "controller_kinematics.py",
                "kinematics_validation.py",
                "joint_reference_qp.py",
                "ranged_ik_profile.py",
                "ranged_ik_backend.py",
                "tools/rangedik_native/native_bridge.rs",
                "tools/rangedik_native/source-patch-receipt.json",
                "ranged_ik_adapter.py",
                "ranged_ik_model.py",
                "direction_calibration.py",
                "profiles/rangedik_world_translation.json",
                "profiles/rangedik_world_anchor.json",
                "profiles/rangedik_unloaded_wide.json",
                "web/quest_hud.mjs",
                "web/direction_calibration.mjs",
                "web/tracking_warning.mjs",
                "web/e6_urdf_view.mjs",
                "web/robot-assets/e6/model.json",
                "web/robot-assets/e6/manifest.json",
                "teleop_recording.py",
            )
            metadata = {
                "schema": 1,
                "source": source,
                "side": side,
                "started_at": timestamp(),
                "units": {
                    "robot_position": "mm",
                    "robot_angles": "deg",
                    "quest_position": "m",
                    "quaternion": "xyzw",
                },
                "disk_source_sha256_at_start": {
                    n: hashlib.sha256((self.source_root / n).read_bytes()).hexdigest()
                    for n in names
                    if (self.source_root / n).is_file()
                },
                "robot_sampling_hz": 20,
                "flush_interval_s": 0.5,
                "pre_roll_s": 5,
                "data_loss_policy": "bounded queue; count loss explicitly; never invent missing samples",
            }
            try:
                (episode.folder / "metadata.json").write_text(
                    json.dumps(metadata, indent=2), encoding="utf-8"
                )
            except OSError as exc:
                self.last_error = str(exc)
                self.finish("METADATA_WRITE_FAILED")
                raise RuntimeError(
                    "Recording metadata could not be saved; trial was not started"
                ) from exc
            cutoff = time.perf_counter_ns() - 5_000_000_000
            for item in self.recent:
                if item["pc_perf_ns"] >= cutoff:
                    episode.put({**item, "pre_roll": True})
            self.capture("session_start", metadata)

    def request(self, action, source):
        if action == "arm":
            self.begin(source)
        elif action == "stop" and self.enabled and self.active is None:
            self.begin("EXPLICIT_STOP_ONLY:" + source)
        self.capture("control_request", {"action": action, "source": source})

    def rejected(self, action, error):
        self.capture(
            "action_rejected", {"action": action, "exception_chain": exception_chain(error)}
        )
        if action in ("arm", "stop"):
            self.finish_deadline = time.monotonic() + 1

    def finish(self, reason):
        with self.lock:
            episode, self.active = self.active, None
            self.finish_deadline = None
            if episode:
                episode.end_reason = reason
                episode.finished.set()
                self.closed.append(episode)

    def snapshot(self):
        episode = self.active or self.last
        error = (episode.error if episode else None) or self.last_error
        state = (
            "ERROR"
            if error
            else "RECORDING"
            if self.active
            else (
                "FINALIZING"
                if episode and episode.thread.is_alive()
                else "SAVED"
                if episode
                else "READY"
            )
        )
        return {
            "state": state,
            "enabled": self.enabled,
            "active": self.active is not None,
            "directory": str(episode.folder) if episode else str(self.root),
            "dropped_records": episode.dropped if episode else 0,
            "writer_error": error,
            "robot_sample_hz": 20,
            "quest_frames": "every accepted frame",
            "flush_interval_s": 0.5,
        }

    async def start(self):
        if self.enabled:
            self.task = asyncio.create_task(self._sample(), name="teleop-record-sampler")

    async def _sample(self):
        last_feedback, last_control = None, None
        while True:
            try:
                trial = self.trial_provider()
                if trial is not None:
                    feedback, control = trial.feedback.snapshot(), trial.snapshot()
                    signature = (
                        feedback.get("host_monotonic_s"),
                        feedback.get("valid"),
                        feedback.get("connected"),
                        feedback.get("reason"),
                    )
                    if signature != last_feedback:
                        self.capture("robot_feedback", feedback)
                        last_feedback = signature
                    self.capture(
                        "control_snapshot",
                        {
                            k: control.get(k)
                            for k in (
                                "state",
                                "armed",
                                "reason",
                                "limits",
                                "center",
                                "target",
                                "pending",
                                "stop_evidence",
                                "command_timings",
                                "incremental_state",
                                "kinematics",
                                "ik_evidence",
                                "ranged_ik",
                                "qp_evidence",
                                "requested_target",
                                "reference_state",
                                "tracking_evidence",
                                "tracking_warning",
                                "direction_calibration",
                                "tool_orientation_cone",
                                "orientation_tracking",
                                "clutch",
                            )
                        },
                    )
                    for event in control.get("events") or []:
                        key = json.dumps(event, sort_keys=True)
                        if key not in self._event_keys:
                            self._event_keys.append(key)
                            self.capture("controller_event", event)
                    state = control.get("state")
                    if (
                        self.active
                        and not control.get("armed")
                        and state in ("FAULT", "DISARMED")
                        and self.finish_deadline is None
                    ):
                        self.finish_deadline = time.monotonic() + 1
                    last_control = state
                if (
                    self.active
                    and self.finish_deadline is not None
                    and time.monotonic() >= self.finish_deadline
                ):
                    self.finish(last_control or "REQUEST_REJECTED")
            except Exception as exc:
                self.last_error = str(exc)
            await asyncio.sleep(0.05)

    async def close(self):
        if self.task:
            self.task.cancel()
            try:
                await self.task
            except asyncio.CancelledError:
                pass
        self.finish("SERVICE_CLEANUP")
        for episode in self.closed:
            await asyncio.to_thread(episode.thread.join, 5)


class TeleopRecorderGroup:
    """Separate arm logs with one aggregated UI status; no mixed robot streams."""
    def __init__(self, recorders, sides_provider):
        self.recorders, self.sides_provider = recorders, sides_provider

    def for_side(self, side):
        return self.recorders[side]

    def _selected(self):
        sides = tuple(self.sides_provider())
        return sides or ("left",)

    def capture(self, kind, data):
        side = data.get("side") if isinstance(data, dict) else None
        for selected in ((side,) if side in self.recorders else self.recorders):
            self.recorders[selected].capture(kind, data)

    def request(self, action, source):
        for side in self._selected():
            self.recorders[side].request(action, source)

    def rejected(self, action, error):
        for side in self._selected():
            self.recorders[side].rejected(action, error)

    def begin(self, source):
        for side in self._selected():
            self.recorders[side].begin(source)

    @property
    def finish_deadline(self):
        return None

    @finish_deadline.setter
    def finish_deadline(self, value):
        for recorder in self.recorders.values():
            if recorder.active is not None:
                recorder.finish_deadline = value

    def snapshot(self):
        views = {side: recorder.snapshot() for side, recorder in self.recorders.items()}
        primary = views[self._selected()[0]]
        active = any(v["active"] for v in views.values())
        errors = [v["writer_error"] for v in views.values() if v["writer_error"]]
        return {**primary, "arms": views, "active": active,
                "state": "RECORDING" if active else primary["state"],
                "writer_error": "; ".join(errors) if errors else None,
                "dropped_records": sum(v["dropped_records"] for v in views.values())}

    async def start(self):
        await asyncio.gather(*(r.start() for r in self.recorders.values()))

    async def close(self):
        await asyncio.gather(*(r.close() for r in self.recorders.values()))


class RecordedDriver:
    """Transparent call/return observer: same arguments, result, exception and count."""

    METHODS = frozenset(
        {
            "RobotMode",
            "GetAngle",
            "GetPose",
            "GetErrorID",
            "GetCurrentCommandID",
            "InverseKin",
            "PositiveKin",
            "ServoJ",
            "MovJ",
            "Stop",
            "RecoverStop",
            "RecoverStop",
            "EnableRobot",
            "RecoveryClearError",
            "DisableRobot",
            "SpeedFactor",
        }
    )

    def __init__(self, driver, recorder):
        self.driver, self.recorder = driver, recorder
        self.ids = itertools.count(1)

    def __getattr__(self, name):
        value = getattr(self.driver, name)
        if name not in self.METHODS or not callable(value):
            return value

        def call(*args, **kwargs):
            request_id = next(self.ids)
            fields = {"id": request_id, "method": name, "args": args, "kwargs": kwargs}
            started = time.perf_counter_ns()
            self.recorder.capture("command_call", fields)
            try:
                result = value(*args, **kwargs)
            except Exception as exc:
                self.recorder.capture(
                    "command_exception",
                    {
                        **fields,
                        "exception_chain": exception_chain(exc),
                        "duration_ms": (time.perf_counter_ns() - started) / 1e6,
                    },
                )
                raise
            self.recorder.capture(
                "command_return",
                {
                    **fields,
                    "raw_reply": result,
                    "reply_error_id": (
                        int(match[1]) if (match := re.match(r"\s*(-?\d+)\s*,", result)) else None
                    )
                    if isinstance(result, str)
                    else None,
                    "duration_ms": (time.perf_counter_ns() - started) / 1e6,
                },
            )
            return result

        return call
