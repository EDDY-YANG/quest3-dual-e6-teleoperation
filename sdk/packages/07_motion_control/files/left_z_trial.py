"""Explicit selected-arm relative SE(3) teleoperation. Historical module name retained.

This controller has no PC Cartesian translation range/speed limits. RangedIK
uses a separate 45-degree downward tool-axis cone. Its
joint-jump guard is not a safety-rated workspace, speed or stop controller.
Socket Stop latency is limited by the driver's current transaction. Real stop
confirmation is reported separately from its acknowledgement.
"""

import asyncio
import json
import math
import time
from collections import deque
from contextlib import suppress
from enum import Enum

from dobot_safe_sdk import STOP_REPLY_TIMEOUT_S, require_success
from robot_stop_state import stop_feedback
from e6_feedback import euler_xyz_deg_to_xyzw
from headset_bridge import QUEST_ADMISSION_MAX_AGE_MS, QUEST_MOTION_MAX_AGE_MS
from relative_pose import grip_direction_mapping, incremental_pose_target, anchored_position_target, anchored_pose_target
from kinematics_validation import commission_model, pose_error
from controller_kinematics import MAX_IK_JOINT_DELTA_DEG
from tool_orientation_cone import MAX_TILT_DEG, INITIAL_TILT_DEG, in_cone, tilt_deg, project_target, joint_path_check, configuration
from joint_reference_qp import (
    ReferenceQP,
    FIRST_DT_S,
    REFERENCE_MAX_VELOCITY_DEG_S,
    REFERENCE_MAX_ACCELERATION_DEG_S2,
    REFERENCE_MAX_TRACKING_ERROR_DEG,
)

PENDING_TARGET_TIMEOUT_S = 10.0
TRACKING_WINDOW_DEG = 30.0
# Independent send guard; never use this to shrink the IK search interval.
DIRECT_MAX_TRACKING_ERROR_DEG = 50.0


class TrialFault(RuntimeError):
    pass


class ConeCandidateRejected(TrialFault):
    """Reject one proposal without committing it or cancelling the Grip session."""


class GripPhase(Enum):
    RELEASED = "released"
    WAIT_RELEASE = "waiting_for_physical_release"
    HELD = "held"


def _six(payload):
    try:
        values = [float(x.strip()) for x in payload.split(",")]
    except (ValueError, AttributeError) as exc:
        raise TrialFault("Expected six numeric values from the controller") from exc
    if len(values) != 6 or not all(math.isfinite(x) for x in values):
        raise TrialFault("Invalid six-element controller reply")
    return values


def _angle_deg(a, b):
    na, nb = math.sqrt(sum(x * x for x in a)), math.sqrt(sum(x * x for x in b))
    if na < 0.9 or nb < 0.9:
        raise TrialFault("Invalid quaternion norm")
    dot = min(1.0, abs(sum(x * y for x, y in zip(a, b)) / (na * nb)))
    return math.degrees(2 * math.acos(dot))


def _no_errors(value):
    if isinstance(value, list):
        return all(_no_errors(x) for x in value)
    return type(value) is int and value == 0


def _queue_id(payload):
    if (
        not isinstance(payload, str)
        or not payload.strip().isascii()
        or not payload.strip().isdecimal()
    ):
        raise TrialFault("INVALID_QUEUE_COMMAND_ID")
    value = int(payload.strip())
    if not 0 <= value < 2**64:
        raise TrialFault("INVALID_QUEUE_COMMAND_ID")
    return value


class LeftZTrial:
    """Inject a bound arm driver, matching FeedbackReader and Quest snapshot provider.

    API: await start(), prepare(), arm(), disarm(), close(); snapshot() is sync.
    Only explicit arm() can call EnableRobot. Grip requires an observed release
    after arm/fault/stop before a new press. Ordinary release stops but does not
    disable the drives. Faults cancel generation and disarm permanently until
    another explicit prepare/arm cycle.
    """

    def __init__(self, driver, feedback_reader, quest_provider, *, ranged_profile=None, runtime_dir=None, direction_calibration=None, side="left", motion_permit=None):
        if not callable(quest_provider):
            raise ValueError("quest_provider must be a synchronous snapshot callable")
        if side not in ("left", "right") or not str(feedback_reader.robot_id).lower().startswith(side):
            raise ValueError("Selected arm does not match the bound feedback identity")
        if direction_calibration is not None and direction_calibration.side != side:
            raise ValueError("Selected arm does not match its direction calibration")
        self.side = side
        self.motion_permit = motion_permit or (lambda: True)
        self.driver, self.feedback, self.quest_provider = driver, feedback_reader, quest_provider
        self.state, self.reason = "DISARMED", "EXPLICIT_ARM_REQUIRED"
        self.armed = False
        self.center = None
        self.target = None
        self.pending = None
        self.stop_evidence = None
        self._prepared_context = None
        self._quest_context = None
        self._anchor_y = self._anchor_z = None
        self._anchor_hand = self._anchor_robot = None
        self.tracking_evidence = None
        self.tracking_warning = None
        self._tracking_warning_last_event = -math.inf
        self._queue_last_seen = self._queue_last_submitted = None
        self._last_queue_query = -math.inf
        self._queue_task = None
        self._queue_task_generation = None
        self._generation = 0
        self._last_command = -math.inf
        self._io_lock = asyncio.Lock()
        self._operation_lock = asyncio.Lock()
        self._loop_task = self._motion_task = self._stop_task = None
        self._closed = False
        self._events = deque(maxlen=80)
        self._command_timings = {}
        self.preflight = {"status": "NOT_RUN"}
        self._release_after = -math.inf
        self._direction_mapping = None
        self._previous_hand = None
        self._previous_hand_seq = -1
        self._last_window_at = -math.inf
        self.last_increment_window = None
        self._kinematics = None
        self.kinematics_evidence = {"status": "NOT_VALIDATED"}
        self.last_ik_evidence = None
        self._reference = None
        self._last_reference_at = None
        self.qp_evidence = None
        self.requested_target = None
        self._ranged_profile = ranged_profile
        self._ranged_solver = None
        self._runtime_dir = runtime_dir
        self.direction_calibration = direction_calibration
        if self._anchored_world_position and direction_calibration is None:
            raise ValueError("Grip-anchored world position requires explicit direction calibration")
        self._pause_latched = False
        self._first_grip_pending = True
        self.cone_evidence = {}
        self.orientation_evidence = None
        self._last_cone_rejection_at = -math.inf

    @property
    def _cone_enabled(self):
        return self._ranged_profile is not None

    def _check_measured_cone(self, feedback, *, initial=False):
        if not self._cone_enabled:
            return
        angle = tilt_deg(feedback["pose_mm_deg"])
        limit = INITIAL_TILT_DEG if initial else MAX_TILT_DEG
        self.cone_evidence["measured_tilt_deg"] = angle
        if not in_cone(feedback["pose_mm_deg"], limit):
            raise TrialFault("INITIAL_TOOL_TILT_EXCEEDS_5_DEG" if initial else "MEASURED_TOOL_TILT_EXCEEDS_45_DEG")

    def _cone_snapshot(self, feedback):
        angle = None
        age = time.monotonic() - feedback.get("host_monotonic_s", -math.inf)
        if (self._cone_enabled and feedback.get("valid") and feedback.get("connected")
                and 0 <= age < .2 and feedback.get("pose_mm_deg") is not None):
            angle = tilt_deg(feedback["pose_mm_deg"])
        return {**configuration(), "enabled": self._cone_enabled,
                "measured_tilt_deg": angle,
                "inside_cone": angle is not None and in_cone(feedback["pose_mm_deg"]),
                "initial_ready": angle is not None and in_cone(feedback["pose_mm_deg"], INITIAL_TILT_DEG),
                "first_grip_pending": self._first_grip_pending,
                "evidence": dict(self.cone_evidence)}

    @property
    def _direct_ranged_output(self):
        return self._ranged_profile is not None and self._ranged_profile.output_mode == "direct"

    @property
    def _anchored_world_position(self):
        return (self._ranged_profile is not None and
                self._ranged_profile.position_target_policy == "GRIP_ANCHORED_WORLD")

    @property
    def _anchored_world_orientation(self):
        return (self._ranged_profile is not None and
                self._ranged_profile.orientation_target_policy == "GRIP_ANCHORED_WORLD")

    def _event(self, kind, **details):
        self._events.append({"host_monotonic_s": time.monotonic(), "kind": kind, **details})

    def _report_tracking_lag(self, errors, observed_at):
        """Advisory only. Does not change arm/state/queue admission or targets."""
        active = max(errors) > TRACKING_WINDOW_DEG
        was_active = bool(self.tracking_warning and self.tracking_warning["active"])
        worst = max(range(6), key=errors.__getitem__)
        warning = {
            "active": active, "code": "TRACKING_LAG_OVER_THRESHOLD",
            "threshold_deg": TRACKING_WINDOW_DEG, "max_error_deg": errors[worst],
            "joint_errors_deg": errors,
            "joints_exceeding": [i+1 for i, error in enumerate(errors) if error > TRACKING_WINDOW_DEG],
            "worst_joint": worst+1, "blocking": False, "host_monotonic_s": observed_at,
            "message": (f"跟随滞后：J{worst+1}误差{errors[worst]:.1f}°，超过{TRACKING_WINDOW_DEG:g}°；继续遥操作"
                        if active else "跟随误差已回到告警阈值内"),
        }
        self.tracking_warning = warning
        # Publish current values in normal telemetry, throttle disk events.
        if active and (not was_active or observed_at-self._tracking_warning_last_event >= 2.0):
            self._event("TRACKING_LAG_WARNING", **warning)
            self._tracking_warning_last_event = observed_at
        elif was_active and not active:
            self._event("TRACKING_LAG_CLEARED", **warning)

    def _robot(self):
        p = self.feedback.snapshot()
        now = time.monotonic()
        if not p.get("valid") or not p.get("connected", True):
            raise TrialFault("ROBOT_FEEDBACK_INVALID: " + str(p.get("reason", "unknown")))
        age = now - p.get("host_monotonic_s", -math.inf)
        if not 0 <= age <= 0.2:
            raise TrialFault("ROBOT_FEEDBACK_STALE")
        if p.get("error_status") != 0 or p.get("user") != 0 or p.get("tool") != 0:
            raise TrialFault("ROBOT_ERROR_OR_UNACCEPTED_FRAME")
        if p.get("robot_mode") not in (4, 5, 7):
            raise TrialFault("ROBOT_MODE_NOT_ALLOWED")
        return p

    @staticmethod
    def _robot_context(p):
        return (p["robot_id"], p["frame_id"], p["user"], p["tool"], p.get("connection_generation"))

    def _input(self, *, max_age_s=QUEST_MOTION_MAX_AGE_MS / 1000):
        q = self.quest_provider()
        frame = q.get("frame") or {}
        now = time.monotonic()
        if (
            not q.get("connected")
            or not 0 <= now - frame.get("host_monotonic_s", -math.inf) <= max_age_s
        ):
            raise TrialFault("QUEST_DISCONNECTED_OR_STALE")
        if frame.get("visibility") != "visible":
            raise TrialFault("QUEST_NOT_VISIBLE")
        if (frame.get("diagnostics") or {}).get("requested_mode") != "immersive-ar":
            raise TrialFault("PASSTHROUGH_AR_SESSION_REQUIRED")
        hand = (frame.get("hands") or {}).get(self.side) or {}
        pose = hand.get("pose") or {}
        gp = hand.get("gamepad") or {}
        if frame.get("input_policy") != "continuous-pose-grip-v2":
            raise TrialFault("REFRESH_QUEST_PAGE_FOR_CONTINUOUS_POSE_V2")
        if not hand.get("is_controller") or hand.get("tracking_valid") is not True:
            raise TrialFault(f"{self.side.upper()}_CONTROLLER_TRACKING_INVALID")
        buttons = gp.get("buttons") or []
        if not gp.get("connected") or gp.get("mapping") != "xr-standard" or len(buttons) < 2:
            raise TrialFault(f"{self.side.upper()}_GRIP_UNAVAILABLE")
        grip = buttons[1].get("value")
        if type(grip) not in (int, float) or not math.isfinite(grip) or not 0 <= grip <= 1:
            raise TrialFault("INVALID_GRIP")
        context = (q.get("session"), frame.get("reference_space"), frame.get("reference_epoch"))
        held = hand.get("grip_held")
        if type(held) is not bool:
            raise TrialFault("GRIP_GATE_STATE_MISSING")
        if (
            (held and grip <= 0.2)
            or not q.get("valid", {}).get(self.side)
            or pose.get("emulated_position") is not False
        ):
            raise TrialFault("CONTINUOUS_CONTROLLER_POSE_INVALID")
        position, orientation = pose.get("position_m"), pose.get("quaternion_xyzw")
        if (
            not isinstance(position, list)
            or len(position) != 3
            or not isinstance(orientation, list)
            or len(orientation) != 4
            or any(
                type(x) not in (int, float) or not math.isfinite(x) for x in position + orientation
            )
        ):
            raise TrialFault("INVALID_HAND_POSE")
        return (
            GripPhase.HELD
            if held
            else GripPhase.RELEASED
            if grip <= 0.2
            else GripPhase.WAIT_RELEASE,
            {"position_m": list(position), "quaternion_xyzw": list(orientation)},
            context,
        )

    async def _call(self, method, *args, guard=None, **kwargs):
        requested = time.monotonic()
        async with self._io_lock:
            if guard is not None:
                guard()
            started = time.monotonic()
            result = "UNCONFIRMED"
            try:
                raw = await asyncio.to_thread(getattr(self.driver, method), *args, **kwargs)
                reply = require_success(raw)
                result = "ACKNOWLEDGED"
                return reply
            finally:
                finished = time.monotonic()
                self._command_timings[method] = {
                    "requested_at": requested,
                    "finished_at": finished,
                    "queue_wait_ms": round((started - requested) * 1000, 3),
                    "request_reply_ms": round((finished - started) * 1000, 3),
                    "result": result,
                    "scope": "PC_REQUEST_TO_REPLY_NOT_PHYSICAL_COMPLETION",
                }

    async def start(self):
        if self._loop_task and not self._loop_task.done():
            return
        self._closed = False
        self._loop_task = asyncio.create_task(self._loop(), name="left-z-trial-watchdog")

    async def prepare(self):
        async with self._operation_lock:
            if self.armed or self.state == "STOPPING":
                raise TrialFault("Disarm and confirm stop before preparing")
            try:
                self.preflight = {"status": "CHECKING", "motion_sent": False}
                p = self._robot()
                context = self._robot_context(p)
                if p["robot_mode"] not in (4, 5):
                    raise TrialFault("ROBOT_NOT_IDLE")
                mode = int((await self._call("RobotMode"))["payload"])
                if mode not in (4, 5):
                    raise TrialFault("COMMAND_MODE_NOT_IDLE")
                joints = _six((await self._call("GetAngle"))["payload"])
                pose = _six((await self._call("GetPose", user=0, tool=0))["payload"])
                alarms = json.loads((await self._call("GetErrorID"))["payload"])
                if not _no_errors(alarms):
                    raise TrialFault("CONTROLLER_ALARMS_PRESENT")
                p = self._robot()
                if max(abs(a - b) for a, b in zip(joints, p["joint_position_deg"])) > 0.1:
                    raise TrialFault("JOINT_QUERY_FEEDBACK_MISMATCH")
                if max(abs(a - b) for a, b in zip(pose[:3], p["position_mm"])) > 0.5:
                    raise TrialFault("POSE_QUERY_FEEDBACK_MISMATCH")
                if _angle_deg(euler_xyz_deg_to_xyzw(pose[3:]), p["quaternion_xyzw"]) > 1:
                    raise TrialFault("ORIENTATION_QUERY_FEEDBACK_MISMATCH")
                solved = None
                current_fk_evidence = None
                if self._ranged_profile is not None:
                    # RangedIK does not execute vendor IK. Validate the current
                    # measured joint/TCP pair through the actual controller FK,
                    # then commission and cross-check the native model below.
                    # Use one feedback packet as the comparison baseline.
                    measured_q = list(p["joint_position_deg"])
                    measured_pose = list(p["pose_mm_deg"])
                    current_fk = await self._query_fk(measured_q)
                    fk_error = pose_error(measured_pose, current_fk)
                    current_fk_evidence = {
                        "measured_joint_deg": measured_q,
                        "measured_pose_mm_deg": measured_pose,
                        "controller_fk_pose_mm_deg": current_fk,
                        "position_error_mm": fk_error[0],
                        "orientation_error_deg": fk_error[1],
                    }
                    if fk_error[0] > 0.05 or fk_error[1] > 0.05:
                        raise TrialFault("CURRENT_POSE_CONTROLLER_FK_MISMATCH")
                else:
                    # Legacy DLS still uses the vendor IK execution path.
                    near = "{" + ",".join(f"{value:.9f}" for value in joints) + "}"
                    solved = _six(
                        (await self._call(
                            "InverseKin", *pose, user=0, tool=0, useJointNear=1, JointNear=near
                        ))["payload"]
                    )
                p = self._robot()
                if self._robot_context(p) != context:
                    raise TrialFault("ROBOT_CONTEXT_CHANGED_DURING_PREFLIGHT")
                if p["robot_mode"] not in (4, 5):
                    raise TrialFault("ROBOT_NOT_IDLE_AFTER_IK_PREFLIGHT")
                if solved is not None and max(abs(a - b) for a, b in zip(solved, p["joint_position_deg"])) > 0.2:
                    raise TrialFault("SAME_POSE_IK_BRANCH_MISMATCH")
                self._kinematics = None
                self.kinematics_evidence = {"status": "CHECKING"}
                self._kinematics, self.kinematics_evidence = await commission_model(
                    self._query_fk, p["joint_position_deg"]
                )
                if self._ranged_profile is not None:
                    from ranged_ik_adapter import RangedIKAdapter
                    if self._ranged_solver:
                        self._ranged_solver.close()
                        self._ranged_solver = None
                    self._ranged_solver = await asyncio.to_thread(
                        RangedIKAdapter, self._kinematics, self._ranged_profile,
                        p["joint_position_deg"], self._runtime_dir, self.side
                    )
                p = self._robot()
                if self._robot_context(p) != context or p["robot_mode"] not in (4, 5):
                    raise TrialFault("ROBOT_CONTEXT_CHANGED_DURING_KINEMATICS_CHECK")
                self._event("KINEMATICS_VALIDATED", **self.kinematics_evidence)
                queue_id = await self._read_queue_id(reset=True)
                self._queue_last_submitted = None
                self.preflight = {
                    "status": "PASSED",
                    "motion_sent": False,
                    "scope": "STATE_QUERIES_CONTROLLER_FK_NATIVE_MODEL_NOT_SERVO_ACCEPTANCE" if self._ranged_profile else "STATE_QUERIES_AND_SAME_POSE_IK_ONLY_NOT_SERVO_ACCEPTANCE",
                    "pose_mm_deg": pose,
                    "ik_joint_deg": solved,
                    "vendor_same_pose_ik_used": self._ranged_profile is None,
                    "current_pose_fk": current_fk_evidence,
                    "robot_context": context,
                    "queue_readback_id": queue_id,
                    "kinematics": dict(self.kinematics_evidence),
                    "ranged_ik": self._ranged_solver.snapshot() if self._ranged_solver else None,
                }
                self._prepared_context = context
                self.state, self.reason = "PREPARED", "EXPLICIT_ARM_REQUIRED"
                self._event("PREPARED", robot_context=self._prepared_context)
            except Exception as exc:
                self.preflight = {"status": "FAILED", "motion_sent": False, "error": str(exc)}
                self._prepared_context = None
                self.state, self.reason = "FAULT", str(exc)
                raise TrialFault(str(exc)) from exc
        return self.snapshot()

    async def _query_fk(self, joints):
        return _six((await self._call("PositiveKin", *joints, user=0, tool=0))["payload"])

    async def _read_queue_id(self, *, reset=False):
        value = await self._queue_read_value()
        self._accept_queue_value(value, reset=reset)
        return value

    async def _queue_read_value(self):
        return _queue_id((await self._call("GetCurrentCommandID"))["payload"])

    def _accept_queue_value(self, value, *, reset=False):
        if not reset and self._queue_last_seen is not None and value < self._queue_last_seen:
            raise TrialFault("CONTROLLER_QUEUE_ID_REGRESSED")
        self._queue_last_seen = value
        self._last_queue_query = time.monotonic()

    def assert_group_ready(self):
        p = self._robot()
        if p["robot_mode"] != 5 or self._robot_context(p) != self._prepared_context:
            raise TrialFault("GROUP_ARM_ROBOT_CONTEXT_CHANGED")
        _, _, context = self._input(max_age_s=QUEST_ADMISSION_MAX_AGE_MS / 1000)
        if context != self._quest_context:
            raise TrialFault("GROUP_ARM_INPUT_CONTEXT_CHANGED")
        self._check_measured_cone(p, initial=True)
        if self.direction_calibration is not None:
            self.direction_calibration.matrix_for(self.quest_provider(), p)

    async def arm(self):
        async with self._operation_lock:
            if self.state != "PREPARED" or self._prepared_context is None:
                raise TrialFault("Prepare the real robot connection first")
            p = self._robot()
            if self._robot_context(p) != self._prepared_context:
                raise TrialFault("ROBOT_CONTEXT_CHANGED")
            self._check_measured_cone(p, initial=True)
            _, _, context = self._input(max_age_s=QUEST_ADMISSION_MAX_AGE_MS / 1000)
            if self.direction_calibration is not None:
                self.direction_calibration.matrix_for(self.quest_provider(), p)
            self._generation += 1
            self._first_grip_pending = True
            generation = self._generation
            self.state, self.reason = "ARMING", "WAITING_FOR_IDLE_FEEDBACK"
            try:
                if p["robot_mode"] == 4:
                    await self._call("EnableRobot")
                await self._call("Stop")
                ack_at = time.monotonic()
                await self._confirm_stop(ack_at, require_enabled=True)
                await self._read_queue_id(reset=True)
                self._queue_last_submitted = None
                p = self._robot()
                _, _, new_context = self._input(max_age_s=QUEST_ADMISSION_MAX_AGE_MS / 1000)
                self._check_measured_cone(p, initial=True)
                if generation != self._generation or new_context != context:
                    raise TrialFault("INPUT_CONTEXT_CHANGED_WHILE_ARMING")
                self.center = {
                    "pose_mm_deg": list(p["pose_mm_deg"]),
                    "quaternion_xyzw": list(p["quaternion_xyzw"]),
                }
                self._quest_context = context
                self.target = None
                self.pending = None
                self._anchor_y = self._anchor_z = None
                self._anchor_hand = self._anchor_robot = None
                self._reset_incremental_reference()
                self.tracking_evidence = None
                self.armed = True
                self._release_after = time.monotonic()
                self._pause_latched = False
                self.state, self.reason = "WAIT_RELEASE", "RELEASE_GRIP_BEFORE_FIRST_PRESS"
                self._event("ARMED", center=self.center)
            except Exception as exc:
                self._request_stop(str(exc), lockout=True)
                if self._stop_task:
                    await self._stop_task
                raise TrialFault(str(exc)) from exc
        await self.start()
        return self.snapshot()

    def _request_stop(self, reason, *, lockout, recover_connection=False, stop_context=None):
        previous_context = (self.stop_evidence or {}).get("expected_robot_context")
        expected_context = stop_context or self._prepared_context or (
            tuple(previous_context) if previous_context is not None else None
        )
        self._generation += 1
        if reason == "GRIP_RELEASED" and not lockout:
            self._pause_latched = True
            self._release_after = time.monotonic()
        self._anchor_y = self._anchor_z = None
        self._anchor_hand = self._anchor_robot = None
        self._reset_incremental_reference()
        self._queue_last_seen = self._queue_last_submitted = None
        self.pending = None
        self.target = None
        self.tracking_warning = None
        if lockout:
            self.armed = False
            self._prepared_context = None
        self.state, self.reason = "STOPPING", reason
        if self._stop_task and not self._stop_task.done():
            return
        self._stop_task = asyncio.create_task(
            self._stop_and_confirm(reason, lockout, expected_context, recover_connection),
            name="left-z-stop",
        )

    async def _stop_and_confirm(self, reason, lockout, expected_context, recover_connection=False):
        started = time.monotonic()
        self.stop_evidence = {
            "requested_at": started,
            "reason": reason,
            "acknowledged": False,
            "feedback_confirmed": False,
            "expected_robot_context": expected_context,
        }
        try:
            stop_method = (
                "RecoverStop"
                if recover_connection and not getattr(self.driver, "connected", True)
                else "Stop"
            )
            self.stop_evidence["transport"] = stop_method
            await self._call(stop_method)
            ack_at = time.monotonic()
            self.stop_evidence.update(
                acknowledged=True,
                ack_at=ack_at,
                request_to_ack_ms=round((ack_at - started) * 1000, 3),
            )
            samples = await self._confirm_stop(ack_at, expected_context=expected_context)
            if not lockout and self.armed:
                # Stop invalidates our old queue baseline. Re-establish it on
                # the same connection before a normal clutch resume can send.
                queue_id = await self._read_queue_id(reset=True)
                if self._robot_context(self._robot()) != expected_context:
                    raise TrialFault("ROBOT_CONTEXT_CHANGED_DURING_PAUSE")
                self._queue_last_submitted = None
                self.stop_evidence["resume_queue_baseline"] = queue_id
            confirmed_at = time.monotonic()
            self.stop_evidence.update(
                feedback_confirmed=True,
                stationary_under_collision_protection=any(p["robot_mode"] == 11 for p in samples),
                confirmed_at=confirmed_at,
                samples=samples,
                request_to_feedback_ms=round((confirmed_at - started) * 1000, 3),
            )
            self.state = "DISARMED" if lockout or not self.armed else "WAIT_RELEASE"
            self.reason = reason
        except Exception as exc:
            self.armed = False
            self._prepared_context = None
            self.state, self.reason = "FAULT", f"STOP_NOT_CONFIRMED: {exc}"
            self.stop_evidence["error"] = str(exc)
        self._event("STOP_RESULT", evidence=dict(self.stop_evidence))

    async def _confirm_stop(self, after, require_enabled=False, expected_context=None):
        expected_context = expected_context or self._prepared_context
        deadline = time.monotonic() + 2
        samples = []
        while time.monotonic() < deadline:
            p = self._robot() if require_enabled else stop_feedback(self.feedback.snapshot())
            if expected_context is not None and self._robot_context(p) != expected_context:
                raise TrialFault("ROBOT_CONTEXT_CHANGED_DURING_STOP")
            if p["host_monotonic_s"] > after and p["robot_mode"] in (
                (5,) if require_enabled else (4, 5, 11)
            ):
                if not samples or p["host_monotonic_s"] > samples[-1]["host_monotonic_s"]:
                    sample = {
                        "host_monotonic_s": p["host_monotonic_s"],
                        "q_deg": list(p["joint_position_deg"]),
                        "robot_mode": p["robot_mode"],
                    }
                    if (
                        samples
                        and max(abs(a - b) for a, b in zip(sample["q_deg"], samples[0]["q_deg"]))
                        > 0.02
                    ):
                        samples = []
                    samples.append(sample)
                    if (
                        len(samples) >= 3
                        and samples[-1]["host_monotonic_s"] - samples[0]["host_monotonic_s"] >= 0.15
                    ):
                        return samples[-6:]
            else:
                samples = []
            await asyncio.sleep(0.02)
        raise TrialFault("No fresh stable idle feedback after Stop")

    async def fault_stop(self, reason):
        # A group fault is not an operator recovery: preserve the old robot
        # context and never reconnect a failed command channel automatically.
        self.armed = False
        self._request_stop(reason, lockout=True)
        await self._stop_task
        return self.snapshot()

    async def disarm(self):
        # A new explicit STOP is allowed to confirm against the currently live
        # feedback generation of the SAME robot/frame. It sends a new Stop ACK;
        # it never re-labels an older failed stop or grants motion permission.
        previous = self._prepared_context or (self.stop_evidence or {}).get("expected_robot_context")
        current = None
        try:
            observed = self._robot_context(stop_feedback(self.feedback.snapshot()))
            if previous is None or tuple(previous[:-1]) == observed[:-1]:
                current = observed
        except (TrialFault, ValueError):
            pass  # Still attempt Stop; unusable feedback must fail confirmation.
        self._event("EXPLICIT_STOP_CONTEXT", previous_context=previous, current_context=current)
        self.armed = False
        self._request_stop("OPERATOR_DISARMED", lockout=True, recover_connection=True, stop_context=current)
        await self._stop_task
        return self.snapshot()

    def _motion_guard(self, generation):
        if not self.armed or self.state != "FOLLOWING" or generation != self._generation or not self.motion_permit():
            raise TrialFault("TARGET_GENERATION_REVOKED")
        p = self._robot()
        self._check_measured_cone(p)
        if p["robot_mode"] == 4:
            raise TrialFault("ROBOT_DISABLED_WHILE_ARMED")
        grip, _, context = self._input()
        if (
            grip is not GripPhase.HELD
            or context != self._quest_context
            or self._robot_context(p) != self._prepared_context
        ):
            raise TrialFault("INPUT_OR_ROBOT_CONTEXT_CHANGED")
        return p

    def _reset_incremental_reference(self):
        self._direction_mapping = None
        self._previous_hand = None
        self._previous_hand_seq = -1
        self._last_window_at = -math.inf
        self._reference = None
        self._last_reference_at = None
        self.requested_target = None
        self.orientation_evidence = None

    def _consume_increment_window(self, hand, frame, decision, *, robot=None, target=None):
        sequence = frame.get("seq")
        if type(sequence) is not int or sequence <= self._previous_hand_seq:
            return False
        window = {
            "seq_from": self._previous_hand_seq,
            "seq_to": sequence,
            "decision": decision,
            "hand_previous": self._previous_hand,
            "hand_current": hand,
            "input_received_at": frame.get("host_monotonic_s"),
            "direction_mapping": self._direction_mapping,
            "position_target_policy": "GRIP_ANCHORED_WORLD" if self._anchored_world_position else "MEASURED_INCREMENTAL",
            "orientation_target_policy": "GRIP_ANCHORED_WORLD" if self._anchored_world_orientation else "MEASURED_INCREMENTAL",
            "hand_anchor": self._anchor_hand if self._anchored_world_position else None,
            "robot_anchor_pose_mm_deg": self._anchor_robot if self._anchored_world_position else None,
            "tool_cone_target": self.cone_evidence.get("target") if self._cone_enabled else None,
        }
        if robot is not None:
            window.update(
                robot_measured_pose_mm_deg=list(robot["pose_mm_deg"]),
                robot_feedback_received_at=robot["host_monotonic_s"],
            )
        if target is not None:
            window["target_pose_mm_deg"] = list(target)
        self._previous_hand, self._previous_hand_seq = hand, sequence
        self._last_window_at = time.monotonic()
        self.last_increment_window = window
        self._event("INCREMENT_WINDOW", **window)
        return True

    def _drop_blocked_increment_window(self, hand, reason):
        now = time.monotonic()
        if (
            self._previous_hand is not None
            and now - self._last_window_at >= 0.1
            and now - self._last_command >= 0.1
        ):
            decision = reason.replace("DROPPED_", "COALESCED_", 1) if self._anchored_world_position else reason
            self._consume_increment_window(hand, self.quest_provider().get("frame") or {}, decision)

    async def _produce_target(self, pose, generation, *, input_received_at=None, cone_target_info=None):
        try:
            p = self._motion_guard(generation)
            q = list(p["joint_position_deg"])
            pose = [float(value) for value in pose]
            if len(pose) != 6 or not all(math.isfinite(value) for value in pose):
                raise TrialFault("INVALID_TARGET_POSE")
            if self._cone_enabled:
                pose, calculated = project_target(pose)
                self.cone_evidence["target"] = cone_target_info or calculated
                self.cone_evidence["last_rejection"] = None
            if self._kinematics is None or self.kinematics_evidence.get("status") != "VALIDATED":
                raise TrialFault("KINEMATIC_MODEL_NOT_VALIDATED")
            base_position_error, base_angle_error = pose_error(
                p["pose_mm_deg"], self._kinematics.fk(q)
            )
            if base_position_error > 0.05 or base_angle_error > 0.05:
                raise TrialFault("LOCAL_FK_MEASURED_MISMATCH")
            ranged = self._ranged_solver
            direct = self._direct_ranged_output
            if direct and ranged is None:
                raise TrialFault("RANGEDIK_BACKEND_NOT_INITIALIZED")
            if ranged:
                admissibility = {"command_delta_limit_deg": DIRECT_MAX_TRACKING_ERROR_DEG} if (
                    self._anchored_world_orientation and direct) else {}
                result = await asyncio.to_thread(
                    ranged.propose, pose, q, **admissibility,
                )
            else:
                result = await asyncio.to_thread(
                    self._kinematics.solve, pose, q, max_delta_deg=MAX_IK_JOINT_DELTA_DEG
                )
            self.last_ik_evidence = {
                "solver": "OFFICIAL_RANGEDIK_PANOC" if ranged else "LOCAL_BOUNDED_DLS",
                "measured_seed_deg": q,
                "requested_pose_mm_deg": pose,
                "local_result": result,
            }
            self._event("IK_SOLUTION", **self.last_ik_evidence)
            self._motion_guard(generation)
            if ranged and (result.get("continuation") or {}).get("output_admissible") is False:
                raise ConeCandidateRejected(result["reason"])
            if not result["success"] and not ranged:
                raise TrialFault(f"IK_NO_CONTINUOUS_SOLUTION_WITHIN_{MAX_IK_JOINT_DELTA_DEG:g}_DEG")
            local_q = [float(x) for x in result["q_deg"]]
            if len(local_q) != 6 or not all(math.isfinite(x) for x in local_q):
                raise TrialFault("INVALID_LOCAL_IK_RESULT")
            radii = ranged.profile.joint_search_radius_deg if ranged else [MAX_IK_JOINT_DELTA_DEG]*6
            if any(abs(a-b) > radius for a,b,radius in zip(local_q, q, radii)):
                raise TrialFault("RANGEDIK_CANDIDATE_SEARCH_BOUND_EXCEEDED" if ranged else
                                 f"IK_JOINT_STEP_EXCEEDS_{MAX_IK_JOINT_DELTA_DEG:g}_DEG")
            if ranged:
                # Range goals are intentionally not exact original-pose IK.
                # Re-solving that original pose in the controller would undo
                # the accepted relaxation. Verify the selected joint solution
                # against the controller's real FK instead.
                target_q = local_q
                selected_pose = result["selected_pose_mm_deg"]
            else:
                near = "{" + ",".join(f"{x:.9f}" for x in local_q) + "}"
                reply = await self._call(
                    "InverseKin", *pose, user=0, tool=0, useJointNear=1, JointNear=near
                )
                target_q = _six(reply["payload"])
                if max(abs(a-b) for a,b in zip(target_q, local_q)) > 0.5:
                    raise TrialFault("CONTROLLER_IK_BRANCH_DISAGREES_WITH_LOCAL")
                selected_pose = pose
            fk = await self._query_fk(target_q)
            position_error, orientation_error = pose_error(selected_pose, fk)
            self.last_ik_evidence.update(
                controller_joint_deg=target_q,
                controller_fk_pose_mm_deg=fk,
                fk_position_error_mm=position_error,
                fk_orientation_error_deg=orientation_error,
                selected_pose_mm_deg=selected_pose,
            )
            self._event("IK_FK_VERIFIED", **self.last_ik_evidence)
            if position_error > 0.05 or orientation_error > 0.05:
                raise TrialFault("IK_CONTROLLER_FK_TARGET_MISMATCH")
            if self._cone_enabled:
                self.cone_evidence["candidate_tilt_deg"] = tilt_deg(fk)
                if not in_cone(fk):
                    raise ConeCandidateRejected("IK_TOOL_TILT_EXCEEDS_45_DEG")
            if ranged:
                real_quality = ranged.task_error(pose, fk)
                if result["status"] == "WITHIN_RANGE" and not real_quality["within_task_range"]:
                    raise TrialFault("RANGEDIK_CONTROLLER_FK_OUTSIDE_ACCEPTED_RANGE")
                if result["status"] == "APPROACHING":
                    before_quality = ranged.task_error(pose, p["pose_mm_deg"])
                    before_cost = before_quality["normalized_error_cost"]
                    if before_cost - real_quality["normalized_error_cost"] <= max(1e-9, 1e-4*before_cost):
                        raise TrialFault("RANGEDIK_CONTROLLER_PROGRESS_MISMATCH")
                self.last_ik_evidence["controller_task_error"] = real_quality
                self.reason = result["reason"]
            p = self._motion_guard(generation)
            if any(abs(a-b) > radius for a,b,radius in zip(target_q, p["joint_position_deg"], radii)):
                raise TrialFault("IK_CANDIDATE_BOUND_CHANGED_BEFORE_OUTPUT")

            requested_q = list(target_q)
            self.requested_target = {"pose_mm_deg": pose, "joint_position_deg": requested_q,
                                     "selected_pose_mm_deg": selected_pose}
            reference, proposal = None, None
            if direct:
                # No post-solver filter, clipping, QP or second exact-pose IK.
                # This exact candidate already passed controller FK above.
                self.qp_evidence = None
                command_pose = fk
                max_tracking_error = DIRECT_MAX_TRACKING_ERROR_DEG
                self._event("RANGEDIK_DIRECT_REFERENCE", q_deg=target_q,
                            pose_mm_deg=command_pose, status=result["status"])
            else:
                reference = self._reference
                if reference is None:
                    raise TrialFault("QP_REFERENCE_NOT_INITIALIZED")
                dt = FIRST_DT_S if self._last_reference_at is None else (
                    time.monotonic() - self._last_reference_at
                )
                proposal = reference.propose(requested_q, p["joint_position_deg"], dt)
                self.qp_evidence = proposal
                self._event("QP_REFERENCE", **proposal)
                if not proposal["success"]:
                    raise TrialFault(proposal["reason"])
                target_q = proposal["q_deg"]
                command_pose = await self._query_fk(target_q)
                max_tracking_error = reference.max_tracking_error_deg
            if ranged:
                ranged.validate_joints(target_q)
            model_error = pose_error(command_pose, self._kinematics.fk(target_q))
            if model_error[0] > 0.05 or model_error[1] > 0.05:
                raise TrialFault("DIRECT_COMMAND_FK_MODEL_MISMATCH" if direct else
                                 "QP_COMMAND_FK_MODEL_MISMATCH")

            def before_send():
                actual = self._motion_guard(generation)
                if ranged:
                    ranged.validate_joints(target_q)
                if (
                    input_received_at is not None
                    and not 0
                    <= time.monotonic() - input_received_at
                    <= QUEST_MOTION_MAX_AGE_MS / 1000
                ):
                    raise TrialFault("PLANNED_TARGET_INPUT_STALE")
                if (
                    max(abs(a - b) for a, b in zip(target_q, actual["joint_position_deg"]))
                    > max_tracking_error + 1e-6
                ):
                    raise TrialFault("DIRECT_TRACKING_ERROR_BEFORE_SEND" if direct else
                                     "QP_TRACKING_ERROR_BEFORE_SEND")
                if self._cone_enabled:
                    if not in_cone(command_pose):
                        raise ConeCandidateRejected("COMMAND_TOOL_TILT_EXCEEDS_45_DEG")
                    path = joint_path_check(self._kinematics, actual["joint_position_deg"], target_q)
                    self.cone_evidence["path"] = path
                    if not path["accepted"]:
                        raise ConeCandidateRejected(path["reason"])
                    # FK checks take time: recheck freshness/generation immediately
                    # before the SDK call; a changed measured start needs a new proposal.
                    newest = self._motion_guard(generation)
                    if newest["joint_position_deg"] != actual["joint_position_deg"]:
                        raise ConeCandidateRejected("TOOL_CONE_PATH_START_CHANGED")
                    if (input_received_at is not None and
                            not 0 <= time.monotonic()-input_received_at <= QUEST_MOTION_MAX_AGE_MS/1000):
                        raise TrialFault("PLANNED_TARGET_INPUT_STALE")

            ack = await self._call(
                "ServoJ", *target_q, t=0.15, aheadtime=50, gain=200, guard=before_send
            )
            if generation == self._generation:
                command_id = _queue_id(ack["payload"])
                if self._queue_last_seen is None or command_id <= max(
                    self._queue_last_seen, self._queue_last_submitted or 0
                ):
                    raise TrialFault("SERVOJ_QUEUE_ID_NOT_ADVANCING")
                self._queue_last_submitted = command_id
                self._last_command = time.monotonic()
                if reference is not None:
                    reference.commit(proposal)
                if ranged:
                    ranged.commit(result)
                self._last_reference_at = self._last_command
                self.pending = {
                    "q_deg": target_q,
                    "pose_mm_deg": command_pose,
                    "z_mm": command_pose[2],
                    "sent_at": self._last_command,
                    "command_id": command_id,
                }
                self.target = {"pose_mm_deg": command_pose, "joint_position_deg": target_q}
                self._event(
                    "TARGET_ACK",
                    pose_mm_deg=command_pose,
                    requested_pose_mm_deg=pose,
                    requested_joint_deg=requested_q,
                    reference_velocity_deg_s=proposal["velocity_deg_s"] if proposal else None,
                    motion_reference="RANGEDIK_DIRECT" if direct else "FILTERED_VELOCITY_ACCELERATION_BOX_QP",
                    z_mm=command_pose[2],
                    q_deg=target_q,
                    command_id=command_id,
                )
        except ConeCandidateRejected as exc:
            if generation == self._generation and self.state != "STOPPING":
                self.reason = str(exc)
                self.cone_evidence["last_rejection"] = {"reason": str(exc), "command_sent": False}
                now = time.monotonic()
                if now-self._last_cone_rejection_at >= 1:
                    self._event("TOOL_CONE_REJECTED", evidence=dict(self.cone_evidence))
                    self._last_cone_rejection_at = now
        except Exception as exc:
            if generation == self._generation and self.state != "STOPPING":
                self._request_stop(str(exc), lockout=True)

    async def tick(self):
        if not self.armed or self.state == "STOPPING" or not self.motion_permit():
            return
        try:
            p = self._robot()
            self._check_measured_cone(p)
            if p["robot_mode"] == 4:
                raise TrialFault("ROBOT_DISABLED_WHILE_ARMED")
            admission = self.state in ("WAIT_RELEASE", "READY")
            if self._robot_context(p) != self._prepared_context:
                raise TrialFault("INPUT_OR_ROBOT_CONTEXT_CHANGED")
            try:
                grip, hand_pose, context = self._input(
                    max_age_s=(QUEST_ADMISSION_MAX_AGE_MS if admission else QUEST_MOTION_MAX_AGE_MS)
                    / 1000
                )
            except TrialFault:
                if admission and self._pause_latched:
                    # The arm is already feedback-confirmed stopped. A resting
                    # hand outside tracking must not cancel the entire session.
                    # Recovery still needs a new valid release then press;
                    # changed session/epoch is checked below before any motion.
                    if self.reason != "PAUSED_WAITING_FOR_VALID_INPUT":
                        self._release_after = time.monotonic()
                    self.state, self.reason = "WAIT_RELEASE", "PAUSED_WAITING_FOR_VALID_INPUT"
                    return
                raise
            if self._robot_context(p) != self._prepared_context or context != self._quest_context:
                raise TrialFault("INPUT_OR_ROBOT_CONTEXT_CHANGED")
            if self.state == "WAIT_RELEASE":
                frame = self.quest_provider().get("frame") or {}
                if (
                    grip is GripPhase.RELEASED
                    and frame.get("host_monotonic_s", -math.inf) > self._release_after
                ):
                    self.state, self.reason = "READY", "PRESS_GRIP_TO_FOLLOW"
                return
            if self.state == "READY":
                if grip is GripPhase.HELD:
                    frame = self.quest_provider().get("frame") or {}
                    if (
                        not 0
                        <= time.monotonic() - frame.get("host_monotonic_s", -math.inf)
                        <= QUEST_MOTION_MAX_AGE_MS / 1000
                    ):
                        return  # Stay ready; never engage or send using a 200-400 ms old frame.
                    if p["robot_mode"] != 5:
                        raise TrialFault("ROBOT_NOT_IDLE_AT_ENGAGE")
                    self._check_measured_cone(p, initial=self._first_grip_pending)
                    self._first_grip_pending = False
                    self._anchor_hand = hand_pose
                    self._anchor_robot = list(p["pose_mm_deg"])
                    self._direction_mapping = (
                        self.direction_calibration.matrix_for(self.quest_provider(), p)
                        if self.direction_calibration is not None
                        else grip_direction_mapping(hand_pose, p["pose_mm_deg"])
                    )
                    self._previous_hand, self._previous_hand_seq = hand_pose, frame["seq"]
                    self._last_window_at = time.monotonic()
                    self._reference = None
                    if not self._direct_ranged_output:
                        self._reference = ReferenceQP()
                        self._reference.reset(p["joint_position_deg"])
                    self._last_reference_at = None
                    if self._ranged_solver:
                        self._ranged_solver.reset(p["joint_position_deg"])
                    self._generation += 1
                    self._pause_latched = False
                    self.state, self.reason = "FOLLOWING", (
                        "LEFT_GRIP_ANCHORED_WORLD_POSE_1_TO_1" if self._anchored_world_orientation
                        else "LEFT_GRIP_ANCHORED_WORLD_POSITION_1_TO_1" if self._anchored_world_position
                        else "LEFT_INCREMENTAL_MEASURED_BASE_1_TO_1"
                    )
                    self._event(
                        "GRIP_ENGAGED",
                        hand_pose=hand_pose,
                        robot_pose_mm_deg=self._anchor_robot,
                        mapping="FIXED_XR_WORLD_TO_ROBOT_BASE" if self.direction_calibration else "GRIP_DIRECTION_ONLY_INCREMENTAL_MEASURED_BASE",
                        direction_mapping=self._direction_mapping,
                        calibration_revision=self.direction_calibration.revision if self.direction_calibration else None,
                        position_target_policy="GRIP_ANCHORED_WORLD" if self._anchored_world_position else "MEASURED_INCREMENTAL",
                        translation_scale=1.0,
                    )
                return
            if grip is not GripPhase.HELD:
                self._request_stop("GRIP_RELEASED", lockout=False)
                return
            if self.pending:
                pending = self.pending
                submitted_id = pending.get("command_id")
                if type(submitted_id) is not int or submitted_id < 0:
                    raise TrialFault("PENDING_QUEUE_ID_UNKNOWN")
                if self._queue_task is None:
                    if time.monotonic() - self._last_queue_query < 0.05:
                        self._drop_blocked_increment_window(hand_pose, "DROPPED_QUEUE_BUSY")
                        return
                    self._queue_task_generation = self._generation
                    self._queue_task = asyncio.create_task(
                        self._queue_read_value(), name="left-queue-readback"
                    )
                    self._drop_blocked_increment_window(hand_pose, "DROPPED_QUEUE_QUERY")
                    return
                if not self._queue_task.done():
                    self._drop_blocked_increment_window(hand_pose, "DROPPED_QUEUE_QUERY")
                    return
                task, self._queue_task = self._queue_task, None
                if self._queue_task_generation != self._generation:
                    with suppress(Exception):
                        task.result()
                    return
                executing_id = task.result()
                self._accept_queue_value(executing_id)
                if self.pending is not pending or not self.armed or self.state != "FOLLOWING":
                    return
                p = self._motion_guard(self._generation)
                joint_errors = [
                    abs(a - b) for a, b in zip(p["joint_position_deg"], self.pending["q_deg"])
                ]
                error = max(joint_errors)
                fresh_after_command = p["host_monotonic_s"] > self.pending["sent_at"]
                if fresh_after_command:
                    self._report_tracking_lag(joint_errors, p["host_monotonic_s"])
                self.tracking_evidence = {
                    "joint_error_deg": error,
                    "joint_errors_deg": joint_errors,
                    "lag_policy": "ADVISORY_ONLY",
                    "fresh_after_command": fresh_after_command,
                    "within_tracking_window": fresh_after_command and error <= TRACKING_WINDOW_DEG,
                    "precise_arrival_0_03_deg": fresh_after_command and error <= 0.03,
                    "host_monotonic_s": p["host_monotonic_s"],
                    "robot_mode": p["robot_mode"],
                    "queue_executing_id": executing_id,
                    "submitted_id": submitted_id,
                    "previous_target_started": executing_id >= submitted_id,
                }
                if (
                    p["robot_mode"] in (5, 7)
                    and fresh_after_command
                    and executing_id >= submitted_id
                ):
                    self._event("TARGET_PROGRESS_ACCEPTED", **self.tracking_evidence)
                    self.pending = None
                else:
                    if time.monotonic() - self.pending["sent_at"] > PENDING_TARGET_TIMEOUT_S:
                        raise TrialFault("QUEUE_PROGRESS_OR_FEEDBACK_TIMEOUT")
                    self._drop_blocked_increment_window(hand_pose, "DROPPED_QUEUE_OR_FEEDBACK_WAIT")
                    return
            if self._motion_task and not self._motion_task.done():
                self._drop_blocked_increment_window(hand_pose, "DROPPED_IK_BUSY")
                return
            if p["robot_mode"] not in (5, 7):
                return
            if time.monotonic() - self._last_command < 0.1:
                return
            if time.monotonic() - self._last_window_at < 0.1:
                return
            if self._previous_hand is None or self._direction_mapping is None:
                raise TrialFault("INCREMENT_REFERENCE_MISSING")
            # Queue queries can yield; use the newest real input, not a queued old sample.
            grip, hand_pose, context = self._input()
            if grip is not GripPhase.HELD:
                self._request_stop("GRIP_RELEASED", lockout=False)
                return
            if context != self._quest_context:
                raise TrialFault("INPUT_OR_ROBOT_CONTEXT_CHANGED")
            frame = self.quest_provider().get("frame") or {}
            if frame.get("seq", -1) <= self._previous_hand_seq:
                return
            p = self._motion_guard(self._generation)
            if self._anchored_world_position:
                if self.direction_calibration is None or self._anchor_hand is None or self._anchor_robot is None:
                    raise TrialFault("WORLD_POSITION_ANCHOR_NOT_INITIALIZED")
                if self._anchored_world_orientation:
                    desired = anchored_pose_target(
                        self._anchor_hand, self._anchor_robot, hand_pose, self._direction_mapping,
                    )
                else:
                    desired = anchored_position_target(
                        self._anchor_hand, self._anchor_robot,
                        self._previous_hand, p["pose_mm_deg"], hand_pose, self._direction_mapping,
                    )
            else:
                desired = incremental_pose_target(
                    self._previous_hand, p["pose_mm_deg"], hand_pose, self._direction_mapping
                )
            cone_target_info = None
            if self._cone_enabled:
                desired, cone_target_info = project_target(desired)
                self.cone_evidence["target"] = cone_target_info
            position_error = math.sqrt(
                sum((a - b) ** 2 for a, b in zip(desired[:3], p["position_mm"]))
            )
            orientation_error = _angle_deg(euler_xyz_deg_to_xyzw(desired[3:]), p["quaternion_xyzw"])
            self.orientation_evidence = {
                "policy": "GRIP_ANCHORED_WORLD" if self._anchored_world_orientation else "MEASURED_INCREMENTAL",
                "gain": 1.0, "input_seq": frame["seq"],
                "target_euler_deg": desired[3:], "measured_euler_deg": list(p["pose_mm_deg"][3:]),
                "error_angle_deg": orientation_error,
                "cone_projected": bool(cone_target_info and cone_target_info["projected"]),
                "unreached_goal_retained": self._anchored_world_orientation,
                "error_metric": self._ranged_profile.orientation_error_metric if self._ranged_profile else None,
                "tolerance_deg": self._ranged_profile.orientation_tolerance_deg if self._ranged_profile else None,
                "within_orientation_tolerance": (
                    orientation_error <= self._ranged_profile.orientation_tolerance_deg*(1+1e-6)
                    if self._ranged_profile and self._ranged_profile.orientation_error_metric == "GEODESIC_ANGLE" else None),
            }
            no_change = position_error < 0.02 and orientation_error < 0.02
            if no_change and p["robot_mode"] == 5 and self._last_reference_at is None:
                self._consume_increment_window(
                    hand_pose, frame, "NO_CHANGE_IDLE_HOLD", robot=p, target=desired
                )
                return
            if no_change and not self._anchored_world_position:
                desired = list(p["pose_mm_deg"])
            self._consume_increment_window(hand_pose, frame, "PLANNED", robot=p, target=desired)
            self._motion_task = asyncio.create_task(
                self._produce_target(
                    desired, self._generation, input_received_at=frame.get("host_monotonic_s"),
                    cone_target_info=cone_target_info,
                ),
                name="left-se3-latest-target",
            )
        except Exception as exc:
            self._request_stop(str(exc), lockout=True)

    async def _loop(self):
        while not self._closed:
            await self.tick()
            await asyncio.sleep(0.02)

    async def close(self):
        if self.armed or self.state in ("ARMING", "STOPPING"):
            await self.disarm()
        self._closed = True
        if self._loop_task:
            self._loop_task.cancel()
            with suppress(asyncio.CancelledError):
                await self._loop_task
        if self._motion_task:
            with suppress(Exception):
                await self._motion_task
        if self._queue_task:
            with suppress(Exception):
                await self._queue_task
        if self._ranged_solver:
            self._ranged_solver.close()
            self._ranged_solver = None

    def snapshot(self):
        feedback = self.feedback.snapshot()
        direct = self._direct_ranged_output
        anchored = self._anchored_world_position
        calibration = (self.direction_calibration.snapshot(self.quest_provider(), feedback)
                       if self.direction_calibration is not None else None)
        phase = ("PAUSING" if self.state == "STOPPING" and self.reason == "GRIP_RELEASED"
                 else "PAUSED" if self._pause_latched and self.state in ("WAIT_RELEASE", "READY")
                 else self.state)
        visualization = {"schema": 1, "frame": "robot_base", "units": "mm",
                         "model_kind": "POE_KINEMATIC_CHAIN", "valid": False,
                         "chain_points_mm": None}
        if callable(getattr(self._kinematics, "chain_points", None)) and feedback.get("valid"):
            visualization.update(valid=True, chain_points_mm=self._kinematics.chain_points(feedback["joint_position_deg"]))
        return {
            "side": self.side,
            "state": self.state,
            "reason": self.reason,
            "armed": self.armed,
            "clutch": {"phase": phase, "pause_reason": self.reason if self._pause_latched else None,
                       "resume_requires_new_press": True},
            "direction_calibration": calibration,
            "tool_orientation_cone": self._cone_snapshot(feedback),
            "orientation_tracking": self.orientation_evidence,
            "visualization": visualization,
            "preflight": dict(self.preflight),
            "limits": {
                "pending_target_timeout_s": PENDING_TARGET_TIMEOUT_S,
                "stop_reply_timeout_s": STOP_REPLY_TIMEOUT_S,
                "translation_scale": 1.0,
                "motion_dof": 6,
                "tool_tilt_limit_deg": MAX_TILT_DEG if self._cone_enabled else None,
                "initial_tool_tilt_limit_deg": INITIAL_TILT_DEG if self._cone_enabled else None,
                "coordinate_mapping": "XR_WORLD_FIXED_CALIBRATED_BASE" if self.direction_calibration else "GRIP_DIRECTION_FIXED_INCREMENTAL_MEASURED_TCP_NOT_WORLD_CALIBRATED",
                "direction_calibration_mode": calibration.get("mode") if calibration else None,
                "operator_frame_revision": (calibration.get("operator_frame") or {}).get("revision") if calibration else None,
                "control_mode": "GRIP_ANCHORED_WORLD_POSITION" if anchored else "INCREMENTAL_MEASURED_BASE",
                "position_target_policy": "GRIP_ANCHORED_WORLD" if anchored else "MEASURED_INCREMENTAL",
                "ik_solver": "OFFICIAL_RANGEDIK_PANOC_CONTROLLER_FK_VERIFIED" if self._ranged_profile else "LOCAL_BOUNDED_DLS_CONTROLLER_FK_VERIFIED",
                "ik_target_policy": "RANGED_GOAL_WITH_EXPLICIT_PROGRESS_OR_HOLD" if self._ranged_profile else "EXACT_TCP_NO_SCALING_NO_JOINT_CLIPPING",
                "ik_search_radius_deg": list(self._ranged_profile.joint_search_radius_deg) if self._ranged_profile else [MAX_IK_JOINT_DELTA_DEG]*6,
                "effective_ik_search_radius_deg": list(self._ranged_profile.joint_search_radius_deg) if direct else None,
                "ik_search_intersects_absolute_joint_limits": bool(self._ranged_profile),
                "motion_reference": "RANGEDIK_DIRECT" if direct else "FILTERED_VELOCITY_ACCELERATION_BOX_QP",
                "reference_max_velocity_deg_s": None if direct else REFERENCE_MAX_VELOCITY_DEG_S,
                "reference_max_acceleration_deg_s2": None if direct else REFERENCE_MAX_ACCELERATION_DEG_S2,
                "reference_max_tracking_error_deg": DIRECT_MAX_TRACKING_ERROR_DEG if direct else REFERENCE_MAX_TRACKING_ERROR_DEG,
                "reference_goal_filter_tau_s": None if direct else 0.2,
                "reference_tracking_tau_s": None if direct else 0.2,
                "solver_smoothness": "SOFT_OBJECTIVES_NOT_HARD_RATE_LIMITS" if self._ranged_profile else None,
                "servo_target_duration_s": 0.15,
                "reference_policy": ("VERIFIED_RANGEDIK_CANDIDATE_RETAINED_POSITION_NO_PATH_REPLAY" if anchored else "VERIFIED_RANGEDIK_CANDIDATE_NO_POST_FILTER_NO_REPLAY") if direct else "UNITY_STATIC_GAIN_WITH_DYNAMIC_LAG_NO_BACKLOG_REPLAY",
                "target_reference": "GRIP_ENGAGEMENT_TCP_POSITION" if anchored else "CURRENT_MEASURED_TCP",
                "hand_delta_reference": "GRIP_ENGAGEMENT_POSITION" if anchored else "PREVIOUS_CONSUMED_CONTROL_WINDOW",
                "orientation_target_policy": "GRIP_ANCHORED_WORLD" if self._anchored_world_orientation else "MEASURED_INCREMENTAL",
                "orientation_target_reference": "GRIP_ENGAGEMENT_TCP_ORIENTATION" if self._anchored_world_orientation else "CURRENT_MEASURED_TCP_PLUS_WINDOW_ROTATION",
                "orientation_scale": 1.0,
                "orientation_error_metric": self._ranged_profile.orientation_error_metric if self._ranged_profile else None,
                "orientation_tolerance_deg": self._ranged_profile.orientation_tolerance_deg if self._ranged_profile else None,
                "blocked_window_policy": "LATEST_ANCHORED_POSITION_NO_PATH_REPLAY" if anchored else "DROP_AND_REBASE_NO_REPLAY",
                "tracking_window_deg": TRACKING_WINDOW_DEG,
                "tracking_lag_policy": "ADVISORY_ONLY",
                "tracking_lag_blocks_next_target": False,
                "next_target_requires_precise_arrival": False,
                "max_unstarted_targets": 1,
                "queue_policy": "PRIOR_SUBMISSION_MUST_HAVE_STARTED_OR_BEEN_PASSED",
                "position_deadband_mm": 0.02,
                "rotation_deadband_deg": 0.02,
                "z_envelope_mm": None,
                "workspace_validation": "NOT_PERFORMED",
                "speed_limit_mm_s": None,
                "step_limit_mm": None,
                "target_rate_limit_mm_s": None,
                "acceleration_limit_mm_s2": None,
                "step_policy": "DIRECT_TARGET_WITH_JOINT_JUMP_REJECTION",
                "controller_dynamic_limits": "UNCHANGED_NOT_INSPECTED",
                "joint_step_limit_deg": min(DIRECT_MAX_TRACKING_ERROR_DEG, max(self._ranged_profile.joint_search_radius_deg)) if direct else (max(self._ranged_profile.joint_search_radius_deg) if self._ranged_profile else MAX_IK_JOINT_DELTA_DEG),
                "joint_arrival_tolerance_deg": 0.03,
                "same_pose_ik_consistency_deg": None if self._ranged_profile else 0.2,
                "quest_admission_max_age_ms": QUEST_ADMISSION_MAX_AGE_MS,
                "quest_motion_max_age_ms": QUEST_MOTION_MAX_AGE_MS,
            },
            "center": self.center,
            "target": self.target,
            "pending": self.pending,
            "tracking_evidence": self.tracking_evidence,
            "tracking_warning": self.tracking_warning,
            "kinematics": dict(self.kinematics_evidence),
            "ik_evidence": self.last_ik_evidence,
            "ranged_ik": self._ranged_solver.snapshot() if self._ranged_solver else (self._ranged_profile.snapshot() if self._ranged_profile else None),
            "qp_evidence": self.qp_evidence,
            "requested_target": self.requested_target,
            "reference_state": self._reference.snapshot() if self._reference else None,
            "queue_evidence": {
                "last_seen_id": self._queue_last_seen,
                "last_submitted_id": self._queue_last_submitted,
            },
            "incremental_state": {
                "previous_hand_seq": self._previous_hand_seq,
                "last_window": self.last_increment_window,
                "hand_anchor": self._anchor_hand if anchored else None,
                "robot_anchor_pose_mm_deg": self._anchor_robot if anchored else None,
            },
            "stop_evidence": self.stop_evidence,
            "command_timings": {key: dict(value) for key, value in self._command_timings.items()},
            "feedback": {
                key: feedback.get(key)
                for key in (
                    "robot_id",
                    "frame_id",
                    "connection_generation",
                    "connected",
                    "valid",
                    "reason",
                    "robot_mode",
                    "enable_status",
                    "error_status",
                    "receive_hz",
                    "feedback_age_ms",
                    "host_monotonic_s",
                )
            },
            "events": list(self._events),
        }
