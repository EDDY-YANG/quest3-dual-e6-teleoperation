"""Pure, no-send Grip-relative mapping. No network or robot-driver dependencies.

Call ``update(quest_snapshot, robot_poses, now=time.monotonic())``. The Quest
snapshot is the existing QuestState.snapshot() result. Each robot entry needs:
valid=True, position_mm[3], quaternion_xyzw[4], host_monotonic_s, robot_mode
(4 or 5), error_status=0, robot_id, frame_id, user, tool. Only actual measured
feedback belongs in these entries; unavailable feedback must stay unavailable.

Position: p_target = p_robot_anchor + scale * R_map * (p_vr - p_vr_anchor).
Orientation: R_target = R_map * R_vr * R_vr_anchor.T * R_map.T * R_robot_anchor.
All outputs are NO_SEND. Even verified-frame targets have no IK, collision,
workspace, dynamic-limit or stopping validation and are never executable here.
"""

import math
from dataclasses import dataclass

SIDES = ("left", "right")


def _vector(values, length):
    if not isinstance(values, (list, tuple)) or len(values) != length:
        raise ValueError("Invalid vector size")
    if any(type(x) not in (int, float) or not math.isfinite(x) for x in values):
        raise ValueError("Non-finite or nonnumeric vector")
    return tuple(float(x) for x in values)


def _transpose(matrix):
    return tuple(zip(*matrix))


def _multiply(a, b):
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3)
    )


def _rotate(matrix, vector):
    return tuple(sum(row[k] * vector[k] for k in range(3)) for row in matrix)


def _rotation(values):
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        raise ValueError("Rotation must be 3 by 3")
    r = tuple(_vector(row, 3) for row in values)
    identity = _multiply(r, _transpose(r))
    if any(abs(identity[i][j] - int(i == j)) > 1e-6 for i in range(3) for j in range(3)):
        raise ValueError("Rotation matrix must be orthonormal")
    det = (
        r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
        - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
        + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0])
    )
    if abs(det - 1.0) > 1e-6:
        raise ValueError("Rotation must be proper (determinant +1), not a reflection")
    return r


def _quaternion(values):
    q = _vector(values, 4)
    norm2 = sum(x * x for x in q)
    if abs(norm2 - 1.0) > 1e-3:
        raise ValueError("Quaternion is not unit length")
    # Correct floating-point roundoff only; reject clearly invalid measurements.
    return tuple(x / math.sqrt(norm2) for x in q)


def _quat_matrix(q):
    x, y, z, w = q
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def _matrix_quat(r, previous):
    trace = r[0][0] + r[1][1] + r[2][2]
    if trace > 0:
        s = 2 * math.sqrt(max(0, 1 + trace))
        q = ((r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s, s / 4)
    else:
        i = max(range(3), key=lambda k: r[k][k])
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2 * math.sqrt(max(0, 1 + r[i][i] - r[j][j] - r[k][k]))
        q = [0.0] * 4
        q[i], q[j], q[k] = s / 4, (r[i][j] + r[j][i]) / s, (r[i][k] + r[k][i]) / s
        q[3] = (r[k][j] - r[j][k]) / s
    q = _quaternion(q)
    return tuple(-x for x in q) if sum(a * b for a, b in zip(q, previous)) < 0 else q


@dataclass(frozen=True)
class FrameCalibration:
    """Caller-supplied axis mapping. Matrix validation is not physical calibration.

    A verified mapping must be explicitly bound to a Quest session/reference
    and robot coordinate frame. An unverified preset can only produce a draft
    when MappingConfig.allow_unverified_frame_preview is explicitly enabled.
    """

    rotation_vr_to_robot: tuple
    calibration_id: str
    verified: bool = False
    quest_session: str | None = None
    reference_space: str | None = None
    reference_epoch: int | None = None
    robot_frame_id: str | None = None

    def __post_init__(self):
        object.__setattr__(self, "rotation_vr_to_robot", _rotation(self.rotation_vr_to_robot))
        if not isinstance(self.calibration_id, str) or not self.calibration_id.strip():
            raise ValueError("Calibration/preset ID is required")
        if type(self.verified) is not bool:
            raise ValueError("verified must be an explicit boolean")


@dataclass(frozen=True)
class MappingConfig:
    translation_scale: float = 0.3
    engage_threshold: float = 0.7
    release_threshold: float = 0.2
    quest_max_age_s: float = 0.5
    robot_max_age_s: float = 0.5
    allow_unverified_frame_preview: bool = False

    def __post_init__(self):
        for key in (
            "translation_scale",
            "engage_threshold",
            "release_threshold",
            "quest_max_age_s",
            "robot_max_age_s",
        ):
            value = getattr(self, key)
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError(f"Invalid {key}")
        if not 0 < self.translation_scale <= 1:
            raise ValueError("translation_scale must be in (0, 1]")
        if not 0 <= self.release_threshold < self.engage_threshold <= 1:
            raise ValueError("Grip thresholds must satisfy 0 <= release < engage <= 1")
        if self.quest_max_age_s <= 0 or self.robot_max_age_s <= 0:
            raise ValueError("Freshness limits must be positive")
        if type(self.allow_unverified_frame_preview) is not bool:
            raise ValueError("Preview opt-in must be a boolean")


@dataclass
class _ArmState:
    requires_release: bool = True
    anchor: dict | None = None
    context: tuple | None = None
    sequence: int = -1
    received_at: float = -math.inf
    generation: int = 0
    last_quaternion: tuple | None = None

    def cancel(self):
        if self.anchor is not None:
            self.generation += 1
        self.anchor = None
        self.last_quaternion = None
        self.requires_release = True


class _Fault(ValueError):
    def __init__(self, state, reason):
        self.state, self.reason = state, reason


def _fresh(received_at, now, maximum):
    return (
        type(received_at) in (float, int)
        and math.isfinite(received_at)
        and 0 <= now - received_at < maximum
    )


class DualClutchMapper:
    """Stateful per-arm clutch; output is a calculation, never a motion request."""

    def __init__(self, calibrations=None, config=None):
        self.config = config or MappingConfig()
        self.calibrations = {}
        self._arms = {side: _ArmState() for side in SIDES}
        for side, calibration in (calibrations or {}).items():
            self.set_calibration(side, calibration)

    def set_calibration(self, side, calibration):
        if side not in SIDES or (
            calibration is not None and not isinstance(calibration, FrameCalibration)
        ):
            raise ValueError("Expected left/right and FrameCalibration or None")
        self.calibrations[side] = calibration
        # A configuration change must not preserve an engaged clutch.
        previous_generation = self._arms[side].generation
        self._arms[side] = _ArmState(generation=previous_generation + 1)

    def _quest(self, snapshot, side, now):
        if not isinstance(snapshot, dict) or snapshot.get("connected") is not True:
            raise _Fault("INPUT_INVALID", "QUEST_DISCONNECTED")
        frame = snapshot.get("frame")
        session = snapshot.get("session")
        if not isinstance(frame, dict) or not isinstance(session, str) or not session:
            raise _Fault("INPUT_INVALID", "QUEST_FRAME_MISSING")
        if not _fresh(frame.get("host_monotonic_s"), now, self.config.quest_max_age_s):
            raise _Fault("INPUT_INVALID", "QUEST_STALE_OR_FUTURE_TIMESTAMP")
        if frame.get("visibility") != "visible":
            raise _Fault("INPUT_INVALID", "XR_NOT_VISIBLE")
        epoch, sequence, ref = (
            frame.get("reference_epoch"),
            frame.get("seq"),
            frame.get("reference_space"),
        )
        if (
            type(epoch) is not int
            or epoch < 0
            or type(sequence) is not int
            or sequence < 0
            or ref not in ("local-floor", "local")
        ):
            raise _Fault("INPUT_INVALID", "INVALID_REFERENCE_OR_SEQUENCE")
        hand = frame.get("hands", {}).get(side)
        if not isinstance(hand, dict) or hand.get("is_controller") is not True:
            raise _Fault("INPUT_INVALID", "CONTROLLER_MISSING")
        pose, gp = hand.get("pose"), hand.get("gamepad")
        if (
            not isinstance(gp, dict)
            or gp.get("connected") is not True
            or gp.get("mapping") != "xr-standard"
        ):
            raise _Fault("INPUT_INVALID", "GRIP_MAPPING_UNAVAILABLE")
        try:
            grip = gp["buttons"][1]["value"]
            if type(grip) not in (int, float) or not math.isfinite(grip) or not 0 <= grip <= 1:
                raise ValueError()
            if frame.get("input_policy") == "grip-gated-v1":
                if (
                    hand.get("tracking_valid") is not True
                    or type(hand.get("grip_held")) is not bool
                ):
                    raise _Fault("INPUT_INVALID", "UPSTREAM_TRACKING_INVALID")
                if not hand["grip_held"]:
                    if pose is not None:
                        raise _Fault("INPUT_INVALID", "IDLE_POSE_NOT_ALLOWED")
                    return (
                        None,
                        None,
                        float(grip),
                        (session, ref, epoch),
                        sequence,
                        frame["host_monotonic_s"],
                    )
                if grip <= self.config.release_threshold:
                    raise _Fault("INPUT_INVALID", "HELD_GRIP_INCONSISTENT_WITH_BUTTON")
            if not isinstance(pose, dict) or pose.get("emulated_position") is not False:
                raise _Fault("INPUT_INVALID", "POSITION_MISSING_OR_EMULATED")
            if snapshot.get("valid", {}).get(side) is not True:
                raise _Fault("INPUT_INVALID", "UPSTREAM_TRACKING_INVALID")
            p, q = _vector(pose["position_m"], 3), _quaternion(pose["quaternion_xyzw"])
            if frame.get("input_policy") == "continuous-pose-grip-v2":
                if (
                    hand.get("tracking_valid") is not True
                    or type(hand.get("grip_held")) is not bool
                ):
                    raise _Fault("INPUT_INVALID", "UPSTREAM_TRACKING_INVALID")
                if not hand["grip_held"]:
                    return (
                        None,
                        None,
                        float(grip),
                        (session, ref, epoch),
                        sequence,
                        frame["host_monotonic_s"],
                    )
                if grip <= self.config.release_threshold:
                    raise _Fault("INPUT_INVALID", "HELD_GRIP_INCONSISTENT_WITH_BUTTON")
        except (ValueError, KeyError, TypeError, IndexError):
            raise _Fault("INPUT_INVALID", "INVALID_GRIP_OR_POSE") from None
        return p, q, float(grip), (session, ref, epoch), sequence, frame["host_monotonic_s"]

    def _robot(self, robot, now):
        if not isinstance(robot, dict) or robot.get("valid") is not True:
            raise _Fault("ROBOT_INVALID", "ROBOT_FEEDBACK_MISSING")
        if not _fresh(robot.get("host_monotonic_s"), now, self.config.robot_max_age_s):
            raise _Fault("ROBOT_INVALID", "ROBOT_STALE_OR_FUTURE_TIMESTAMP")
        if type(robot.get("error_status")) is not int or robot["error_status"] != 0:
            raise _Fault("ROBOT_INVALID", "ROBOT_ERROR_OR_ERROR_STATUS_UNKNOWN")
        if type(robot.get("robot_mode")) is not int or robot["robot_mode"] not in (4, 5):
            raise _Fault("ROBOT_INVALID", "ROBOT_NOT_IDLE_OR_MODE_UNKNOWN")
        if any(not isinstance(robot.get(k), str) or not robot[k] for k in ("robot_id", "frame_id")):
            raise _Fault("ROBOT_INVALID", "ROBOT_IDENTITY_OR_FRAME_MISSING")
        if any(type(robot.get(k)) is not int or robot[k] < 0 for k in ("user", "tool")):
            raise _Fault("ROBOT_INVALID", "ROBOT_USER_TOOL_UNKNOWN")
        try:
            p, q = _vector(robot["position_mm"], 3), _quaternion(robot["quaternion_xyzw"])
        except (ValueError, KeyError, TypeError):
            raise _Fault("ROBOT_INVALID", "INVALID_ROBOT_POSE") from None
        metadata = {k: robot[k] for k in ("robot_id", "frame_id", "user", "tool")}
        return p, q, metadata

    def update(self, quest_snapshot, robot_poses, *, now):
        outputs = {}
        clock_valid = type(now) in (int, float) and math.isfinite(now)
        for side in SIDES:
            state, calibration = self._arms[side], self.calibrations.get(side)
            out = {
                "state": "NOT_CALIBRATED",
                "reason": None,
                "clutch_state": "DISENGAGED",
                "calibration_verified": False,
                "calibration_id": calibration.calibration_id if calibration else None,
                "target_valid": False,
                "target": None,
                "draft_target": None,
                "execution_allowed": False,
                "grip_value": None,
            }
            try:
                if not clock_valid:
                    raise _Fault("INPUT_INVALID", "INVALID_HOST_CLOCK")
                if calibration is None:
                    raise _Fault("NOT_CALIBRATED", "NO_FRAME_MAPPING_CONFIGURED")
                p_vr, q_vr, grip, vr_context, seq, received = self._quest(quest_snapshot, side, now)
                p_robot, q_robot, metadata = self._robot((robot_poses or {}).get(side), now)
                out["grip_value"] = grip
                expected_context = (
                    calibration.quest_session,
                    calibration.reference_space,
                    calibration.reference_epoch,
                )
                verified = (
                    calibration.verified
                    and expected_context == vr_context
                    and calibration.robot_frame_id == metadata["frame_id"]
                )
                out["calibration_verified"] = verified
                if not verified and not self.config.allow_unverified_frame_preview:
                    raise _Fault("NOT_CALIBRATED", "FRAME_MAPPING_UNVERIFIED_OR_REFERENCE_CHANGED")
                context = (*vr_context, *metadata.values())
                if context != state.context:
                    state.cancel()
                    state.context, state.sequence, state.received_at = context, -1, -math.inf
                if seq < state.sequence or received < state.received_at:
                    raise _Fault("INPUT_INVALID", "OUT_OF_ORDER_QUEST_FRAME")
                state.sequence, state.received_at = seq, received
                if state.requires_release:
                    if grip <= self.config.release_threshold:
                        state.requires_release = False
                        out.update(state="READY", reason="RELEASE_OBSERVED_WAIT_FOR_PRESS")
                    else:
                        out.update(
                            state="RELEASE_REQUIRED", reason="RELEASE_GRIP_BEFORE_REENGAGING"
                        )
                elif state.anchor is not None and grip <= self.config.release_threshold:
                    state.cancel()
                    state.requires_release = False
                    out.update(state="RELEASED", reason="GRIP_RELEASED_TARGET_CANCELLED")
                elif p_vr is None:
                    if state.anchor is not None:
                        state.cancel()
                    out.update(
                        state="RELEASE_REQUIRED" if state.requires_release else "READY",
                        reason="UPSTREAM_GRIP_GATE_INACTIVE",
                    )
                else:
                    if state.anchor is None and grip >= self.config.engage_threshold:
                        state.anchor = {
                            "vr_p": p_vr,
                            "vr_r": _quat_matrix(q_vr),
                            "robot_p": p_robot,
                            "robot_r": _quat_matrix(q_robot),
                        }
                        state.last_quaternion = q_robot
                        state.generation += 1
                    if state.anchor is None:
                        out.update(state="READY", reason="WAIT_FOR_GRIP_PRESS")
                    else:
                        a, mapping = state.anchor, calibration.rotation_vr_to_robot
                        delta = tuple(
                            (p_vr[i] - a["vr_p"][i]) * 1000 * self.config.translation_scale
                            for i in range(3)
                        )
                        offset = _rotate(mapping, delta)
                        target_p = _vector(tuple(a["robot_p"][i] + offset[i] for i in range(3)), 3)
                        delta_r = _multiply(_quat_matrix(q_vr), _transpose(a["vr_r"]))
                        target_r = _multiply(
                            _multiply(_multiply(mapping, delta_r), _transpose(mapping)),
                            a["robot_r"],
                        )
                        target_q = _matrix_quat(target_r, state.last_quaternion)
                        state.last_quaternion = target_q
                        target = {
                            "position_mm": list(target_p),
                            "quaternion_xyzw": list(target_q),
                            **metadata,
                        }
                        out.update(
                            state="FOLLOWING" if verified else "UNVERIFIED_FRAME_PREVIEW",
                            clutch_state="FOLLOWING",
                            reason="NO_SEND_ONLY"
                            if verified
                            else "AXIS_PRESET_NOT_PHYSICALLY_CALIBRATED",
                            target_valid=verified,
                            target=target if verified else None,
                            draft_target=None if verified else target,
                        )
            except _Fault as fault:
                state.cancel()
                out.update(state=fault.state, reason=fault.reason)
            except (ValueError, TypeError, AttributeError, KeyError, OverflowError):
                state.cancel()
                out.update(state="INPUT_INVALID", reason="MALFORMED_INPUT_OR_NONFINITE_TARGET")
            out.update(requires_release=state.requires_release, generation=state.generation)
            outputs[side] = out
        return {"mode": "NO_SEND", "execution_allowed": False, "arms": outputs}
