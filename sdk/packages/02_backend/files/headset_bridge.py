"""Narrow paired-headset status/action contract. No sockets or robot commands.

The server routes only explicit prepare/arm/stop requests to the same PC trial.
No addresses, raw commands, alarm resets, limits or robot settings are accepted.
"""

import re
import time

# Admission/display tolerance is separate from the active motion watchdog.
QUEST_ADMISSION_MAX_AGE_MS = 400
QUEST_MOTION_MAX_AGE_MS = 200
HUD_STATUS_MAX_AGE_MS = 1500


def validate_action(data):
    if not isinstance(data, dict) or set(data) not in ({"type", "id", "action"}, {"type", "id", "action", "side"}):
        raise ValueError("Expected type, id, action and optional arm side")
    if data["type"] != "control_action" or data["action"] not in ("prepare", "arm", "stop", "return_left_start", "return_right_start"):
        raise ValueError("Unsupported headset action")
    if not isinstance(data["id"], str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", data["id"]):
        raise ValueError("Invalid headset action ID")
    if "side" in data and (data["side"] not in ("left", "right", "both", "auto") or data["action"] not in ("prepare", "arm")):
        raise ValueError("Arm side is only accepted for prepare/arm")
    return data["id"], data["action"]


def home_input_ready(quest, side=None):
    frame = quest.get("frame") or {}
    age = quest.get("age_ms")
    hands = [h for h in (frame.get("hands") or {}).values() if isinstance(h, dict)]
    selected = (frame.get("hands") or {}).get(side) if side in ("left", "right") else None
    released = (not selected or selected.get("grip_held") is False) if side else all(h.get("grip_held") is False for h in hands)
    return bool(quest.get("connected") and type(age) in (int, float) and 0 <= age < 400
                and frame.get("visibility") == "visible"
                and (frame.get("diagnostics") or {}).get("requested_mode") == "immersive-ar"
                and any(h.get("is_controller") and h.get("tracking_valid") for h in hands)
                and released)


def validate_calibration_action(data):
    if (not isinstance(data, dict) or set(data) not in ({"type", "id", "action"}, {"type", "id", "action", "side"})
            or data["type"] != "calibration_action"
            or data.get("side", "left") not in ("left", "right")
            or data["action"] not in ("begin", "capture_x", "capture_y", "apply", "cancel", "clear", "record_forward")
            or not isinstance(data["id"], str)
            or not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", data["id"])):
        raise ValueError("Invalid calibration action")
    return data["id"], data["action"]


def input_ready(quest, side="left"):
    frame = quest.get("frame") or {}
    hand = (frame.get("hands") or {}).get(side) or {}
    age = quest.get("age_ms")
    return bool(
        quest.get("connected") is True
        and type(age) in (int, float)
        and 0 <= age < QUEST_ADMISSION_MAX_AGE_MS
        and frame.get("visibility") == "visible"
        and frame.get("input_policy") == "continuous-pose-grip-v2"
        and (frame.get("diagnostics") or {}).get("requested_mode") == "immersive-ar"
        and hand.get("is_controller") is True
        and hand.get("tracking_valid") is True
        and hand.get("grip_held") is False
        and len((hand.get("gamepad") or {}).get("buttons") or []) >= 2
        and (hand["gamepad"]["buttons"][1].get("value", 1) <= 0.2)
    )


def headset_status(quest, trial=None, side="left"):
    side = getattr(trial, "side", side)
    frame = quest.get("frame") or {}
    hand = (frame.get("hands") or {}).get("left") or {}
    right_hand = (frame.get("hands") or {}).get("right") or {}
    network = (frame.get("diagnostics") or {}).get("network") or {}
    control = trial.snapshot() if trial is not None else {"state": "NOT_PREPARED", "armed": False}
    feedback = trial.feedback.snapshot() if trial is not None else {}
    state = control.get("state")
    ready = input_ready(quest, side)
    active_hand = (frame.get("hands") or {}).get(side) or {}
    active_buttons = (active_hand.get("gamepad") or {}).get("buttons") or []
    active = control.get("armed") is True or state in ("ARMING", "STOPPING")
    stopped = (control.get("stop_evidence") or {}).get("feedback_confirmed") is True
    prepare_allowed = not active and (state != "FAULT" or stopped)
    robot_valid = feedback.get("valid") is True and feedback.get("connected") is True
    age = feedback.get("feedback_age_ms")
    fresh_robot = robot_valid and type(age) in (int, float) and 0 <= age < 200
    preflight = control.get("preflight") or {}
    prepared_context = preflight.get("robot_context")
    current_context = [
        feedback.get(key)
        for key in ("robot_id", "frame_id", "user", "tool", "connection_generation")
    ]
    preflight_current = (
        preflight.get("status") == "PASSED"
        and isinstance(prepared_context, (list, tuple))
        and len(prepared_context) == 5
        and all(value is not None for value in current_context)
        and list(prepared_context) == current_context
    )
    calibration = control.get("direction_calibration")
    calibration_ready = calibration is None or (
        calibration.get("state") == "CALIBRATED" and calibration.get("current") is True
        and calibration.get("control_ready") is True
    )
    calibration_allowed = bool(
        calibration is not None and not active and preflight_current and fresh_robot
        and feedback.get("robot_mode") in (4, 5) and state == "PREPARED"
    )
    capture_allowed = calibration_allowed and calibration.get("capture_ready") is True
    cone = control.get("tool_orientation_cone") or {}
    initial_pose_ready = not cone.get("enabled") or cone.get("initial_ready") is True
    result = {
        "type": "teleop_status",
        "schema": 1,
        "teleop_side": side,
        "single_arm": True,
        "server_monotonic_s": time.monotonic(),
        "timing_policy": {
            "quest_admission_max_age_ms": QUEST_ADMISSION_MAX_AGE_MS,
            "quest_motion_max_age_ms": QUEST_MOTION_MAX_AGE_MS,
            "hud_status_max_age_ms": HUD_STATUS_MAX_AGE_MS,
        },
        "quest": {
            **{
                key: quest.get(key)
                for key in ("status", "connected", "age_ms", "receive_hz", "valid")
            },
            "visibility": frame.get("visibility"),
            "left_tracking": hand.get("tracking_valid"),
            "right_tracking": right_hand.get("tracking_valid"),
            "right_grip_held": right_hand.get("grip_held"),
            "active_tracking": active_hand.get("tracking_valid"),
            "active_grip_held": active_hand.get("grip_held"),
            "active_grip_value": active_buttons[1].get("value") if len(active_buttons) >= 2 else None,
            "left_grip_held": hand.get("grip_held"),
            "left_grip_value": (
                hand["gamepad"]["buttons"][1].get("value")
                if len((hand.get("gamepad") or {}).get("buttons") or []) >= 2
                else None
            ),
            "http_rtt_ms": network.get("http_rtt_ms"),
        },
        "robot": {
            "visualization": control.get("visualization"),
            **{key: feedback.get(key)
            for key in (
                "connected",
                "valid",
                "reason",
                "robot_mode",
                "enable_status",
                "error_status",
                "feedback_age_ms",
                "receive_hz",
                "position_mm",
                "pose_mm_deg",
                "joint_position_deg",
                "user",
                "tool",
                "user_value_mm_deg",
                "tool_value_mm_deg",
            )},
        },
        "control": {
            "side": side,
            "preflight_current": preflight_current,
            **{
                key: control.get(key)
                for key in (
                    "state",
                    "armed",
                    "reason",
                    "preflight",
                    "center",
                    "target",
                    "pending",
                    "stop_evidence",
                    "limits",
                    "tracking_evidence",
                    "tracking_warning",
                    "queue_evidence",
                    "incremental_state",
                    "kinematics",
                    "ik_evidence",
                    "ranged_ik",
                    "qp_evidence",
                    "requested_target",
                    "reference_state",
                    "clutch",
                    "direction_calibration",
                    "tool_orientation_cone",
                    "orientation_tracking",
                )
            },
        },
        "capabilities": {
            "prepare": prepare_allowed and quest.get("connected") is True,
            "arm": ready
            and fresh_robot
            and feedback.get("robot_mode") in (4, 5)
            and state == "PREPARED"
            and preflight_current and calibration_ready and initial_pose_ready,
            "stop": trial is not None,
            "calibration_begin": capture_allowed,
            "calibration_capture_x": capture_allowed and calibration.get("state") == "WAIT_X",
            "calibration_capture_y": capture_allowed and calibration.get("state") == "WAIT_Y",
            "calibration_apply": capture_allowed and calibration.get("state") == "REVIEW",
            "calibration_cancel": calibration_allowed,
            "calibration_clear": calibration_allowed,
        },
        "scope": "SELECTED_SINGLE_ARM_RELATIVE_SE3_PAIRED_HEADSET_EXPLICIT_ACTIONS",
    }
    result["robots"] = {s: result["robot"] if s == side else None for s in ("left", "right")}
    return result
