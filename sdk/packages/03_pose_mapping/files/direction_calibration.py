"""Per-arm, session-bound yaw calibration with gravity-fixed vertical axes.

Samples come from validated live hand input, never a client-supplied matrix.
Only horizontal heading is fitted. WebXR +Y maps exactly to base +Z;
level robot bases are required. This is not physical/AR position registration.
"""

import math
import time

import numpy as np

from operator_frame import OperatorFrame, UP_XR, hand_sample, horizontal_unit

MIN_AXIS_M = 0.08
MAX_AXIS_M = 1.5


class DirectionCalibration:
    def __init__(self, side="left", operator=None):
        if side not in ("left", "right"):
            raise ValueError("Unknown calibration side")
        self.side = side
        self.operator = operator if operator is not None else OperatorFrame()
        self.revision = 0
        self.active = None
        self.draft = None
        self.stage = "UNCALIBRATED"
        self.last_error = None
        self.last_attempt = None

    @staticmethod
    def context(quest, robot):
        frame = quest.get("frame") or {}
        fields = (quest.get("session"), frame.get("reference_space"),
                  frame.get("reference_epoch"), robot.get("robot_id"),
                  robot.get("frame_id"), robot.get("user"), robot.get("tool"))
        return fields if all(v is not None for v in fields) else None

    def sample(self, quest, robot):
        sample, _ = hand_sample(quest, self.side)
        robot_age = time.monotonic() - robot.get("host_monotonic_s", -math.inf)
        if not (robot.get("valid") is True and robot.get("connected") is True
                and 0 <= robot_age <= .2 and robot.get("robot_mode") in (4, 5)
                and robot.get("error_status") == 0 and robot.get("user") == 0 and robot.get("tool") == 0):
            raise ValueError("校准需要该臂静止且反馈有效")
        if self.context(quest, robot) is None:
            raise ValueError("校准缺少机器人坐标上下文")
        return sample

    def apply_action(self, action, quest, robot):
        frame = quest.get("frame") or {}
        self.last_attempt = {"action": action, "seq": frame.get("seq"),
                             "host_monotonic_s": frame.get("host_monotonic_s")}
        try:
            self._apply_action(action, quest, robot)
        except (ValueError, TypeError, KeyError) as exc:
            self.last_error = str(exc)
            raise
        self.last_error = None
        return self.snapshot(quest, robot)

    def _apply_action(self, action, quest, robot):
        if action in ("capture_y", "apply"):
            raise ValueError("新版只采X，完成X后自动应用；无需+Y或单独应用，请刷新头显页面")
        if action not in ("begin", "capture_x", "cancel", "clear"):
            raise ValueError("Unknown direction calibration action")
        if action == "clear":
            self.active = self.draft = None
            self.stage = "UNCALIBRATED"
            self.revision += 1
            return
        if action == "cancel":
            self.draft = None
            self.stage = "CALIBRATED" if self.active else "UNCALIBRATED"
            return
        sample = self.sample(quest, robot)
        self.last_attempt["sample"] = sample
        context = self.context(quest, robot)
        if action == "begin":
            self.draft = {"context": context, "samples": [sample]}
            self.stage = "WAIT_X"
            return
        if self.stage != "WAIT_X" or not self.draft or self.draft["context"] != context:
            raise ValueError("先点击开始记录X；坐标会话变化后需要重录")
        origin = self.draft["samples"][0]
        if sample["seq"] <= origin["seq"] or sample["host_monotonic_s"] <= origin["host_monotonic_s"]:
            raise ValueError("需要新的手柄帧，请稍后再完成X标定")
        delta = np.array(sample["position_m"]) - np.array(origin["position_m"])
        horizontal = delta - UP_XR * float(delta @ UP_XR)
        length = float(np.linalg.norm(horizontal))
        self.last_attempt.update(horizontal_distance_mm=length*1000,
                                 vertical_drift_mm=float(delta[1]*1000))
        if length < MIN_AXIS_M:
            raise ValueError(f"水平位移仅{length*1000:.0f}mm，至少需要{MIN_AXIS_M*1000:.0f}mm；无需回起点")
        if float(np.linalg.norm(delta)) > MAX_AXIS_M:
            raise ValueError("位移超过1500mm，请重新开始记录X")
        x, _ = horizontal_unit(delta, MIN_AXIS_M)
        y = np.cross(UP_XR, x)
        y /= np.linalg.norm(y)
        x = np.cross(y, UP_XR)
        self.active = {"context": context, "samples": [origin, sample],
                       "R_base_from_xr": np.vstack((x, y, UP_XR)).tolist(),
                       "axis_lengths_mm": [length*1000],
                       "vertical_drift_mm": [float(delta[1]*1000)]}
        self.draft = None
        self.stage = "CALIBRATED"
        self.revision += 1

    def base_matrix_for(self, quest, robot):
        if (self.stage != "CALIBRATED" or self.active is None
                or self.active["context"] != self.context(quest, robot)):
            raise ValueError("WORLD_DIRECTION_CALIBRATION_REQUIRED")
        return self.active["R_base_from_xr"]

    def matrix_for(self, quest, robot):
        base = np.array(self.base_matrix_for(quest, robot))
        operator = self.operator.matrix_for(quest)
        # Explicit operator-frame mapping, algebraically equal to the frozen
        # world-to-base map. Never align operator +X forcibly with base +X.
        base_from_operator = base @ operator.T
        return (base_from_operator @ operator).tolist()

    def snapshot(self, quest, robot):
        current = self.context(quest, robot)
        record = self.draft or self.active
        state = self.stage
        if record and record["context"] != current:
            state = "INVALID"
        messages = {"UNCALIBRATED": "先连接/重检，再校准房间方向",
                    "WAIT_X": "沿该臂基座+X平移，再点击完成X标定；无需回起点",
                    "CALIBRATED": "X标定已自动应用；Y/Z已补齐，Z固定向上",
                    "INVALID": "坐标会话或机器人改变，请重新校准方向"}
        live_horizontal_mm = None
        try:
            live = self.sample(quest, robot)
            capture_ready = True
            if state == "WAIT_X" and self.draft:
                delta = np.array(live["position_m"]) - np.array(self.draft["samples"][0]["position_m"])
                live_horizontal_mm = float(np.linalg.norm(delta-UP_XR*float(delta@UP_XR))*1000)
        except (ValueError, TypeError, KeyError):
            capture_ready = False
        frame = quest.get("frame") or {}
        operator = self.operator.snapshot(quest)
        base_current = state == "CALIBRATED" and self.active is not None
        base_from_operator = None
        if base_current and operator["current"]:
            base_from_operator = (np.array(self.active["R_base_from_xr"]) @
                                  np.array(operator["R_operator_from_xr"]).T).tolist()
        return {"schema": 3, "side": self.side, "mode": "X_ONLY_YAW_FIXED_UP",
                "auto_apply": True, "generated_axes": ["Y", "Z"],
                "last_error": self.last_error, "last_attempt": self.last_attempt,
                "live_horizontal_distance_mm": live_horizontal_mm,
                "state": state, "revision": self.revision,
                "up_xr": UP_XR.tolist(), "up_base": [0, 0, 1],
                "operator_frame": operator, "control_ready": base_current and operator["current"],
                "R_base_from_operator": base_from_operator,
                "operator_forward_yaw_deg": math.degrees(math.atan2(base_from_operator[1][0], base_from_operator[0][0])) if base_from_operator else None,
                "vertical_drift_mm": record.get("vertical_drift_mm") if record else None,
                "current": state == "CALIBRATED" and self.active is not None,
                "R_base_from_xr": record.get("R_base_from_xr") if record else None,
                "reference_space": frame.get("reference_space"),
                "reference_epoch": frame.get("reference_epoch"),
                "message": messages[state], "capture_ready": capture_ready,
                "axis_lengths_mm": record.get("axis_lengths_mm") if record else None,
                "samples": [dict(sample) for sample in record["samples"]] if record else [],
                "min_axis_length_mm": MIN_AXIS_M*1000,
                "min_distance_mm": MIN_AXIS_M*1000,
                "scope": "YAW_ONLY_LEVEL_BASE_DIRECTION_NOT_AR_REGISTRATION"}
