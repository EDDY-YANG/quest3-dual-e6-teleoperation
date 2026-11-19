"""Frozen, gravity-aligned operator frame sampled from the real WebXR pointing ray."""
import math
import numpy as np
from relative_pose import _hand_pose

UP_XR = np.array([0.0, 1.0, 0.0])
MIN_FORWARD_HORIZONTAL = 0.25


def xr_context(quest):
    frame = quest.get("frame") or {}
    values = (quest.get("session"), frame.get("reference_space"), frame.get("reference_epoch"))
    return values if all(v is not None for v in values) else None


def hand_sample(quest, side):
    frame = quest.get("frame") or {}
    hand = (frame.get("hands") or {}).get(side) or {}
    age = quest.get("age_ms")
    buttons = (hand.get("gamepad") or {}).get("buttons") or []
    if not (quest.get("connected") is True and type(age) in (int, float) and 0 <= age <= 200
            and frame.get("visibility") == "visible"
            and frame.get("input_policy") == "continuous-pose-grip-v2"
            and hand.get("tracking_valid") is True and hand.get("is_controller") is True
            and (hand.get("pose") or {}).get("emulated_position") is False
            and hand.get("grip_held") is False and len(buttons) >= 2
            and type(buttons[1].get("value")) in (int, float) and 0 <= buttons[1]["value"] <= .2):
        raise ValueError(f"需要新鲜真实{'左' if side == 'left' else '右'}手输入并松开 Grip")
    if xr_context(quest) is None:
        raise ValueError("缺少XR参考会话")
    position, _ = _hand_pose(hand["pose"])
    return {"position_m": list(position), "seq": frame["seq"],
            "host_monotonic_s": frame["host_monotonic_s"]}, hand


def horizontal_unit(value, minimum):
    value = np.array(value, dtype=float)
    horizontal = value - UP_XR * float(value @ UP_XR)
    length = float(np.linalg.norm(horizontal))
    if not math.isfinite(length) or length < minimum:
        raise ValueError("水平方向不足；请避免竖直指向/竖直采样")
    return horizontal / length, length


class OperatorFrame:
    def __init__(self):
        self.active = None
        self.revision = 0

    @staticmethod
    def forward_sample(quest, side):
        sample, hand = hand_sample(quest, side)
        ray = hand.get("target_ray")
        if not isinstance(ray, dict) or ray.get("emulated_position") is not False:
            raise ValueError("缺少真实手柄指向；请刷新Quest页面")
        q = ray.get("quaternion_xyzw")
        if not isinstance(q, list) or len(q) != 4 or any(type(v) not in (int, float) or not math.isfinite(v) for v in q):
            raise ValueError("指向四元数无效")
        norm = sum(v*v for v in q)
        if abs(norm-1) > .001:
            raise ValueError("指向四元数不是单位旋转")
        x, y, z, w = np.array(q) / math.sqrt(norm)
        # WebXR targetRaySpace -Z is the platform-defined pointing direction.
        ray_forward = np.array([-2*(x*z+w*y), -2*(y*z-w*x), -(1-2*(x*x+y*y))])
        forward, projection = horizontal_unit(ray_forward, MIN_FORWARD_HORIZONTAL)
        left = np.cross(UP_XR, forward)
        rotation = np.vstack((forward, left, UP_XR))
        return {**sample, "source_hand": side, "source": "TARGET_RAY_NEGATIVE_Z",
                "forward_xr": forward.tolist(), "horizontal_projection": projection,
                "R_operator_from_xr": rotation.tolist(), "context": xr_context(quest)}

    def capture(self, quest, side):
        if side not in ("left", "right"):
            raise ValueError("Unknown operator source hand")
        self.active = self.forward_sample(quest, side)
        self.revision += 1
        return self.snapshot(quest)

    def matrix_for(self, quest):
        if self.active is None or self.active["context"] != xr_context(quest):
            raise ValueError("OPERATOR_FORWARD_REQUIRED")
        return np.array(self.active["R_operator_from_xr"])

    def snapshot(self, quest):
        current = self.active is not None and self.active["context"] == xr_context(quest)
        ready = {}
        for side in ("left", "right"):
            try:
                self.forward_sample(quest, side)
                ready[side] = True
            except (ValueError, TypeError, KeyError):
                ready[side] = False
        return {"schema": 1, "current": current, "revision": self.revision,
                "state": "RECORDED" if current else "INVALID" if self.active else "UNRECORDED",
                "message": "操作者前方已固定，不随手腕旋转" if current else "将选定手柄指向正前方，再记录",
                "axes": {"X": "FROZEN_HORIZONTAL_FORWARD", "Y": "LEFT", "Z": "UP"},
                "up_xr": UP_XR.tolist(), "capture_ready": ready,
                "R_operator_from_xr": self.active["R_operator_from_xr"] if current else None,
                "forward_xr": self.active["forward_xr"] if current else None,
                "source_hand": self.active["source_hand"] if current else None}

