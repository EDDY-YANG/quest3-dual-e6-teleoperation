"""Explicit opt-in RangedIK task ranges; no robot/network side effects."""

import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path

E6_NOMINAL_ABSOLUTE_LIMIT_RAD = (6.27, 2.356, 2.6878, 2.7925, 3.0194, 6.27)


def vector(value, length, name, *, positive=False):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{name} requires {length} numbers")
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in value):
        raise ValueError(f"{name} requires finite numbers, not booleans")
    if positive and any(v <= 0 for v in value):
        raise ValueError(f"{name} must be positive")
    return tuple(float(v) for v in value)


@dataclass(frozen=True)
class RangedIKProfile:
    profile_id: str
    library_path: Path
    position_tolerance_mm: tuple
    rotation_tolerance_deg: tuple
    joint_search_radius_deg: tuple
    joint_limits_deg: tuple
    objective_weights: dict
    source_path: Path
    sha256: str
    output_mode: str
    position_target_policy: str = "MEASURED_INCREMENTAL"
    orientation_target_policy: str = "MEASURED_INCREMENTAL"
    orientation_error_metric: str = "ROTATION_VECTOR_COMPONENTS"
    orientation_tolerance_deg: float | None = None
    seed_strategy: str = "MEASURED_ONLY"

    @classmethod
    def load(cls, path):
        path = Path(path).resolve()
        raw = path.read_bytes()
        cfg = json.loads(raw)
        if type(cfg.get("schema")) is not int or cfg.get("schema") != 1 or cfg.get("backend") != "official_rangedik_panoc":
            raise ValueError("Expected schema 1, official_rangedik_panoc profile")
        profile_id = cfg.get("profile_id")
        if not isinstance(profile_id, str) or not profile_id:
            raise ValueError("profile_id is required")
        output_mode = cfg.get("output_mode")
        if output_mode not in ("direct", "filtered_qp"):
            raise ValueError("output_mode must explicitly select direct or filtered_qp")
        position_policy = cfg.get("position_target_policy", "MEASURED_INCREMENTAL")
        if position_policy not in ("MEASURED_INCREMENTAL", "GRIP_ANCHORED_WORLD"):
            raise ValueError("Unsupported position_target_policy")
        if position_policy == "GRIP_ANCHORED_WORLD" and output_mode != "direct":
            raise ValueError("Grip-anchored world position requires direct RangedIK output")
        orientation_policy = cfg.get("orientation_target_policy", "MEASURED_INCREMENTAL")
        if orientation_policy not in ("MEASURED_INCREMENTAL", "GRIP_ANCHORED_WORLD"):
            raise ValueError("Unsupported orientation_target_policy")
        if orientation_policy == "GRIP_ANCHORED_WORLD" and position_policy != "GRIP_ANCHORED_WORLD":
            raise ValueError("Persistent orientation requires the calibrated grip-anchored world contract")
        seed_strategy = cfg.get("seed_strategy", "MEASURED_ONLY")
        if seed_strategy not in ("MEASURED_ONLY", "VALIDATED_MULTI_START"):
            raise ValueError("Unsupported seed_strategy")
        position = vector(cfg["position_tolerance_mm"], 3, "position_tolerance_mm", positive=True)
        metric = cfg.get("orientation_error_metric", "ROTATION_VECTOR_COMPONENTS")
        if metric not in ("ROTATION_VECTOR_COMPONENTS", "GEODESIC_ANGLE"):
            raise ValueError("Unsupported orientation_error_metric")
        if seed_strategy == "VALIDATED_MULTI_START" and (
                metric != "GEODESIC_ANGLE" or orientation_policy != "GRIP_ANCHORED_WORLD" or output_mode != "direct"):
            raise ValueError("Multi-start requires persistent geodesic direct control")
        total_tolerance = cfg.get("orientation_tolerance_deg")
        if metric == "GEODESIC_ANGLE":
            if (type(total_tolerance) not in (int, float) or not math.isfinite(total_tolerance)
                    or not 0 < total_tolerance < 180 or "rotation_tolerance_deg" in cfg):
                raise ValueError("Geodesic mode requires one positive orientation_tolerance_deg below 180; no component ranges")
            total_tolerance = float(total_tolerance)
            # Retain the existing native six-tolerance ABI. The native metric
            # explicitly interprets these equal values as ONE spherical radius.
            rotation = (total_tolerance,)*3
        else:
            if total_tolerance is not None:
                raise ValueError("Component mode cannot declare a total orientation tolerance")
            rotation = vector(cfg["rotation_tolerance_deg"], 3, "rotation_tolerance_deg", positive=True)
        search = vector(cfg["joint_search_radius_deg"], 6, "joint_search_radius_deg", positive=True)
        if max(rotation) >= 180 or max(search) > 180:
            raise ValueError("Rotation tolerances must be below 180 degrees; search at most 180")
        limits = cfg.get("joint_limits_rad")
        if not isinstance(limits, list) or len(limits) != 6:
            raise ValueError("Six explicit absolute joint limits are required")
        limits = tuple(vector(pair, 2, "joint limit") for pair in limits)
        if any(lo >= hi for lo, hi in limits):
            raise ValueError("Joint lower limit must be below upper limit")
        if any(
            lo < -bound or hi > bound
            for (lo, hi), bound in zip(limits, E6_NOMINAL_ABSOLUTE_LIMIT_RAD)
        ):
            raise ValueError(
                "Absolute limits may tighten, not exceed, the official E6 nominal envelope"
            )
        weights = cfg.get("objective_weights", {})
        for key in ("position", "rotation"):
            weights[key] = vector(weights[key], 3, key, positive=True)
        if metric == "GEODESIC_ANGLE" and len(set(weights["rotation"])) != 1:
            raise ValueError("Geodesic orientation uses equal rotation weights for an isotropic angle")
        weights.setdefault("orientation_center", 0.0)
        for key in ("velocity", "acceleration", "jerk", "manipulability", "joint_limits", "orientation_center"):
            value = weights.get(key)
            if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
                raise ValueError(f"Invalid {key} objective weight")
        if cfg.get("collision_geometry") != "unvalidated_disabled":
            raise ValueError("This kinematics-only bundle has no validated collision geometry")
        library = Path(cfg["native_library"])
        if not library.is_absolute():
            library = path.parent / library
        return cls(
            profile_id,
            library.resolve(),
            position,
            rotation,
            search,
            tuple(tuple(math.degrees(v) for v in pair) for pair in limits),
            weights,
            path,
            hashlib.sha256(raw).hexdigest(),
            output_mode,
            position_policy,
            orientation_policy,
            metric,
            total_tolerance,
            seed_strategy,
        )

    def snapshot(self):
        return {
            "profile_id": self.profile_id,
            "backend": "official_rangedik_panoc",
            "output_mode": self.output_mode,
            "position_target_policy": self.position_target_policy,
            "orientation_target_policy": self.orientation_target_policy,
            "seed_strategy": self.seed_strategy,
            "objective_weights": {k: list(v) if isinstance(v, tuple) else v
                                  for k, v in self.objective_weights.items()},
            "profile_sha256": self.sha256,
            "position_tolerance_mm": list(self.position_tolerance_mm),
            "orientation_error_metric": self.orientation_error_metric,
            "orientation_tolerance_deg": self.orientation_tolerance_deg,
            "rotation_vector_tolerance_deg": list(self.rotation_tolerance_deg) if self.orientation_error_metric == "ROTATION_VECTOR_COMPONENTS" else None,
            "tolerance_frame": "GOAL_TCP",
            "joint_search_radius_deg": list(self.joint_search_radius_deg),
            "absolute_joint_limits_deg": [list(x) for x in self.joint_limits_deg],
            "absolute_limit_source": "OFFICIAL_E6_URDF_NOMINAL_VERIFY_ON_CONTROLLER",
            "collision_geometry": "UNVALIDATED_DISABLED",
            "model_validation": "CONTROLLER_FK_REQUIRED_PER_ARM",
        }
