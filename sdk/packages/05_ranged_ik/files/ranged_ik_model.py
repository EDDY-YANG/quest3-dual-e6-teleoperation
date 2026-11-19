"""Export the identified E6 POE model to a position-limited RangedIK URDF.

Only the kinematics are represented. Axis points are mathematical points on
joint axes, not physical link/mesh geometry. Never enable collision objectives
on these virtual segments as if they were calibrated E6 collision geometry.
No robot, service or network is contacted by this module.
"""

import hashlib
import json
import math
import re
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np

from controller_kinematics import PoseModel


def _array(value, shape, name):
    try:
        raw = np.asarray(value)
        if raw.dtype.kind not in "iuf" or any(
            isinstance(v, (bool, np.bool_)) for v in np.asarray(value, dtype=object).flat
        ):
            raise ValueError(f"{name} must contain real numbers")
        data = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Invalid {name}") from exc
    if data.shape != shape or not np.isfinite(data).all():
        raise ValueError(f"{name} must be finite with shape {shape}")
    return data.copy()


def _names(arm_id):
    if not isinstance(arm_id, str) or not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]{0,31}", arm_id):
        raise ValueError("arm_id must be an ASCII identifier, 1-32 characters")
    return {
        "world_link": f"{arm_id}_world",
        "base_link": f"{arm_id}_base",
        "ee_link": f"{arm_id}_tcp",
        "joint_names": [f"{arm_id}_joint_{i}" for i in range(1, 7)],
    }


def _limits(value):
    limits = _array(value, (6, 2), "joint_limits_deg")
    if np.any(limits[:, 0] >= limits[:, 1]):
        raise ValueError("Each joint lower limit must be smaller than its upper limit")
    return limits


def _frame(axis, point):
    # Pick the least aligned reference direction to avoid a degenerate cross product.
    helper = np.eye(3)[int(np.argmin(np.abs(axis)))]
    x = helper - float(helper @ axis) * axis
    x /= np.linalg.norm(x)
    y = np.cross(axis, x)
    result = np.eye(4)
    result[:3, :3] = np.column_stack((x, y, axis))
    result[:3, 3] = point
    return result


def _inverse(transform):
    result = np.eye(4)
    result[:3, :3] = transform[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ transform[:3, 3]
    return result


def _rpy(rotation):
    pitch = math.atan2(-rotation[2, 0], math.hypot(rotation[0, 0], rotation[1, 0]))
    if abs(math.cos(pitch)) < 1e-10:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        yaw = 0.0
    else:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    return [roll, pitch, yaw]


def _numbers(values):
    return " ".join(format(float(v), ".17g") for v in values)


def _joint(parent, name, kind, parent_link, child_link, transform):
    element = ET.SubElement(parent, "joint", name=name, type=kind)
    ET.SubElement(element, "parent", link=parent_link)
    ET.SubElement(element, "child", link=child_link)
    ET.SubElement(
        element,
        "origin",
        xyz=_numbers(transform[:3, 3] / 1000.0),
        rpy=_numbers(_rpy(transform[:3, :3])),
    )
    return element


def export_urdf(model, joint_limits_deg, arm_id="left", *, velocity_limits_deg_s=None):
    """Return a UTF-8 URDF string with six local +Z revolute joints.

    ``joint_limits_deg`` is a caller-provided (6,2) table. This function never
    invents or expands physical position limits. Optional velocity values are
    supplied by the caller, not inferred from the FK model. Required URDF effort
    and unspecified velocity attributes are zero, unused schema values; they
    are NOT actuator capabilities. Dynamic/visual/collision geometry is absent.
    """
    if not isinstance(model, PoseModel):
        raise TypeError("An actual validated PoseModel instance is required")
    names, limits = _names(arm_id), _limits(joint_limits_deg)
    velocities = None
    if velocity_limits_deg_s is not None:
        velocities = _array(velocity_limits_deg_s, (6,), "velocity_limits_deg_s")
        if np.any(velocities <= 0):
            raise ValueError("Velocity limits must be positive")

    robot = ET.Element("robot", name=f"{arm_id}_identified_kinematics")
    robot.append(
        ET.Comment(
            " KINEMATICS ONLY: mathematical axis points, no validated collision or dynamics model. "
        )
    )
    ET.SubElement(robot, "link", name=names["world_link"])
    ET.SubElement(robot, "link", name=names["base_link"])
    _joint(
        robot,
        f"{arm_id}_world_to_base",
        "fixed",
        names["world_link"],
        names["base_link"],
        np.eye(4),
    )
    previous_frame = np.eye(4)
    previous_link = names["base_link"]
    for i, (axis, point) in enumerate(zip(model.axes, model.axis_points)):
        frame = _frame(axis, point)
        child = f"{arm_id}_link_{i + 1}"
        ET.SubElement(robot, "link", name=child)
        joint = _joint(
            robot,
            names["joint_names"][i],
            "revolute",
            previous_link,
            child,
            _inverse(previous_frame) @ frame,
        )
        # The pinned upstream parser only understands exact principal axes.
        ET.SubElement(joint, "axis", xyz="0 0 1")
        attributes = {
            "lower": format(math.radians(limits[i, 0]), ".17g"),
            "upper": format(math.radians(limits[i, 1]), ".17g"),
            "effort": "0",
            "velocity": "0",
        }
        if velocities is not None:
            attributes["velocity"] = format(math.radians(velocities[i]), ".17g")
        ET.SubElement(joint, "limit", **attributes)
        previous_frame, previous_link = frame, child
    ET.SubElement(robot, "link", name=names["ee_link"])
    _joint(
        robot,
        f"{arm_id}_tool_fixed",
        "fixed",
        previous_link,
        names["ee_link"],
        _inverse(previous_frame) @ model.home,
    )
    ET.indent(robot, space="  ")
    return (
        '<?xml version="1.0" encoding="utf-8"?>\n' + ET.tostring(robot, encoding="unicode") + "\n"
    )


def write_model_bundle(
    model,
    joint_limits_deg,
    starting_q_deg,
    output_dir,
    arm_id="left",
    *,
    velocity_limits_deg_s=None,
    overwrite=False,
):
    """Write a model/settings/manifest bundle; caller owns the output directory.

    Settings use an absolute URDF path for the native bridge. The unpatched
    upstream from_local_settings prepends configs/urdfs and cannot load these
    absolute-path settings. Use the project bridge or place an exported URDF in
    the upstream configs/urdfs directory and explicitly adapt its setting file.
    Existing bundle files are never overwritten unless overwrite=True.
    """
    names, limits = _names(arm_id), _limits(joint_limits_deg)
    starting = _array(starting_q_deg, (6,), "starting_q_deg")
    if np.any(starting < limits[:, 0]) or np.any(starting > limits[:, 1]):
        raise ValueError("Starting joints lie outside the supplied joint limits")
    if type(overwrite) is not bool:
        raise ValueError("overwrite must be bool")
    urdf = export_urdf(model, limits, arm_id, velocity_limits_deg_s=velocity_limits_deg_s)
    directory = Path(output_dir).resolve()
    paths = {
        "urdf": directory / f"{arm_id}.urdf",
        "settings": directory / f"{arm_id}.yaml",
        "manifest": directory / f"{arm_id}-model.json",
    }
    if not overwrite and any(path.exists() for path in paths.values()):
        raise FileExistsError("A model bundle file already exists; choose a new version directory")
    model_data = model.to_dict()
    source_json = json.dumps(model_data, sort_keys=True, allow_nan=False)
    settings = "\n".join(
        [
            "# Project native bridge settings: absolute URDF path; no physical movement.",
            "urdf: " + json.dumps(paths["urdf"].as_posix()),
            "base_links: " + json.dumps([names["base_link"]]),
            "ee_links: " + json.dumps([names["ee_link"]]),
            "starting_config: " + json.dumps(np.radians(starting).tolist()),
            "kinematics_only: true",
            "self_collision_geometry_validated: false",
            "enable_self_collision: false",
            "enable_environment_collision: false",
            "",
        ]
    )
    manifest = {
        "schema": 1,
        "arm_id": arm_id,
        **names,
        "scope": "IDENTIFIED_KINEMATICS_ONLY_NOT_PHYSICAL_OR_COLLISION_ACCEPTANCE",
        "input_model_units": {"position": "mm", "joint_angle": "deg"},
        "urdf_units": {"position": "m", "joint_angle": "rad"},
        "joint_limits_deg": limits.tolist(),
        "position_limits_source": "CALLER_SUPPLIED_NOT_INFERRED_FROM_KINEMATICS",
        "starting_q_deg": starting.tolist(),
        "model_sha256": hashlib.sha256(source_json.encode()).hexdigest(),
        "urdf_sha256": hashlib.sha256(urdf.encode()).hexdigest(),
        "model": model_data,
        "self_collision_geometry_validated": False,
        "inter_arm_collision_geometry_validated": False,
        "dynamics_model_provided": False,
        "effort_zero": "UNUSED_REQUIRED_URDF_FIELD_NOT_AN_ACTUATOR_SPECIFICATION",
        "paths": {key: str(path) for key, path in paths.items()},
    }
    texts = {
        "urdf": urdf,
        "settings": settings,
        "manifest": json.dumps(manifest, indent=2, allow_nan=False) + "\n",
    }
    directory.mkdir(parents=True, exist_ok=True)
    for key, path in paths.items():
        with path.open("w" if overwrite else "x", encoding="utf-8", newline="\n") as stream:
            stream.write(texts[key])
    return {
        **{key: str(path) for key, path in paths.items()},
        **names,
        "arm_id": arm_id,
        "kinematics_only": True,
    }
