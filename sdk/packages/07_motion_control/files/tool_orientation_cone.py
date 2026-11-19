"""Tool +Z within a downward cone. Pure geometry; never sends robot commands.

Down is base -Z under the existing level-base convention. The interpolation
certificate below applies to the identified all-revolute joint-linear model,
not to unobserved controller interpolation, tracking error or stop dynamics.
"""
import math
from relative_pose import _euler_rotation, _multiply, _near_euler, _vector

MAX_TILT_DEG = 45.0
INITIAL_TILT_DEG = 5.0
ANGLE_EPS_DEG = 1e-6
MAX_PATH_EVALUATIONS = 128


def configuration():
    """Configured policy only; this is not a robot measurement or readiness."""
    return {"limit_deg": MAX_TILT_DEG, "initial_limit_deg": INITIAL_TILT_DEG,
            "axis": "TOOL_PLUS_Z", "reference": "BASE_MINUS_Z_LEVEL_BASE_REQUIRED",
            "scope": "RANGEDIK_TELEOP_ONLY; HOMING_HAS_SEPARATE_5_DEG_ENDPOINT_CHECK",
            "physical_continuous_bound_validated": False}


def tilt_deg(pose):
    pose = _vector(pose, 6, "TCP pose")
    r = _euler_rotation(pose[3:])
    return math.degrees(math.acos(max(-1.0, min(1.0, -r[2][2]))))


def in_cone(pose, limit_deg=MAX_TILT_DEG):
    return tilt_deg(pose) <= limit_deg + ANGLE_EPS_DEG


def project_target(pose):
    """Closest swing towards down; preserves XYZ and never clamps Euler axes."""
    pose = list(_vector(pose, 6, "TCP target"))
    r = _euler_rotation(pose[3:])
    before = tilt_deg(pose)
    evidence = {"raw_target_pose_mm_deg": pose, "raw_tilt_deg": before,
                "limit_deg": MAX_TILT_DEG, "projected": before > MAX_TILT_DEG}
    if before <= MAX_TILT_DEG:
        return list(pose), {**evidence, "target_tilt_deg": before}
    # Cross(tool_z, down) rotates the tool axis towards -Z.
    axis = [-r[1][2], r[0][2], 0.0]
    norm = math.sqrt(sum(v*v for v in axis))
    if norm < 1e-10:
        # Antiparallel up/down has no unique shortest swing. Tool X supplies
        # a deterministic orthogonal axis; no guessed physical measurement.
        axis = [r[i][0] for i in range(3)]
        norm = math.sqrt(sum(v*v for v in axis))
    x, y, z = [v/norm for v in axis]
    a = math.radians(before-MAX_TILT_DEG)
    c, s, t = math.cos(a), math.sin(a), 1-math.cos(a)
    correction = ((t*x*x+c, t*x*y-s*z, t*x*z+s*y),
                  (t*x*y+s*z, t*y*y+c, t*y*z-s*x),
                  (t*x*z-s*y, t*y*z+s*x, t*z*z+c))
    bounded = pose[:3] + _near_euler(_multiply(correction, r), pose[3:])
    after = tilt_deg(bounded)
    if after > MAX_TILT_DEG + ANGLE_EPS_DEG:
        raise ValueError("TOOL_CONE_PROJECTION_FAILED")
    return bounded, {**evidence, "target_tilt_deg": after}


def joint_path_check(model, q_from, q_to, max_evaluations=MAX_PATH_EVALUATIONS):
    """Conservatively certify the model's joint-linear interpolation or refuse.

    Along a six-revolute chain, total orientation travel is bounded by the
    sum of absolute joint angle changes. Cone angle is 1-Lipschitz in
    orientation distance. Each interval therefore has a certified angle
    upper bound (angle_at_a + angle_at_b + sum_abs_delta * interval)/2.
    If finite evaluation budget cannot certify it, return NOT_CERTIFIED;
    do not treat sparse samples as proof that the full path is inside.
    """
    start = _vector(q_from, 6, "path start joints")
    end = _vector(q_to, 6, "path target joints")
    delta = [b-a for a, b in zip(start, end)]
    length_bound = sum(abs(v) for v in delta)
    a0, a1 = tilt_deg(model.fk(start)), tilt_deg(model.fk(end))
    evidence = {"scope": "IDENTIFIED_MODEL_JOINT_LINEAR_PATH_ONLY",
                "evaluations": 2, "max_sample_tilt_deg": max(a0, a1),
                "orientation_travel_bound_deg": length_bound,
                "max_certified_tilt_bound_deg": max(a0, a1)}
    def reject(reason):
        return {**evidence, "accepted": False, "reason": reason}
    if max(a0, a1) > MAX_TILT_DEG + ANGLE_EPS_DEG:
        return reject("TOOL_CONE_PATH_OUTSIDE")
    if length_bound < 1e-12:
        return {**evidence, "accepted": True, "reason": "STATIONARY"}
    pending = [(0.0, a0, 1.0, a1)]
    while pending:
        lo, left, hi, right = pending.pop()
        upper = max(left, right, (left+right+length_bound*(hi-lo))/2)
        if upper <= MAX_TILT_DEG + ANGLE_EPS_DEG:
            evidence["max_certified_tilt_bound_deg"] = max(evidence["max_certified_tilt_bound_deg"], upper)
            continue
        if evidence["evaluations"] >= max_evaluations:
            return reject("TOOL_CONE_PATH_NOT_CERTIFIED")
        mid = (lo+hi)/2
        angle = tilt_deg(model.fk([q+mid*d for q, d in zip(start, delta)]))
        evidence["evaluations"] += 1
        evidence["max_sample_tilt_deg"] = max(evidence["max_sample_tilt_deg"], angle)
        if angle > MAX_TILT_DEG + ANGLE_EPS_DEG:
            return reject("TOOL_CONE_PATH_OUTSIDE")
        pending.extend([(lo, left, mid, angle), (mid, angle, hi, right)])
    return {**evidence, "accepted": True, "reason": "CERTIFIED_FOR_MODEL"}

