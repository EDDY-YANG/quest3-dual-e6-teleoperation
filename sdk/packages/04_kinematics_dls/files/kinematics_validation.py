"""Controller-FK model commissioning; calculation queries only, never motion."""

import hashlib
import json
import math

from controller_kinematics import identify
from e6_feedback import euler_xyz_deg_to_xyzw

FK_POSITION_TOLERANCE_MM = 0.05
FK_ORIENTATION_TOLERANCE_DEG = 0.05


def pose_error(target, actual):
    position = math.dist(target[:3], actual[:3])
    qa, qb = euler_xyz_deg_to_xyzw(target[3:]), euler_xyz_deg_to_xyzw(actual[3:])
    angle = math.degrees(2 * math.acos(min(1.0, abs(sum(a*b for a, b in zip(qa, qb))))))
    return position, angle


async def commission_model(query_fk, measured_q):
    samples = []
    vectors = [[0.0] * 6]
    for axis in range(6):
        for angle in (-30.0, 30.0):
            q = [0.0] * 6
            q[axis] = angle
            vectors.append(q)
    for q in vectors:
        samples.append({"q_deg": q, "pose_mm_deg": await query_fk(q)})
    model = identify(samples)
    # These mixed configurations were not used for identification. They are
    # mathematical FK queries and never physical movement requests.
    vectors = [list(measured_q), [15, -30, 45, 10, 35, -20], [-40, 20, -30, 15, -45, 60]]
    for axis in range(6):
        q = list(measured_q)
        q[axis] += 1.0
        vectors.append(q)
    errors = []
    for q in vectors:
        actual = await query_fk(q)
        position, orientation = pose_error(actual, model.fk(q))
        errors.append((position, orientation))
        if position > FK_POSITION_TOLERANCE_MM or orientation > FK_ORIENTATION_TOLERANCE_DEG:
            raise ValueError("CONTROLLER_FK_MODEL_VALIDATION_FAILED")
    evidence = {
        "status": "VALIDATED",
        "model": "CONTROLLER_IDENTIFIED_6R_POE",
        "identification_samples": len(samples),
        "independent_validation_samples": len(vectors),
        "max_position_error_mm": max(e[0] for e in errors),
        "max_orientation_error_deg": max(e[1] for e in errors),
        "samples_sha256": hashlib.sha256(json.dumps(samples, sort_keys=True).encode()).hexdigest(),
    }
    return model, evidence
