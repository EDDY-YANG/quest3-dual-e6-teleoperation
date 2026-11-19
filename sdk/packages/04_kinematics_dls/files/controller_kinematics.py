"""Pure 6R kinematics identified from controller FK queries; no I/O or motion.

Space-frame product of exponentials: T(q) = exp(S1*q1)...exp(S6*q6)*M.
Each revolute axis is identified by moving that coordinate in FK *queries*
only, while the other five coordinates are zero. Parameters use millimetres
and degrees; orientation is fixed-axis XYZ, Rz @ Ry @ Rx. Independent mixed
joint controller samples must validate the model before a caller uses it.

Reference: Modern Robotics, chapter 4.1.1 (space-frame POE).
The local solver never leaves its seed-centred joint box. A failed result's
q is diagnostic only; callers must check success and controller FK again.
"""

import math

import numpy as np

from relative_pose import _euler_rotation, _near_euler

POSITION_TOLERANCE_MM = 0.02
ORIENTATION_TOLERANCE_DEG = 0.02
ROTATION_WEIGHT_MM = 100.0
MAX_ITERATION_STEP_DEG = 2.0
MAX_IK_JOINT_DELTA_DEG = 30.0
DEG_TO_RAD = math.pi / 180.0


def _array(value, shape, name):
    try:
        original = np.asarray(value)
        if original.dtype.kind not in "iuf" or any(
            isinstance(item, (bool, np.bool_)) for item in np.asarray(value, dtype=object).flat
        ):
            raise ValueError(name + " must contain real numbers")
        result = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(name + " must contain real numbers") from exc
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f"{name} must be finite with shape {shape}")
    return result.copy()


def pose_matrix(pose_mm_deg):
    """Convert millimetres/fixed-axis XYZ degrees to a homogeneous matrix."""
    pose = _array(pose_mm_deg, (6,), "pose_mm_deg")
    result = np.eye(4)
    result[:3, :3] = _euler_rotation(pose[3:])
    result[:3, 3] = pose[:3]
    return result


def rotation_log(rotation):
    """Principal spatial rotation vector, in radians, including the pi branch."""
    r = _array(rotation, (3, 3), "rotation")
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-7, rtol=0) or abs(np.linalg.det(r) - 1.0) > 1e-7:
        raise ValueError("rotation must be proper and orthonormal")
    vee = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    sin_angle = float(np.linalg.norm(vee)) / 2
    cos_angle = float(np.clip((np.trace(r) - 1) / 2, -1, 1))
    angle = math.atan2(sin_angle, cos_angle)
    if angle < 1e-8:
        return vee / 2
    if math.pi - angle < 1e-6:
        _, vectors = np.linalg.eigh((r + r.T) / 2)
        axis = vectors[:, -1]
        if np.dot(axis, vee) < 0:
            axis = -axis
        return axis * angle
    return vee * (angle / (2 * sin_angle))


def _axis_transform(axis, point, angle_rad):
    x, y, z = axis
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    rotation = np.eye(3) + math.sin(angle_rad) * skew + (1 - math.cos(angle_rad)) * (skew @ skew)
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = (np.eye(3) - rotation) @ point
    return result


def pose_error(actual_pose, target_pose):
    """Geometric position norm (mm) and SO(3) orientation distance (deg)."""
    actual, target = pose_matrix(actual_pose), pose_matrix(target_pose)
    return (
        float(np.linalg.norm(target[:3, 3] - actual[:3, 3])),
        math.degrees(float(np.linalg.norm(rotation_log(target[:3, :3] @ actual[:3, :3].T)))),
    )


class PoseModel:
    """Identified spatial axes, home pose and bounded local TCP inverse solver."""

    def __init__(self, home, axes, axis_points, fit_statistics=None):
        self.home = _array(home, (4, 4), "home")
        rotation_log(self.home[:3, :3])
        if not np.allclose(self.home[3], [0, 0, 0, 1], rtol=0, atol=1e-10):
            raise ValueError("home is not a homogeneous transform")
        self.axes = _array(axes, (6, 3), "axes")
        self.axis_points = _array(axis_points, (6, 3), "axis_points")
        if not np.allclose(np.linalg.norm(self.axes, axis=1), 1, atol=1e-7, rtol=0):
            raise ValueError("Each revolute axis must be unit length")
        self.fit_statistics = dict(fit_statistics or {})
        for value in (self.home, self.axes, self.axis_points):
            value.flags.writeable = False

    def to_dict(self):
        return {
            "schema": 1,
            "model": "CONTROLLER_IDENTIFIED_6R_SPACE_POE",
            "home": self.home.tolist(),
            "axes": self.axes.tolist(),
            "axis_points": self.axis_points.tolist(),
            "fit_statistics": dict(self.fit_statistics),
        }

    @classmethod
    def from_dict(cls, data):
        if (
            not isinstance(data, dict)
            or data.get("schema") != 1
            or data.get("model") != "CONTROLLER_IDENTIFIED_6R_SPACE_POE"
        ):
            raise ValueError("Unsupported kinematics model schema")
        return cls(data["home"], data["axes"], data["axis_points"], data.get("fit_statistics"))

    def _forward(self, q_deg, with_jacobian=False):
        q = _array(q_deg, (6,), "q_deg")
        prefix = np.eye(4)
        moving_axes, moving_points = [], []
        for axis, point, angle in zip(self.axes, self.axis_points, q):
            if with_jacobian:
                moving_axes.append(prefix[:3, :3] @ axis)
                moving_points.append(prefix[:3, :3] @ point + prefix[:3, 3])
            prefix = prefix @ _axis_transform(axis, point, angle * DEG_TO_RAD)
        transform = prefix @ self.home
        if not with_jacobian:
            return transform
        position = transform[:3, 3]
        jacobian = np.empty((6, 6))
        for i, (axis, point) in enumerate(zip(moving_axes, moving_points)):
            jacobian[:3, i] = np.cross(axis, position - point) * DEG_TO_RAD
            jacobian[3:, i] = axis * DEG_TO_RAD
        return transform, jacobian

    def fk(self, q_deg):
        transform = self._forward(q_deg)
        return transform[:3, 3].tolist() + _near_euler(transform[:3, :3], [0, 0, 0])

    def chain_points(self, q_deg):
        """Base, six moving axis reference points, TCP; not collision geometry."""
        q = _array(q_deg, (6,), "q_deg")
        prefix = np.eye(4)
        points = [[0.0, 0.0, 0.0]]
        for axis, point, angle in zip(self.axes, self.axis_points, q):
            points.append((prefix[:3, :3] @ point + prefix[:3, 3]).tolist())
            prefix = prefix @ _axis_transform(axis, point, angle * DEG_TO_RAD)
        points.append((prefix @ self.home)[:3, 3].tolist())
        return points

    def jacobian(self, q_deg):
        """6x6 geometric J: position mm/deg, spatial angular velocity rad/deg."""
        return self._forward(q_deg, with_jacobian=True)[1]

    def solve(self, target_pose, q_seed, max_delta_deg=MAX_IK_JOINT_DELTA_DEG, max_iterations=40):
        """Damped local solve, with feasible steps throughout (never post-clipped).

        A successful result reaches both full TCP tolerances within the box;
        an unreachable/ill-conditioned local target returns explicit failure.
        The 2-degree iteration trust step does not alter the requested target.
        """
        seed = _array(q_seed, (6,), "q_seed")
        target = pose_matrix(target_pose)
        if (
            type(max_delta_deg) not in (int, float)
            or not math.isfinite(max_delta_deg)
            or not (0 < max_delta_deg <= 180)
        ):
            raise ValueError("max_delta_deg must be finite in (0, 180]")
        if type(max_iterations) is not int or not 1 <= max_iterations <= 1000:
            raise ValueError("max_iterations must be an integer in [1, 1000]")
        low, high = seed - max_delta_deg, seed + max_delta_deg
        q = seed.copy()
        max_iterate_delta = 0.0
        reason = "ITERATION_LIMIT"
        condition_number = math.inf

        def errors(transform):
            ep = target[:3, 3] - transform[:3, 3]
            er = rotation_log(target[:3, :3] @ transform[:3, :3].T)
            return ep, er, np.concatenate((ep, ROTATION_WEIGHT_MM * er))

        for iteration in range(max_iterations + 1):
            transform, jacobian = self._forward(q, with_jacobian=True)
            ep, er, error = errors(transform)
            position_error = float(np.linalg.norm(ep))
            orientation_error = math.degrees(float(np.linalg.norm(er)))
            if (
                position_error <= POSITION_TOLERANCE_MM
                and orientation_error <= ORIENTATION_TOLERANCE_DEG
            ):
                reason = "CONVERGED"
                break
            if iteration == max_iterations:
                break
            jacobian[3:] *= ROTATION_WEIGHT_MM
            singular_values = np.linalg.svd(jacobian, compute_uv=False)
            condition_number = (
                float(singular_values[0] / singular_values[-1])
                if (singular_values[-1] > 1e-12)
                else math.inf
            )
            gradient = jacobian.T @ error
            free = ~(((q - low <= 1e-9) & (gradient < 0)) | ((high - q <= 1e-9) & (gradient > 0)))
            if not np.any(free):
                reason = "LOCAL_JOINT_BOUND_REACHED"
                break
            reduced = jacobian[:, free]
            u, s, vt = np.linalg.svd(reduced, full_matrices=False)
            accepted = False
            cost = float(error @ error)
            for damping in (0.01, 0.1, 1.0, 10.0):
                step = np.zeros(6)
                step[free] = vt.T @ ((s / (s * s + damping * damping)) * (u.T @ error))
                peak = float(np.max(np.abs(step)))
                if peak < 1e-11:
                    continue
                if peak > MAX_ITERATION_STEP_DEG:
                    step *= MAX_ITERATION_STEP_DEG / peak
                fraction = 1.0
                for i in range(6):
                    if step[i] > 0:
                        fraction = min(fraction, (high[i] - q[i]) / step[i])
                    elif step[i] < 0:
                        fraction = min(fraction, (low[i] - q[i]) / step[i])
                if fraction <= 1e-12:
                    continue
                for reduction in range(12):
                    candidate = q + step * fraction * (0.5**reduction)
                    if np.any(candidate < low - 1e-10) or np.any(candidate > high + 1e-10):
                        raise ArithmeticError("Bounded solver generated an infeasible iterate")
                    _, _, candidate_error = errors(self._forward(candidate))
                    if float(candidate_error @ candidate_error) < cost - max(1e-14, cost * 1e-12):
                        q = candidate
                        max_iterate_delta = max(max_iterate_delta, float(np.max(np.abs(q - seed))))
                        accepted = True
                        break
                if accepted:
                    break
            if not accepted:
                reason = "NO_DESCENT_WITHIN_LOCAL_JOINT_BOUND"
                break
        success = reason == "CONVERGED"
        return {
            "success": success,
            "q": q.tolist(),
            "q_deg": q.tolist(),
            "position_error_mm": position_error,
            "orientation_error_deg": orientation_error,
            "reason": reason,
            "iterations": iteration,
            "max_iterate_delta_deg": max_iterate_delta,
            "max_joint_delta_deg": float(np.max(np.abs(q - seed))),
            "condition_number": condition_number if math.isfinite(condition_number) else None,
        }


def identify(samples):
    """Fit a zero pose and positive/negative single-axis FK samples for six axes.

    Sample format: {q_deg: [six degrees], pose_mm_deg: [XYZ mm, RPY degrees]}.
    Only +/- single-axis angles in (0, 90] degrees are accepted. Both signs are
    mandatory to detect bad pose conventions, non-revolute data or corruption.
    Withheld mixed-axis validation remains mandatory at the caller boundary.
    """
    if not isinstance(samples, (list, tuple)) or len(samples) != 13:
        raise ValueError("Expected exactly 13 FK samples (zero and six +/- axis pairs)")
    parsed, pairs, zeros = [], [[] for _ in range(6)], []
    for sample in samples:
        if not isinstance(sample, dict):
            raise ValueError("Each FK sample must be a dictionary")
        q = _array(sample.get("q_deg"), (6,), "sample q_deg")
        transform = pose_matrix(sample.get("pose_mm_deg"))
        parsed.append((q, transform))
        moving = np.flatnonzero(np.abs(q) > 1e-9)
        if not len(moving):
            zeros.append(transform)
        elif len(moving) == 1 and 1 <= abs(q[moving[0]]) <= 90:
            pairs[moving[0]].append((float(q[moving[0]]) * DEG_TO_RAD, transform))
        else:
            raise ValueError(
                "Identification requires isolated single-axis angles in [1, 90] degrees"
            )
    if len(zeros) != 1 or any(len(pair) != 2 or pair[0][0] * pair[1][0] >= 0 for pair in pairs):
        raise ValueError("Need one zero pose and both signs for every axis")
    home = zeros[0]
    inverse_home = np.eye(4)
    inverse_home[:3, :3] = home[:3, :3].T
    inverse_home[:3, 3] = -home[:3, :3].T @ home[:3, 3]
    axes, points = [], []
    for pair in pairs:
        relatives = [(angle, transform @ inverse_home) for angle, transform in pair]
        estimates = [rotation_log(transform[:3, :3]) / angle for angle, transform in relatives]
        if any(abs(np.linalg.norm(value) - 1) > 1e-3 for value in estimates):
            raise ValueError("FK rotation does not match the sampled joint angle")
        axis = np.mean(estimates, axis=0)
        axis /= np.linalg.norm(axis)
        if any(np.linalg.norm(value - axis) > 1e-3 for value in estimates):
            raise ValueError("Positive/negative FK rotations disagree on the joint axis")
        lhs = np.vstack(
            [np.eye(3) - transform[:3, :3] for _, transform in relatives] + [axis[None, :]]
        )
        rhs = np.concatenate([transform[:3, 3] for _, transform in relatives] + [np.zeros(1)])
        point = np.linalg.lstsq(lhs, rhs, rcond=None)[0]
        axes.append(axis)
        points.append(point)
    model = PoseModel(home, axes, points)
    position_errors, rotation_errors = [], []
    for q, actual in parsed:
        predicted = model._forward(q)
        position_errors.append(float(np.linalg.norm(predicted[:3, 3] - actual[:3, 3])))
        rotation_errors.append(
            math.degrees(float(np.linalg.norm(rotation_log(predicted[:3, :3] @ actual[:3, :3].T))))
        )
    if (
        max(position_errors) > POSITION_TOLERANCE_MM
        or max(rotation_errors) > ORIENTATION_TOLERANCE_DEG
    ):
        raise ValueError("Single-axis FK samples do not fit a consistent 6R POE model")
    model.fit_statistics = {
        "sample_count": len(samples),
        "max_position_error_mm": max(position_errors),
        "max_orientation_error_deg": max(rotation_errors),
        "independent_validation": "REQUIRED_NOT_PERFORMED_BY_IDENTIFY",
    }
    return model
