"""Pure measured-anchor, tool-relative pose mapping; no robot or network calls.

At Grip engagement, hand local axes are assigned to the measured tool local
axes. This relative contract is not an external/world-frame calibration:

    R_map = R_robot_anchor @ R_hand_anchor.T
    p_target = p_robot_anchor + 1000 * R_map @ (p_hand - p_hand_anchor)
    R_target = R_map @ R_hand

Positions use metres for hand inputs and millimetres for robot outputs. Robot
angles are fixed-axis XYZ degrees, R = Rz @ Ry @ Rx. This module computes a
target only; motion permission, IK and robot limits belong to the caller.
"""

import math


def _vector(value, length, name):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{name} must contain {length} numbers")
    if any(type(x) not in (int, float) or not math.isfinite(x) for x in value):
        raise ValueError(f"{name} must contain finite numbers")
    return tuple(float(x) for x in value)


def _hand_pose(pose):
    if not isinstance(pose, dict):
        raise ValueError("Hand pose must be a dictionary")
    position = _vector(pose.get("position_m"), 3, "position_m")
    quaternion = _vector(pose.get("quaternion_xyzw"), 4, "quaternion_xyzw")
    norm2 = sum(x * x for x in quaternion)
    if abs(norm2 - 1.0) > 1e-3:
        raise ValueError("Hand quaternion must be unit length")
    # Match the existing feedback contract: correct validated roundoff only.
    x, y, z, w = (x / math.sqrt(norm2) for x in quaternion)
    rotation = (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )
    return position, rotation


def _multiply(a, b):
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3)
    )


def _euler_rotation(angles):
    x, y, z = (math.radians(value) for value in angles)
    sx, cx, sy, cy, sz, cz = (
        math.sin(x),
        math.cos(x),
        math.sin(y),
        math.cos(y),
        math.sin(z),
        math.cos(z),
    )
    return (
        (cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx),
        (sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx),
        (-sy, cy * sx, cy * cx),
    )


def _nearest_angle(angle, reference):
    return reference + (angle - reference + 180.0) % 360.0 - 180.0


def _near_euler(rotation, anchor):
    """Choose equivalent XYZ angles nearest the anchor, including at gimbal lock.

    This avoids gratuitous 360-degree wraps and Euler-branch flips relative to
    the engagement pose. A stateless Euler result cannot promise continuity
    for every path through the Euler singularity.
    """
    r = rotation
    cosine_y = math.hypot(r[0][0], r[1][0])
    y = math.degrees(math.atan2(-r[2][0], cosine_y))
    if cosine_y <= 1e-10:
        # At +/-90 deg, only x - sign(y)*z is determined. Split its required
        # change equally between x and z to minimize distance to the anchor.
        sign = 1 if -r[2][0] >= 0 else -1
        coupled = math.degrees(math.atan2(sign * r[0][1], r[1][1]))
        anchor_coupled = anchor[0] - sign * anchor[2]
        change = _nearest_angle(coupled, anchor_coupled) - anchor_coupled
        return [
            anchor[0] + change / 2,
            _nearest_angle(y, anchor[1]),
            anchor[2] - sign * change / 2,
        ]

    x = math.degrees(math.atan2(r[2][1], r[2][2]))
    z = math.degrees(math.atan2(r[1][0], r[0][0]))
    candidates = [
        [_nearest_angle(value, ref) for value, ref in zip(candidate, anchor)]
        for candidate in ((x, y, z), (x + 180, 180 - y, z + 180))
    ]
    return min(candidates, key=lambda values: sum((v - a) ** 2 for v, a in zip(values, anchor)))


def relative_pose_target(hand_anchor, robot_anchor_pose_mm_deg, hand_current):
    """Return six finite robot target values [X,Y,Z,Rx,Ry,Rz] in mm/degrees.

    Hand arguments contain position_m[3] and unit quaternion_xyzw[4]. Inputs
    are read without mutation. Invalid numeric poses raise ValueError; no
    synthetic pose, fallback calibration or target clamping is substituted.
    """
    anchor_position, anchor_rotation = _hand_pose(hand_anchor)
    current_position, current_rotation = _hand_pose(hand_current)
    robot_anchor = _vector(robot_anchor_pose_mm_deg, 6, "robot_anchor_pose_mm_deg")
    robot_rotation = _euler_rotation(robot_anchor[3:])
    mapping = _multiply(robot_rotation, tuple(zip(*anchor_rotation)))
    displacement = tuple(current_position[i] - anchor_position[i] for i in range(3))
    position = [
        robot_anchor[i] + 1000 * sum(mapping[i][j] * displacement[j] for j in range(3))
        for i in range(3)
    ]
    orientation = _near_euler(_multiply(mapping, current_rotation), robot_anchor[3:])
    target = position + orientation
    if not all(math.isfinite(value) for value in target):
        raise ValueError("Relative pose target is not finite")
    return target


def grip_direction_mapping(hand_at_grip, robot_pose_mm_deg):
    """Freeze only the agreed direction transform, never a cumulative target base."""
    _, hand_rotation = _hand_pose(hand_at_grip)
    robot = _vector(robot_pose_mm_deg, 6, "robot_pose_mm_deg")
    return _multiply(_euler_rotation(robot[3:]), tuple(zip(*hand_rotation)))


def incremental_pose_target(
    hand_previous, robot_measured_pose_mm_deg, hand_current, direction_mapping
):
    """One window delta added to this window's measured robot pose (1:1).

    C is fixed for the agreed Grip direction convention. Rotation increments
    are spatial: C*(R_now*R_prev.T)*C.T, left-composed with measured R_robot.
    The original relative_pose_target stays unchanged for historical log replay.
    """
    previous_position, previous_rotation = _hand_pose(hand_previous)
    current_position, current_rotation = _hand_pose(hand_current)
    robot = _vector(robot_measured_pose_mm_deg, 6, "robot_measured_pose_mm_deg")
    if not isinstance(direction_mapping, (list, tuple)) or len(direction_mapping) != 3:
        raise ValueError("direction_mapping must be a proper 3x3 rotation")
    c = tuple(_vector(row, 3, "direction_mapping") for row in direction_mapping)
    identity = _multiply(c, tuple(zip(*c)))
    determinant = (
        c[0][0] * (c[1][1] * c[2][2] - c[1][2] * c[2][1])
        - c[0][1] * (c[1][0] * c[2][2] - c[1][2] * c[2][0])
        + c[0][2] * (c[1][0] * c[2][1] - c[1][1] * c[2][0])
    )
    if abs(determinant - 1) > 1e-6 or any(
        abs(identity[i][j] - int(i == j)) > 1e-6 for i in range(3) for j in range(3)
    ):
        raise ValueError("direction_mapping must be a proper rotation, without scale")
    displacement = tuple(current_position[i] - previous_position[i] for i in range(3))
    position = [
        robot[i] + 1000 * sum(c[i][j] * displacement[j] for j in range(3)) for i in range(3)
    ]
    hand_delta = _multiply(current_rotation, tuple(zip(*previous_rotation)))
    robot_delta = _multiply(_multiply(c, hand_delta), tuple(zip(*c)))
    orientation = _near_euler(_multiply(robot_delta, _euler_rotation(robot[3:])), robot[3:])
    target = position + orientation
    if not all(math.isfinite(value) for value in target):
        raise ValueError("Incremental target is not finite")
    return target


def anchored_position_target(
    hand_anchor, robot_anchor_pose_mm_deg, hand_previous,
    robot_measured_pose_mm_deg, hand_current, direction_mapping
):
    """B mode: retained grip-anchored WORLD position, incremental orientation.

    Translation is p_robot_at_grip + C*(p_hand_now-p_hand_at_grip), with
    metre-to-mm conversion. Actual robot lag cannot move this position goal.
    Rotation keeps the existing measured-base one-window increment contract.
    A new Grip must supply new measured anchors; this function never stores
    or replays positions from a released grip.
    """
    anchored = incremental_pose_target(
        hand_anchor, robot_anchor_pose_mm_deg, hand_current, direction_mapping
    )
    incremental = incremental_pose_target(
        hand_previous, robot_measured_pose_mm_deg, hand_current, direction_mapping
    )
    return anchored[:3] + incremental[3:]


def anchored_pose_target(hand_anchor, robot_anchor_pose_mm_deg, hand_current, direction_mapping):
    """Retain world translation AND spatial rotation relative to this Grip.

    R_goal = C (R_hand_now R_hand_grip.T) C.T R_robot_grip.
    This uses rotations, not Euler subtraction. Measured robot lag and consumed
    input windows cannot erase the goal. Fresh anchors are mandatory on regrip.
    The caller subsequently projects orientation to the downward cone.
    """
    return incremental_pose_target(
        hand_anchor, robot_anchor_pose_mm_deg, hand_current, direction_mapping
    )
