"""Pure six-axis ACK-committed filtered joint-reference generator.

The diagonal strictly convex velocity QP is solved exactly by projection:
    min_v .5 ||v - (q_filtered-q_ref)/tracking_tau||^2
          + .5 weight ||v-v_previous||^2
subject to velocity, acceleration and next-reference tracking boxes.

This adds dynamic lag while retaining unity gain for a feasible static goal.
It does not change the upstream TCP target or claim physical stability.
No hardware, sockets, clocks, files or external optimization solver are used.
The caller supplies measured dt and commits a proposal only after command ACK.
"""

import math

import numpy as np

MIN_DT_S = 0.02
MAX_DT_S = 0.25
FIRST_DT_S = 0.14
REFERENCE_MAX_VELOCITY_DEG_S = 25.0
REFERENCE_MAX_ACCELERATION_DEG_S2 = 50.0
REFERENCE_MAX_TRACKING_ERROR_DEG = 30.0


def _six(value, name):
    try:
        raw = np.asarray(value)
        if raw.dtype.kind not in "iuf" or any(
            isinstance(item, (bool, np.bool_)) for item in np.asarray(value, dtype=object).flat
        ):
            raise ValueError(name + " must contain six real numbers")
        result = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(name + " must contain six real numbers") from exc
    if result.shape != (6,) or not np.isfinite(result).all():
        raise ValueError(name + " must contain six finite numbers")
    return result.copy()


def _positive(value, name, *, allow_zero=False):
    if (
        type(value) not in (int, float)
        or not math.isfinite(value)
        or (value < 0 if allow_zero else value <= 0)
    ):
        raise ValueError(
            name + " must be finite and " + ("nonnegative" if allow_zero else "positive")
        )
    return float(value)


class ReferenceQP:
    """Stateful reference; propose() is pure, reset()/commit() advance state."""

    def __init__(
        self,
        goal_tau_s=0.20,
        tracking_tau_s=0.20,
        velocity_smoothing_weight=0.5,
        max_velocity_deg_s=REFERENCE_MAX_VELOCITY_DEG_S,
        max_acceleration_deg_s2=REFERENCE_MAX_ACCELERATION_DEG_S2,
        max_tracking_error_deg=REFERENCE_MAX_TRACKING_ERROR_DEG,
    ):
        self.goal_tau_s = _positive(goal_tau_s, "goal_tau_s")
        self.tracking_tau_s = _positive(tracking_tau_s, "tracking_tau_s")
        self.velocity_smoothing_weight = _positive(
            velocity_smoothing_weight, "velocity_smoothing_weight", allow_zero=True
        )
        self.max_velocity_deg_s = _positive(max_velocity_deg_s, "max_velocity_deg_s")
        self.max_acceleration_deg_s2 = _positive(max_acceleration_deg_s2, "max_acceleration_deg_s2")
        self.max_tracking_error_deg = _positive(max_tracking_error_deg, "max_tracking_error_deg")
        self._q_ref = self._velocity = self._filtered_goal = None
        self._generation = 0

    def reset(self, actual_q):
        actual = _six(actual_q, "actual_q")
        self._q_ref = actual.copy()
        self._filtered_goal = actual.copy()
        self._velocity = np.zeros(6)
        self._generation += 1
        return self.snapshot()

    def snapshot(self):
        return {
            "initialized": self._q_ref is not None,
            "generation": self._generation,
            "q_deg": None if self._q_ref is None else self._q_ref.tolist(),
            "velocity_deg_s": None if self._velocity is None else self._velocity.tolist(),
            "filtered_goal_deg": None
            if self._filtered_goal is None
            else self._filtered_goal.tolist(),
            "goal_tau_s": self.goal_tau_s,
            "tracking_tau_s": self.tracking_tau_s,
            "velocity_smoothing_weight": self.velocity_smoothing_weight,
            "max_velocity_deg_s": self.max_velocity_deg_s,
            "max_acceleration_deg_s2": self.max_acceleration_deg_s2,
            "max_tracking_error_deg": self.max_tracking_error_deg,
            "dt_range_s": [MIN_DT_S, MAX_DT_S],
        }

    def propose(self, goal_q, actual_q, dt):
        goal = _six(goal_q, "goal_q")
        actual = _six(actual_q, "actual_q")
        base = {
            "success": False,
            "generation": self._generation,
            "goal_q_deg": goal.tolist(),
            "actual_q_deg": actual.tolist(),
            "q_deg": None,
            "velocity_deg_s": None,
            "acceleration_deg_s2": None,
            "filtered_goal_deg": None,
            "active_constraints": {},
        }
        if self._q_ref is None:
            return {**base, "reason": "REFERENCE_NOT_INITIALIZED"}
        if type(dt) not in (int, float) or not math.isfinite(dt) or not MIN_DT_S <= dt <= MAX_DT_S:
            return {**base, "reason": "REFERENCE_DT_OUT_OF_RANGE"}
        dt = float(dt)
        base["dt_s"] = dt
        base["q_previous_deg"] = self._q_ref.tolist()
        base["velocity_previous_deg_s"] = self._velocity.tolist()
        alpha = -math.expm1(-dt / self.goal_tau_s)
        try:
            with np.errstate(over="raise", invalid="raise", divide="raise"):
                filtered = self._filtered_goal + alpha * (goal - self._filtered_goal)
                desired = (filtered - self._q_ref) / self.tracking_tau_s
                weight = self.velocity_smoothing_weight
                unconstrained = (desired + weight * self._velocity) / (1.0 + weight)
                acceleration_low = self._velocity - self.max_acceleration_deg_s2 * dt
                acceleration_high = self._velocity + self.max_acceleration_deg_s2 * dt
                tracking_low = (actual - self.max_tracking_error_deg - self._q_ref) / dt
                tracking_high = (actual + self.max_tracking_error_deg - self._q_ref) / dt
                low = np.maximum.reduce(
                    [np.full(6, -self.max_velocity_deg_s), acceleration_low, tracking_low]
                )
                high = np.minimum.reduce(
                    [np.full(6, self.max_velocity_deg_s), acceleration_high, tracking_high]
                )
                base.update(
                    filtered_goal_deg=filtered.tolist(),
                    desired_velocity_deg_s=desired.tolist(),
                    unconstrained_velocity_deg_s=unconstrained.tolist(),
                    velocity_lower_deg_s=low.tolist(),
                    velocity_upper_deg_s=high.tolist(),
                    filter_alpha=alpha,
                )
                infeasible = np.flatnonzero(low > high)
                if len(infeasible):
                    return {
                        **base,
                        "reason": "REFERENCE_TRACKING_BOX_INFEASIBLE",
                        "infeasible_joints": [f"J{i + 1}" for i in infeasible],
                    }
                # H=(1+w)I is positive definite. Coordinate-wise projection
                # gives the unique global minimizer for this separable box QP.
                velocity = np.minimum(np.maximum(unconstrained, low), high)
                acceleration = (velocity - self._velocity) / dt
                reference = self._q_ref + dt * velocity
                objective = 0.5 * float(np.sum((velocity - desired) ** 2)) + (
                    0.5 * weight * float(np.sum((velocity - self._velocity) ** 2))
                )
        except FloatingPointError:
            return {**base, "reason": "REFERENCE_NUMERICAL_ERROR"}
        if not np.isfinite(np.concatenate((reference, velocity, acceleration, filtered))).all():
            return {**base, "reason": "REFERENCE_NUMERICAL_ERROR"}
        if (
            np.any(np.abs(velocity) > self.max_velocity_deg_s + 1e-9)
            or np.any(np.abs(acceleration) > self.max_acceleration_deg_s2 + 1e-9)
            or np.any(np.abs(reference - actual) > self.max_tracking_error_deg + 1e-9)
        ):
            return {**base, "reason": "REFERENCE_CONSTRAINT_CHECK_FAILED"}
        active = {}
        for i in range(6):
            constraints = []
            for name, bound in (
                ("velocity_min", -self.max_velocity_deg_s),
                ("velocity_max", self.max_velocity_deg_s),
                ("acceleration_min", acceleration_low[i]),
                ("acceleration_max", acceleration_high[i]),
                ("tracking_min", tracking_low[i]),
                ("tracking_max", tracking_high[i]),
            ):
                if abs(velocity[i] - bound) <= 1e-9:
                    constraints.append(name)
            if constraints:
                active[f"J{i + 1}"] = constraints
        return {
            **base,
            "success": True,
            "reason": "QP_OPTIMAL",
            "q_deg": reference.tolist(),
            "velocity_deg_s": velocity.tolist(),
            "acceleration_deg_s2": acceleration.tolist(),
            "active_constraints": active,
            "tracking_error_deg": (reference - actual).tolist(),
            "objective": objective,
            "solver": "ANALYTIC_DIAGONAL_BOX_QP",
        }

    def commit(self, proposal):
        """Commit exactly one successful proposal after its command ACK.

        Recomputed checks reject stale or modified proposals. This method
        cannot observe hardware ACK; the transport-owning caller enforces it.
        """
        if not isinstance(proposal, dict) or proposal.get("success") is not True:
            raise ValueError("Only a successful reference proposal can be committed")
        if proposal.get("generation") != self._generation:
            raise ValueError("Reference proposal is stale")
        try:
            expected = self.propose(
                proposal["goal_q_deg"], proposal["actual_q_deg"], proposal["dt_s"]
            )
            if not expected["success"]:
                raise ValueError("Reference proposal is no longer feasible")
            for key in ("q_deg", "velocity_deg_s", "acceleration_deg_s2", "filtered_goal_deg"):
                candidate = _six(proposal[key], key)
                if not np.allclose(candidate, expected[key], rtol=0, atol=1e-10):
                    raise ValueError("Reference proposal was modified: " + key)
        except KeyError as exc:
            raise ValueError("Incomplete reference proposal") from exc
        self._q_ref = np.asarray(expected["q_deg"], dtype=float)
        self._velocity = np.asarray(expected["velocity_deg_s"], dtype=float)
        self._filtered_goal = np.asarray(expected["filtered_goal_deg"], dtype=float)
        self._generation += 1
        return self.snapshot()
