"""Pure QP/reference checks; no physical closed-loop stability claim."""

import copy
import unittest

import numpy as np

from joint_reference_qp import ReferenceQP


class ReferenceQPTests(unittest.TestCase):
    def test_analytic_result_satisfies_box_kkt_and_objective_minimum(self):
        qp = ReferenceQP()
        qp.reset([0] * 6)
        proposal = qp.propose([-100, 100, 1, -1, 10, -10], [0, 0, 0, 0, -29.9, 29.9], 0.14)
        self.assertTrue(proposal["success"])
        v = np.array(proposal["velocity_deg_s"])
        lo, hi = (
            np.array(proposal["velocity_lower_deg_s"]),
            np.array(proposal["velocity_upper_deg_s"]),
        )
        desired = np.array(proposal["desired_velocity_deg_s"])
        previous = np.array(proposal["velocity_previous_deg_s"])
        gradient = (v - desired) + 0.5 * (v - previous)
        for i in range(6):
            if abs(v[i] - lo[i]) < 1e-9:
                self.assertGreaterEqual(gradient[i], -1e-9)
            elif abs(v[i] - hi[i]) < 1e-9:
                self.assertLessEqual(gradient[i], 1e-9)
            else:
                self.assertAlmostEqual(gradient[i], 0.0, places=10)
        rng = np.random.default_rng(793)
        candidates = rng.uniform(lo, hi, size=(1000, 6))
        costs = 0.5 * np.sum((candidates - desired) ** 2, axis=1) + 0.25 * np.sum(
            (candidates - previous) ** 2, axis=1
        )
        self.assertTrue(np.all(costs >= proposal["objective"] - 1e-9))
        self.assertIn("tracking_max", proposal["active_constraints"]["J5"])
        self.assertIn("tracking_min", proposal["active_constraints"]["J6"])

    def test_velocity_acceleration_and_tracking_limits_on_reversals(self):
        qp = ReferenceQP()
        qp.reset([0] * 6)
        prior_q = np.zeros(6)
        prior_velocity = np.zeros(6)
        for step in range(70):
            dt = (0.02, 0.07, 0.14, 0.20, 0.25)[step % 5]
            goal = [100 if (step // 10) % 2 == 0 else -100] * 6
            proposal = qp.propose(goal, prior_q, dt)
            self.assertTrue(proposal["success"], proposal)
            q, v = np.array(proposal["q_deg"]), np.array(proposal["velocity_deg_s"])
            a = np.array(proposal["acceleration_deg_s2"])
            self.assertLessEqual(np.max(np.abs(v)), 25 + 1e-9)
            self.assertLessEqual(np.max(np.abs(a)), 50 + 1e-9)
            self.assertLessEqual(np.max(np.abs(q - prior_q)), 30 + 1e-9)
            np.testing.assert_allclose(q, prior_q + v * dt, rtol=0, atol=1e-12)
            np.testing.assert_allclose(a, (v - prior_velocity) / dt, rtol=0, atol=1e-12)
            qp.commit(proposal)
            prior_q, prior_velocity = q, v

    def test_reversing_goal_does_not_reverse_full_speed_in_one_step(self):
        qp = ReferenceQP()
        qp.reset([0] * 6)
        q = [0] * 6
        for _ in range(20):
            proposal = qp.propose([100] * 6, q, 0.14)
            qp.commit(proposal)
            q = proposal["q_deg"]
        self.assertAlmostEqual(proposal["velocity_deg_s"][0], 25)
        reversed_proposal = qp.propose([-100] * 6, q, 0.14)
        self.assertTrue(reversed_proposal["success"])
        self.assertGreaterEqual(reversed_proposal["velocity_deg_s"][0], 18.0 - 1e-9)
        self.assertGreater(reversed_proposal["q_deg"][0], q[0])

    def test_static_target_retains_unity_gain_with_dynamic_lag(self):
        qp = ReferenceQP()
        qp.reset([0] * 6)
        goal = np.array([4, -3, 2, 1, -2, 3], dtype=float)
        actual = np.zeros(6)
        first = qp.propose(goal, actual, 0.14)
        self.assertFalse(np.allclose(first["q_deg"], goal))
        for _ in range(200):
            proposal = qp.propose(goal, actual, 0.14)
            self.assertTrue(proposal["success"], proposal)
            qp.commit(proposal)
            # Ideal follower is only a numerical reference-generation fixture.
            actual = np.array(proposal["q_deg"])
        np.testing.assert_allclose(actual, goal, rtol=0, atol=1e-8)
        np.testing.assert_allclose(proposal["velocity_deg_s"], np.zeros(6), rtol=0, atol=1e-8)

    def test_proposal_does_not_mutate_state_and_ack_commit_happens_once(self):
        qp = ReferenceQP()
        qp.reset([1, 2, 3, 4, 5, 6])
        before = qp.snapshot()
        proposal = qp.propose([2, 3, 4, 5, 6, 7], [1, 2, 3, 4, 5, 6], 0.14)
        repeated = qp.propose([2, 3, 4, 5, 6, 7], [1, 2, 3, 4, 5, 6], 0.14)
        self.assertEqual(qp.snapshot(), before)
        self.assertEqual(proposal, repeated)
        qp.commit(proposal)
        self.assertEqual(qp.snapshot()["q_deg"], proposal["q_deg"])
        with self.assertRaisesRegex(ValueError, "stale"):
            qp.commit(repeated)
        after = qp.snapshot()
        with self.assertRaises(ValueError):
            qp.commit({"success": False})
        self.assertEqual(qp.snapshot(), after)

    def test_infeasible_tracking_box_fails_without_reset_or_state_change(self):
        qp = ReferenceQP()
        qp.reset([0] * 6)
        before = qp.snapshot()
        proposal = qp.propose([10] * 6, [31] * 6, 0.14)
        # At zero velocity the next step is <= .98 deg, insufficient to enter
        # [actual-30, actual+30]=[1,61]. Do not jump q_ref to actual.
        self.assertFalse(proposal["success"])
        self.assertEqual(proposal["reason"], "REFERENCE_TRACKING_BOX_INFEASIBLE")
        self.assertEqual(proposal["infeasible_joints"], [f"J{i}" for i in range(1, 7)])
        self.assertIsNone(proposal["q_deg"])
        self.assertEqual(qp.snapshot(), before)
        with self.assertRaises(ValueError):
            qp.commit(proposal)

    def test_reference_integrates_prior_reference_not_new_actual_measurement(self):
        qp = ReferenceQP()
        qp.reset([0] * 6)
        proposal = qp.propose([2] * 6, [0] * 6, 0.14)
        qp.commit(proposal)
        prior = np.array(proposal["q_deg"])
        # A measurement perturbation within the tracking box is not a reset.
        next_proposal = qp.propose([2] * 6, prior + 0.7, 0.14)
        self.assertTrue(next_proposal["success"])
        np.testing.assert_allclose(
            next_proposal["q_deg"], prior + 0.14 * np.array(next_proposal["velocity_deg_s"])
        )

    def test_dt_rejection_and_modified_commit_never_advance_state(self):
        qp = ReferenceQP()
        self.assertFalse(qp.propose([0] * 6, [0] * 6, 0.14)["success"])
        qp.reset([0] * 6)
        before = qp.snapshot()
        for dt in (0, 0.0199, 0.2501, float("nan"), float("inf"), True):
            proposal = qp.propose([1] * 6, [0] * 6, dt)
            self.assertFalse(proposal["success"])
            self.assertEqual(proposal["reason"], "REFERENCE_DT_OUT_OF_RANGE")
            self.assertEqual(qp.snapshot(), before)
        proposal = qp.propose([1] * 6, [0] * 6, 0.14)
        bad = copy.deepcopy(proposal)
        bad["q_deg"][0] += 0.01
        with self.assertRaisesRegex(ValueError, "modified"):
            qp.commit(bad)
        self.assertEqual(qp.snapshot(), before)
        qp.reset([0] * 6)
        with self.assertRaisesRegex(ValueError, "stale"):
            qp.commit(proposal)

    def test_invalid_inputs_and_settings_rejected(self):
        for kwargs in (
            {"goal_tau_s": 0},
            {"tracking_tau_s": -1},
            {"velocity_smoothing_weight": -1},
            {"max_velocity_deg_s": float("inf")},
            {"max_acceleration_deg_s2": True},
            {"max_tracking_error_deg": 0},
        ):
            with self.assertRaises(ValueError):
                ReferenceQP(**kwargs)
        qp = ReferenceQP()
        for q in ([0] * 5, [float("nan")] * 6, [True, 0, 0, 0, 0, 0], ["1"] * 6):
            with self.assertRaises(ValueError):
                qp.reset(q)


if __name__ == "__main__":
    unittest.main()
