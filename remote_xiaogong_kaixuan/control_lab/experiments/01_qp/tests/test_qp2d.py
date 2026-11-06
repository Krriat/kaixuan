from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
import osqp

from kaixuan_control_lab.qp2d import (active_constraints, admm_trace, enumerate_reference,
                                    kkt_residuals, make_problem, solve_osqp)


class QP2DTests(unittest.TestCase):
    def test_hand_derived_optima_and_multipliers(self):
        cases = (
            ([0.2, -0.1], [0.2, -0.1], [0, 0, 0, 0, 0]),
            ([1., 1.], [.6, .2], [0, 0, 0, 0, .8]),
            ([2., 1.], [1., -.2], [.8, 0, 0, 0, 1.2]),
            ([-2., -2.], [-1., -1.], [0, 2, 0, 1, 0]),
            ([2., -2.], [1., -1.], [2, 0, 0, 1, 0]),
        )
        for r, optimum, multipliers in cases:
            with self.subTest(reference=r):
                problem = make_problem(r)
                self.assertLess(max(kkt_residuals(problem, optimum, multipliers).values()), 1e-12)
                reference = enumerate_reference(problem)
                np.testing.assert_allclose(reference.z, optimum, atol=1e-12)
                solution = solve_osqp(problem)
                self.assertTrue(solution.accepted, solution.rejection)
                np.testing.assert_allclose(solution.z, optimum, rtol=0, atol=1e-7)

    def test_random_targets_match_independent_active_set_enumeration(self):
        for r in np.random.default_rng(20261106).uniform(-3, 3, (64, 2)):
            problem = make_problem(r)
            solution = solve_osqp(problem)
            self.assertTrue(solution.accepted, solution.rejection)
            reference = enumerate_reference(problem)
            np.testing.assert_allclose(solution.z, reference.z, rtol=0, atol=1e-6)

    def test_geometrically_active_does_not_require_positive_multiplier(self):
        problem = make_problem([2., 1.8])
        reference = enumerate_reference(problem)
        self.assertEqual(set(active_constraints(problem, reference.z)), {"z1_upper", "sum_upper"})
        # This target is precisely where z1_upper enters; its multiplier is zero.
        self.assertAlmostEqual(reference.multipliers[0], 0)

    def test_infeasible_problem_exposes_no_solution(self):
        problem = make_problem([1., 1.], sum_upper=-2.5)
        self.assertIsNone(enumerate_reference(problem))
        result = solve_osqp(problem)
        self.assertEqual(result.status_value, int(osqp.SolverStatus.OSQP_PRIMAL_INFEASIBLE))
        self.assertFalse(result.accepted)
        self.assertIsNone(result.z)
        self.assertIsNone(result.multipliers)

    def test_iteration_limit_exposes_no_solution(self):
        result = solve_osqp(make_problem([2., 2.]), max_iter=1)
        self.assertEqual(result.status_value, int(osqp.SolverStatus.OSQP_MAX_ITER_REACHED))
        self.assertFalse(result.accepted)
        self.assertIsNone(result.z)

    def test_solved_status_alone_does_not_accept_invalid_numerical_result(self):
        info = SimpleNamespace(status="solved", status_val=int(osqp.SolverStatus.OSQP_SOLVED),
                               iter=1, prim_res=0., dual_res=0., setup_time=0., solve_time=0., run_time=0.)
        for candidate in ([10., 10.], [float("nan"), 0.], [0., 0.]):
            with self.subTest(candidate=candidate), patch("kaixuan_control_lab.qp2d.osqp.OSQP") as factory:
                # Even a feasible zero vector is not optimal for r=(1,1).
                factory.return_value.solve.return_value = SimpleNamespace(info=info, x=candidate, y=np.zeros(5))
                result = solve_osqp(make_problem([1., 1.]))
                self.assertFalse(result.accepted)
                self.assertIsNone(result.z)

    def test_teaching_admm_reaches_reference_and_small_residuals(self):
        problem = make_problem([1., 1.])
        final = admm_trace(problem)[-1]
        np.testing.assert_allclose([final["z1"], final["z2"]], [.6, .2], rtol=0, atol=1e-7)
        for key in ("primal_residual", "dual_residual", "stationarity", "complementarity"):
            self.assertLess(final[key], 1e-7)

    def test_rejects_invalid_problem_and_iteration_inputs(self):
        for r in ([0.], [float("nan"), 0.], [0., float("inf")]):
            with self.assertRaises(ValueError):
                make_problem(r)
        with self.assertRaises(ValueError):
            make_problem([0., 0.], sum_upper=float("nan"))
        with self.assertRaises(ValueError):
            solve_osqp(make_problem([0., 0.]), max_iter=0)


if __name__ == "__main__":
    unittest.main()
