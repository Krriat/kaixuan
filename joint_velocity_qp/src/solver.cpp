#include "joint_velocity_qp/solver.hpp"

#include <Eigen/SparseCore>
#include <osqp.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <vector>

namespace joint_velocity_qp {
namespace {

static_assert(std::is_same_v<OSQPFloat, double>,
              "Build OSQP with OSQP_USE_FLOAT=OFF (double precision).");
using SparseMatrix = Eigen::SparseMatrix<OSQPFloat, Eigen::ColMajor, OSQPInt>;
using Triplet = Eigen::Triplet<OSQPFloat, OSQPInt>;

void require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::invalid_argument(message);
  }
}

void checkVector(const Eigen::VectorXd& vector, Eigen::Index size,
                 const std::string& name) {
  require(vector.size() == size, name + ": dimension mismatch");
  require(vector.allFinite(), name + ": values must be finite");
}

void checkBounds(const Bounds& bounds, Eigen::Index size,
                 const std::string& name) {
  require(bounds.lower.size() == size && bounds.upper.size() == size,
          name + ": dimension mismatch");
  for (Eigen::Index i = 0; i < size; ++i) {
    const double lower = bounds.lower[i];
    const double upper = bounds.upper[i];
    require(!std::isnan(lower) && !std::isnan(upper), name + ": NaN bound");
    require(lower != std::numeric_limits<double>::infinity() &&
                upper != -std::numeric_limits<double>::infinity(),
            name + ": lower may only use -infinity; upper only +infinity");
    require(lower <= upper, name + ": lower exceeds upper");
    require((!std::isfinite(lower) || std::abs(lower) < OSQP_INFTY) &&
                (!std::isfinite(upper) || std::abs(upper) < OSQP_INFTY),
            name + ": finite bound exceeds OSQP numeric range");
  }
}

// Preserve explicit infinity, but reject overflow of a finite derived bound.
double positionBound(double bound, double position, double dt) {
  if (std::isinf(bound)) {
    return bound;
  }
  const double value = (bound - position) / dt;
  require(std::isfinite(value), "position bound overflow");
  return value;
}

double accelerationBound(double bound, double previous_velocity, double dt) {
  if (std::isinf(bound)) {
    return bound;
  }
  const double value = previous_velocity + dt * bound;
  require(std::isfinite(value), "acceleration bound overflow");
  return value;
}

OSQPCscMatrix cscView(SparseMatrix& matrix) {
  matrix.makeCompressed();
  OSQPCscMatrix view{};
  // The Eigen storage outlives both this view and the solver workspace.
  OSQPCscMatrix_set_data(&view, matrix.rows(), matrix.cols(), matrix.nonZeros(),
                        matrix.valuePtr(), matrix.innerIndexPtr(),
                        matrix.outerIndexPtr());
  return view;
}

struct SolverDeleter {
  void operator()(OSQPSolver* solver) const noexcept {
    if (solver != nullptr) {
      osqp_cleanup(solver);
    }
  }
};

Result run(const Problem& problem, const Options& options) {
  const auto& J = problem.jacobian;
  const auto m = J.rows();
  const auto n = J.cols();
  require(m > 0 && n > 0, "jacobian must have positive dimensions");
  require(J.allFinite(), "jacobian must be finite");
  checkVector(problem.desired_velocity, m, "desired_velocity");
  checkBounds(problem.velocity_limits, n, "velocity_limits");
  require(std::isfinite(options.regularization) && options.regularization > 0.0,
          "regularization must be finite and positive");
  require(std::isfinite(options.absolute_tolerance) &&
              options.absolute_tolerance > 0.0 &&
              std::isfinite(options.relative_tolerance) &&
              options.relative_tolerance > 0.0 &&
              std::isfinite(options.constraint_tolerance) &&
              options.constraint_tolerance >= 0.0,
          "invalid solver tolerances");
  require(options.max_iterations > 0, "max_iterations must be positive");

  Eigen::VectorXd weights = Eigen::VectorXd::Ones(m);
  if (problem.task_weights.size() != 0) {
    checkVector(problem.task_weights, m, "task_weights");
    require((problem.task_weights.array() >= 0.0).all(),
            "task_weights must be nonnegative");
    weights = problem.task_weights;
  }
  Eigen::VectorXd reference = Eigen::VectorXd::Zero(n);
  if (problem.reference_velocity.size() != 0) {
    checkVector(problem.reference_velocity, n, "reference_velocity");
    reference = problem.reference_velocity;
  }

  if (problem.position_limits || problem.acceleration_limits) {
    require(std::isfinite(problem.dt) && problem.dt > 0.0,
            "dt must be finite and positive for position/acceleration limits");
  }

  Bounds velocity = problem.velocity_limits;
  if (problem.position_limits) {
    const auto& limits = *problem.position_limits;
    checkVector(limits.position, n, "position");
    checkBounds(limits.limits, n, "position_limits");
    for (Eigen::Index i = 0; i < n; ++i) {
      velocity.lower[i] = std::max(velocity.lower[i], positionBound(
          limits.limits.lower[i], limits.position[i], problem.dt));
      velocity.upper[i] = std::min(velocity.upper[i], positionBound(
          limits.limits.upper[i], limits.position[i], problem.dt));
    }
  }
  if (problem.acceleration_limits) {
    const auto& limits = *problem.acceleration_limits;
    checkVector(limits.previous_velocity, n, "previous_velocity");
    checkBounds(limits.limits, n, "acceleration_limits");
    for (Eigen::Index i = 0; i < n; ++i) {
      velocity.lower[i] = std::max(velocity.lower[i], accelerationBound(
          limits.limits.lower[i], limits.previous_velocity[i], problem.dt));
      velocity.upper[i] = std::min(velocity.upper[i], accelerationBound(
          limits.limits.upper[i], limits.previous_velocity[i], problem.dt));
    }
  }

  Eigen::Index extra_rows = 0;
  if (problem.linear_constraints) {
    const auto& constraints = *problem.linear_constraints;
    require(constraints.matrix.cols() == n && constraints.matrix.allFinite(),
            "linear constraint matrix must be finite with n columns");
    extra_rows = constraints.matrix.rows();
    checkBounds(constraints.limits, extra_rows, "linear_constraints");
  }

  Result result;
  if ((velocity.lower.array() > velocity.upper.array()).any()) {
    result.status = Status::Infeasible;
    result.message = "velocity, position and acceleration limits have empty intersection";
    return result;
  }
  checkBounds(velocity, n, "combined velocity limits");

  const Eigen::MatrixXd weighted_J = weights.asDiagonal() * J;
  Eigen::MatrixXd P_dense = J.transpose() * weighted_J;
  P_dense.diagonal().array() += options.regularization;
  Eigen::VectorXd q = -J.transpose() *
      (weights.array() * problem.desired_velocity.array()).matrix()
      - options.regularization * reference;
  require(P_dense.allFinite() && q.allFinite(), "QP objective overflow");

  // OSQP consumes only the upper triangle of P, in compressed column format.
  SparseMatrix P = P_dense.triangularView<Eigen::Upper>().toDenseMatrix().sparseView();
  std::vector<Triplet> entries;
  entries.reserve(static_cast<std::size_t>(n));
  for (Eigen::Index i = 0; i < n; ++i) {
    entries.emplace_back(i, i, 1.0);
  }
  if (problem.linear_constraints) {
    const auto& matrix = problem.linear_constraints->matrix;
    for (Eigen::Index col = 0; col < n; ++col) {
      for (Eigen::Index row = 0; row < extra_rows; ++row) {
        if (matrix(row, col) != 0.0) {
          entries.emplace_back(n + row, col, matrix(row, col));
        }
      }
    }
  }
  SparseMatrix A(n + extra_rows, n);
  A.setFromTriplets(entries.begin(), entries.end());
  Eigen::VectorXd lower(n + extra_rows);
  Eigen::VectorXd upper(n + extra_rows);
  lower.head(n) = velocity.lower;
  upper.head(n) = velocity.upper;
  if (problem.linear_constraints) {
    lower.tail(extra_rows) = problem.linear_constraints->limits.lower;
    upper.tail(extra_rows) = problem.linear_constraints->limits.upper;
  }
  // Keep original infinities for the independent post-solve constraint check.
  const Eigen::VectorXd lower_original = lower;
  const Eigen::VectorXd upper_original = upper;
  for (Eigen::Index i = 0; i < lower.size(); ++i) {
    if (std::isinf(lower[i])) {
      lower[i] = -OSQP_INFTY;
    }
    if (std::isinf(upper[i])) {
      upper[i] = OSQP_INFTY;
    }
  }

  OSQPCscMatrix P_csc = cscView(P);
  OSQPCscMatrix A_csc = cscView(A);
  OSQPSettings settings{};
  osqp_set_default_settings(&settings);
  settings.verbose = 0;
  settings.polishing = options.polish ? 1 : 0;
  settings.max_iter = options.max_iterations;
  settings.eps_abs = options.absolute_tolerance;
  settings.eps_rel = options.relative_tolerance;

  OSQPSolver* raw_solver = nullptr;
  const OSQPInt setup_error = osqp_setup(&raw_solver, &P_csc, q.data(), &A_csc,
      lower.data(), upper.data(), A.rows(), n, &settings);
  std::unique_ptr<OSQPSolver, SolverDeleter> solver(raw_solver);
  if (setup_error != 0 || !solver) {
    result.message = "osqp_setup failed, error=" + std::to_string(setup_error);
    return result;
  }
  const OSQPInt solve_error = osqp_solve(solver.get());
  if (solve_error != 0 || solver->info == nullptr) {
    result.message = "osqp_solve failed, error=" + std::to_string(solve_error);
    return result;
  }
  result.osqp_status = static_cast<int>(solver->info->status_val);
  result.iterations = static_cast<int>(solver->info->iter);
  result.message = solver->info->status;
  switch (solver->info->status_val) {
    case OSQP_SOLVED:
      break;
    case OSQP_PRIMAL_INFEASIBLE:
    case OSQP_PRIMAL_INFEASIBLE_INACCURATE:
      result.status = Status::Infeasible;
      return result;
    case OSQP_MAX_ITER_REACHED:
      result.status = Status::IterationLimit;
      return result;
    case OSQP_SOLVED_INACCURATE:
      result.status = Status::Inaccurate;
      return result;
    default:
      return result;
  }
  if (solver->solution == nullptr || solver->solution->x == nullptr) {
    result.message = "OSQP returned no solution";
    return result;
  }
  const Eigen::VectorXd dq = Eigen::Map<const Eigen::VectorXd>(solver->solution->x, n);
  const Eigen::VectorXd constrained = A * dq;
  const Eigen::VectorXd achieved = J * dq;
  if (!dq.allFinite() || !constrained.allFinite() || !achieved.allFinite()) {
    result.message = "non-finite solution or constraint evaluation";
    return result;
  }
  result.max_constraint_violation = 0.0;
  for (Eigen::Index i = 0; i < constrained.size(); ++i) {
    result.max_constraint_violation = std::max(result.max_constraint_violation,
        std::max(lower_original[i] - constrained[i], constrained[i] - upper_original[i]));
  }
  if (result.max_constraint_violation > options.constraint_tolerance) {
    result.status = Status::ConstraintViolation;
    result.message = "solution exceeds constraint_tolerance";
    return result;
  }
  result.status = Status::Solved;
  result.joint_velocity = dq;
  result.achieved_velocity = achieved;
  result.tracking_error = (achieved - problem.desired_velocity).norm();
  return result;
}

}  // namespace

Result solve(const Problem& problem, const Options& options) {
  try {
    return run(problem, options);
  } catch (const std::invalid_argument& error) {
    Result result;
    result.status = Status::InvalidInput;
    result.message = error.what();
    return result;
  }
}

const char* statusName(Status status) noexcept {
  switch (status) {
    case Status::Solved: return "solved";
    case Status::InvalidInput: return "invalid_input";
    case Status::Infeasible: return "infeasible";
    case Status::IterationLimit: return "iteration_limit";
    case Status::Inaccurate: return "inaccurate";
    case Status::ConstraintViolation: return "constraint_violation";
    case Status::SolverError: return "solver_error";
  }
  return "unknown";
}

}  // namespace joint_velocity_qp
