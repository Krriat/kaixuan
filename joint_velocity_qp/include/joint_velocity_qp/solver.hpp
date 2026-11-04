#pragma once

#include <Eigen/Core>

#include <limits>
#include <optional>
#include <string>

namespace joint_velocity_qp {

struct Bounds {
  Eigen::VectorXd lower;
  Eigen::VectorXd upper;
};

struct PositionLimits {
  Eigen::VectorXd position;
  Bounds limits;
};

struct AccelerationLimits {
  Eigen::VectorXd previous_velocity;
  Bounds limits;
};

struct LinearConstraints {
  Eigen::MatrixXd matrix;
  Bounds limits;
};

struct Problem {
  Eigen::MatrixXd jacobian;          // m x n
  Eigen::VectorXd desired_velocity;  // m
  Bounds velocity_limits;           // n; required, +/-infinity allowed

  // Empty vectors mean unit task weights and zero reference velocity.
  Eigen::VectorXd task_weights;       // m, nonnegative diagonal entries of W
  Eigen::VectorXd reference_velocity; // n

  // Optional hard constraints. dt must be positive when either is present.
  double dt = 0.0;
  std::optional<PositionLimits> position_limits;
  std::optional<AccelerationLimits> acceleration_limits;
  std::optional<LinearConstraints> linear_constraints;
};

struct Options {
  // Cost: 0.5*(J*dq-v)'*W*(J*dq-v) + 0.5*regularization*||dq-dq_ref||^2.
  double regularization = 1e-4;
  double absolute_tolerance = 1e-7;
  double relative_tolerance = 1e-7;
  // Additional absolute check on every assembled row of l <= A*dq <= u.
  double constraint_tolerance = 1e-6;
  int max_iterations = 10000;
  bool polish = true;
};

enum class Status {
  Solved,
  InvalidInput,
  Infeasible,
  IterationLimit,
  Inaccurate,
  ConstraintViolation,
  SolverError,
};

struct Result {
  Status status = Status::SolverError;
  std::string message;
  // These vectors remain empty unless status == Solved.
  Eigen::VectorXd joint_velocity;
  Eigen::VectorXd achieved_velocity;
  double tracking_error = std::numeric_limits<double>::quiet_NaN();
  double max_constraint_violation = std::numeric_limits<double>::infinity();
  int iterations = 0;
  int osqp_status = 0;

  bool ok() const noexcept { return status == Status::Solved; }
};

// Each call owns a fresh OSQP workspace. No global state or ROS dependency.
Result solve(const Problem& problem, const Options& options = {});
const char* statusName(Status status) noexcept;

}  // namespace joint_velocity_qp
