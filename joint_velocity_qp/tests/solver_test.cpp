#include "joint_velocity_qp/solver.hpp"

#include <Eigen/Cholesky>

#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace qp = joint_velocity_qp;

namespace {

void expect(bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void expectNear(const Eigen::VectorXd& actual, const Eigen::VectorXd& expected,
                double tolerance = 2e-6) {
  expect(actual.size() == expected.size(), "solution dimension mismatch");
  expect((actual - expected).lpNorm<Eigen::Infinity>() < tolerance,
         "solution differs from analytical reference");
}

qp::Problem identityProblem(int n) {
  qp::Problem problem;
  problem.jacobian = Eigen::MatrixXd::Identity(n, n);
  problem.desired_velocity = Eigen::VectorXd::Ones(n);
  problem.velocity_limits = {Eigen::VectorXd::Constant(n, -10.0),
                             Eigen::VectorXd::Constant(n, 10.0)};
  return problem;
}

void expectFailure(const qp::Result& result, qp::Status expected) {
  expect(result.status == expected, "unexpected failure status: " + result.message);
  expect(result.joint_velocity.size() == 0 && result.achieved_velocity.size() == 0,
         "failed solve exposed a velocity command");
}

void weightedLeastSquares() {
  auto problem = identityProblem(7);
  problem.jacobian.resize(6, 7);
  for (Eigen::Index row = 0; row < 6; ++row) {
    for (Eigen::Index col = 0; col < 7; ++col) {
      problem.jacobian(row, col) = std::sin(0.7 * row + 0.3 * col) +
                                  (row == col ? 1.0 : 0.0);
    }
  }
  problem.desired_velocity = Eigen::VectorXd::LinSpaced(6, -0.3, 0.4);
  problem.task_weights = Eigen::VectorXd::LinSpaced(6, 0.5, 2.0);
  problem.reference_velocity = Eigen::VectorXd::LinSpaced(7, -0.1, 0.2);
  problem.velocity_limits.lower.setConstant(-std::numeric_limits<double>::infinity());
  problem.velocity_limits.upper.setConstant(std::numeric_limits<double>::infinity());
  qp::Options options;
  options.regularization = 0.05;
  const Eigen::MatrixXd hessian = problem.jacobian.transpose() *
      problem.task_weights.asDiagonal() * problem.jacobian +
      options.regularization * Eigen::MatrixXd::Identity(7, 7);
  const Eigen::VectorXd rhs = problem.jacobian.transpose() *
      problem.task_weights.asDiagonal() * problem.desired_velocity +
      options.regularization * problem.reference_velocity;
  const Eigen::VectorXd expected = hessian.ldlt().solve(rhs);
  const auto result = qp::solve(problem, options);
  expect(result.ok(), result.message);
  expectNear(result.joint_velocity, expected);
  expectNear(result.achieved_velocity, problem.jacobian * expected);
}

void velocitySaturation() {
  auto problem = identityProblem(2);
  problem.desired_velocity << 2.0, -3.0;
  problem.velocity_limits.lower << -0.3, -0.4;
  problem.velocity_limits.upper << 0.3, 0.4;
  const auto result = qp::solve(problem);
  expect(result.ok(), result.message);
  expectNear(result.joint_velocity, (Eigen::Vector2d() << 0.3, -0.4).finished());
  expect(result.tracking_error > 1.0, "unreachable task must retain tracking error");
  expect(result.max_constraint_violation <= 1e-6, "velocity limit violation");
}

void singularJacobian() {
  auto problem = identityProblem(3);
  problem.jacobian.resize(2, 3);
  problem.jacobian << 1, 0, 1, 2, 0, 2;
  problem.desired_velocity = (Eigen::Vector2d() << 1.0, 2.0).finished();
  qp::Options options;
  options.regularization = 0.01;
  const double active = 5.0 / (10.0 + options.regularization);
  const auto result = qp::solve(problem, options);
  expect(result.ok(), result.message);
  expectNear(result.joint_velocity, (Eigen::Vector3d() << active, 0, active).finished());
}

void positionAndAccelerationLimits() {
  auto problem = identityProblem(3);
  problem.desired_velocity << 1, -1, 1;
  problem.velocity_limits.lower.setConstant(-0.5);
  problem.velocity_limits.upper.setConstant(0.5);
  problem.dt = 0.1;
  problem.position_limits = qp::PositionLimits{
      (Eigen::Vector3d() << 0.999, -0.999, 0).finished(),
      {Eigen::Vector3d::Constant(-1), Eigen::Vector3d::Constant(1)}};
  problem.acceleration_limits = qp::AccelerationLimits{
      Eigen::Vector3d::Zero(),
      {Eigen::Vector3d::Constant(-0.5), Eigen::Vector3d::Constant(0.5)}};
  const auto result = qp::solve(problem);
  expect(result.ok(), result.message);
  expectNear(result.joint_velocity, (Eigen::Vector3d() << 0.01, -0.01, 0.05).finished());
  const Eigen::VectorXd next = problem.position_limits->position +
                              problem.dt * result.joint_velocity;
  expect((next.array().abs() <= 1.0 + 1e-8).all(), "next position exceeds limits");
  const Eigen::VectorXd acceleration = (result.joint_velocity -
      problem.acceleration_limits->previous_velocity) / problem.dt;
  expect((acceleration.array().abs() <= 0.5 + 1e-7).all(), "acceleration exceeds limits");
}

void coupledInequality() {
  auto problem = identityProblem(2);
  problem.linear_constraints = qp::LinearConstraints{
      Eigen::MatrixXd::Ones(1, 2),
      {Eigen::VectorXd::Constant(1, -std::numeric_limits<double>::infinity()),
       Eigen::VectorXd::Constant(1, 0.5)}};
  const auto result = qp::solve(problem);
  expect(result.ok(), result.message);
  expectNear(result.joint_velocity, Eigen::Vector2d::Constant(0.25));
}

void coupledEquality() {
  auto problem = identityProblem(2);
  problem.desired_velocity << 1, 0;
  problem.linear_constraints = qp::LinearConstraints{
      Eigen::MatrixXd::Ones(1, 2),
      {Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1)}};
  const auto result = qp::solve(problem);
  const double x = 0.5 / (1.0 + qp::Options{}.regularization);
  expect(result.ok(), result.message);
  expectNear(result.joint_velocity, (Eigen::Vector2d() << x, -x).finished());
}

void infeasibleLinearConstraints() {
  auto problem = identityProblem(2);
  problem.velocity_limits.lower.setConstant(-0.2);
  problem.velocity_limits.upper.setConstant(0.2);
  problem.linear_constraints = qp::LinearConstraints{
      Eigen::MatrixXd::Ones(1, 2),
      {Eigen::VectorXd::Constant(1, 1.5),
       Eigen::VectorXd::Constant(1, std::numeric_limits<double>::infinity())}};
  expectFailure(qp::solve(problem), qp::Status::Infeasible);
}

void infeasibleIntersection() {
  auto problem = identityProblem(1);
  problem.dt = 0.1;
  problem.position_limits = qp::PositionLimits{
      Eigen::VectorXd::Constant(1, 0.99),
      {Eigen::VectorXd::Constant(1, -1), Eigen::VectorXd::Constant(1, 1)}};
  problem.acceleration_limits = qp::AccelerationLimits{
      Eigen::VectorXd::Constant(1, 1.0),
      {Eigen::VectorXd::Constant(1, -0.1), Eigen::VectorXd::Constant(1, 0.1)}};
  expectFailure(qp::solve(problem), qp::Status::Infeasible);
}

void invalidInputs() {
  using Mutation = std::function<void(qp::Problem&)>;
  const std::vector<Mutation> mutations = {
      [](auto& p) { p.desired_velocity.resize(1); },
      [](auto& p) { p.velocity_limits.lower.resize(1); },
      [](auto& p) { p.jacobian(0, 0) = std::numeric_limits<double>::quiet_NaN(); },
      [](auto& p) { p.desired_velocity[0] = std::numeric_limits<double>::infinity(); },
      [](auto& p) { p.velocity_limits.lower[0] = 20; },
      [](auto& p) { p.velocity_limits.upper[0] = -std::numeric_limits<double>::infinity(); },
      [](auto& p) { p.task_weights = Eigen::Vector2d::Constant(-1); },
      [](auto& p) { p.reference_velocity = Eigen::VectorXd::Zero(1); },
      [](auto& p) {
        p.position_limits = qp::PositionLimits{Eigen::Vector2d::Zero(),
            {Eigen::Vector2d::Constant(-1), Eigen::Vector2d::Constant(1)}};
        p.dt = 0;
      },
      [](auto& p) {
        p.linear_constraints = qp::LinearConstraints{Eigen::MatrixXd::Zero(1, 3),
            {Eigen::VectorXd::Zero(1), Eigen::VectorXd::Ones(1)}};
      },
      [](auto& p) { p.jacobian.setConstant(1e300); },
  };
  for (const auto& mutate : mutations) {
    auto problem = identityProblem(2);
    mutate(problem);
    expectFailure(qp::solve(problem), qp::Status::InvalidInput);
  }
  for (const double invalid : {0.0, -1.0, std::numeric_limits<double>::infinity()}) {
    qp::Options options;
    options.regularization = invalid;
    expectFailure(qp::solve(identityProblem(2), options), qp::Status::InvalidInput);
  }
}

void iterationLimit() {
  auto problem = identityProblem(2);
  problem.desired_velocity << 2, -3;
  problem.velocity_limits.lower.setConstant(-0.1);
  problem.velocity_limits.upper.setConstant(0.1);
  qp::Options options;
  options.max_iterations = 1;
  expectFailure(qp::solve(problem, options), qp::Status::IterationLimit);
}

void zeroTaskWeights() {
  auto problem = identityProblem(2);
  problem.task_weights = Eigen::Vector2d::Zero();
  problem.reference_velocity = (Eigen::Vector2d() << 0.2, -0.3).finished();
  const auto result = qp::solve(problem);
  expect(result.ok(), result.message);
  expectNear(result.joint_velocity, problem.reference_velocity, 1e-4);
}

}  // namespace

int main() {
  const std::vector<std::pair<const char*, std::function<void()>>> tests = {
      {"weighted least squares", weightedLeastSquares},
      {"velocity saturation", velocitySaturation},
      {"singular Jacobian", singularJacobian},
      {"position and acceleration limits", positionAndAccelerationLimits},
      {"coupled inequality", coupledInequality},
      {"coupled equality", coupledEquality},
      {"infeasible linear constraints", infeasibleLinearConstraints},
      {"infeasible limit intersection", infeasibleIntersection},
      {"invalid input validation", invalidInputs},
      {"iteration limit", iterationLimit},
      {"zero task weights", zeroTaskWeights},
  };
  int failures = 0;
  for (const auto& test : tests) {
    try {
      test.second();
      std::cout << "PASS " << test.first << '\n';
    } catch (const std::exception& error) {
      ++failures;
      std::cerr << "FAIL " << test.first << ": " << error.what() << '\n';
    }
  }
  return failures == 0 ? 0 : 1;
}
