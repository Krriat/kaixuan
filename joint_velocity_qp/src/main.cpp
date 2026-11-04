#include "joint_velocity_qp/solver.hpp"

#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace qp = joint_velocity_qp;

namespace {

double readNumber(std::istream& input) {
  std::string token;
  if (!(input >> token)) {
    throw std::invalid_argument("unexpected end of input");
  }
  std::size_t used = 0;
  const double value = std::stod(token, &used);
  if (used != token.size()) {
    throw std::invalid_argument("invalid number: " + token);
  }
  return value;
}

Eigen::VectorXd readVector(std::istream& input, Eigen::Index size) {
  Eigen::VectorXd vector(size);
  for (Eigen::Index i = 0; i < size; ++i) {
    vector[i] = readNumber(input);
  }
  return vector;
}

qp::Problem readProblem(std::istream& input) {
  long long m = 0;
  long long n = 0;
  if (!(input >> m >> n) || m <= 0 || n <= 0 ||
      n > 1000000 || m > 1000000 / n) {
    throw std::invalid_argument("expected positive m n with m*n <= 1000000");
  }
  qp::Problem problem;
  problem.jacobian.resize(m, n);
  for (Eigen::Index row = 0; row < m; ++row) {
    for (Eigen::Index col = 0; col < n; ++col) {
      problem.jacobian(row, col) = readNumber(input);
    }
  }
  problem.desired_velocity = readVector(input, m);
  problem.velocity_limits.lower = readVector(input, n);
  problem.velocity_limits.upper = readVector(input, n);
  std::string trailing;
  if (input >> trailing) {
    throw std::invalid_argument("unexpected trailing input: " + trailing);
  }
  return problem;
}

qp::Problem demoProblem() {
  qp::Problem problem;
  problem.jacobian = Eigen::MatrixXd::Zero(6, 7);
  problem.jacobian.leftCols(6).setIdentity();
  problem.jacobian.col(6) << 0.3, -0.2, 0.1, 0.0, 0.2, 0.1;
  problem.desired_velocity.resize(6);
  problem.desired_velocity << 0.3, -0.2, 0.1, 0.05, 0.1, -0.05;
  problem.velocity_limits = {Eigen::VectorXd::Constant(7, -0.5),
                             Eigen::VectorXd::Constant(7, 0.5)};
  problem.dt = 0.01;
  qp::PositionLimits position;
  position.position = Eigen::VectorXd::Zero(7);
  position.position[0] = 0.999;  // First joint can move at most +0.1 rad/s.
  position.limits = {Eigen::VectorXd::Constant(7, -1.0),
                     Eigen::VectorXd::Constant(7, 1.0)};
  problem.position_limits = position;
  problem.acceleration_limits = qp::AccelerationLimits{
      Eigen::VectorXd::Zero(7),
      {Eigen::VectorXd::Constant(7, -10.0), Eigen::VectorXd::Constant(7, 10.0)}};
  qp::LinearConstraints linear;
  linear.matrix = Eigen::MatrixXd::Zero(1, 7);
  linear.matrix(0, 0) = 1.0;
  linear.matrix(0, 1) = -1.0;
  linear.limits = {Eigen::VectorXd::Constant(1, -std::numeric_limits<double>::infinity()),
                   Eigen::VectorXd::Constant(1, 0.15)};
  problem.linear_constraints = linear;
  return problem;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "Usage: joint_velocity_qp_cli [--demo | --help]\n"
                 "Default: read m n, J (m rows, n columns), v (m), "
                 "dq_min (n), dq_max (n) from stdin.\n"
                 "Bounds accept -inf/inf. Output: n joint velocities on stdout.\n";
    return 0;
  }
  const bool demo = argc == 2 && std::string(argv[1]) == "--demo";
  if (argc != 1 && !demo) {
    std::cerr << "invalid arguments; use --help\n";
    return 2;
  }
  try {
    const qp::Problem problem = demo ? demoProblem() : readProblem(std::cin);
    // Some OSQP 1.0 builds print a polishing message even with verbose=0.
    // Keep stdout suitable for piping when using an externally installed OSQP.
    qp::Options options;
    options.polish = false;
    const qp::Result result = qp::solve(problem, options);
    std::cerr << "status=" << qp::statusName(result.status)
              << ", message=" << result.message
              << ", iterations=" << result.iterations << '\n';
    if (!result.ok()) {
      return result.status == qp::Status::InvalidInput ? 2 : 1;
    }
    std::cout << std::setprecision(17) << result.joint_velocity.transpose() << '\n';
    std::cerr << "tracking_error=" << result.tracking_error
              << ", max_constraint_violation=" << result.max_constraint_violation << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "input_error: " << error.what() << '\n';
    return 2;
  }
}
