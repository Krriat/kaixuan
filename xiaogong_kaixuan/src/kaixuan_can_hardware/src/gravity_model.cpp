#include "kaixuan_can_hardware/gravity_model.hpp"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <cmath>
#include <stdexcept>

namespace kaixuan_can_hardware
{
struct GravityModel::Impl
{
  pinocchio::Model model;
  std::unique_ptr<pinocchio::Data> data;
  Eigen::VectorXd q;
  std::vector<std::string> names;
  std::vector<int> q_indices, v_indices;
  std::vector<double> torques;
};
GravityModel::GravityModel() = default;
GravityModel::~GravityModel() = default;

void GravityModel::configure(const std::string & urdf, const std::array<double, 3> & gravity,
  const std::string & payload_frame, double payload_mass, const std::array<double, 3> & payload_com)
{
  auto next = std::make_unique<Impl>();
  pinocchio::urdf::buildModelFromXML(urdf, next->model);
  const Eigen::Vector3d g(gravity[0], gravity[1], gravity[2]);
  if (!g.allFinite() || g.norm() < 1.0 || g.norm() > 20.0 ||
    !std::isfinite(payload_mass) || payload_mass < 0.0)
  {throw std::invalid_argument("Invalid gravity vector or payload mass");}
  next->model.gravity.linear() = g;
  const Eigen::Vector3d com(payload_com[0], payload_com[1], payload_com[2]);
  if (!com.allFinite()) {throw std::invalid_argument("Invalid payload center of mass");}
  if (payload_mass > 0.0) {
    if (!next->model.existFrame(payload_frame, pinocchio::BODY)) {throw std::invalid_argument("Unknown payload link frame");}
    const auto & frame = next->model.frames[next->model.getFrameId(payload_frame, pinocchio::BODY)];
    next->model.appendBodyToJoint(frame.parentJoint,
      pinocchio::Inertia(payload_mass, com, Eigen::Matrix3d::Zero()), frame.placement);
  }
  for (pinocchio::JointIndex i = 1; i < next->model.joints.size(); ++i) {
    const auto & joint = next->model.joints[i];
    if (joint.nq() != 1 || joint.nv() != 1) {
      throw std::invalid_argument("Gravity controller requires fixed-base, single-DoF joints");
    }
    const auto & inertia = next->model.inertias[i];
    if (!std::isfinite(inertia.mass()) || inertia.mass() < 0 || !inertia.lever().allFinite()) {
      throw std::invalid_argument("Invalid URDF mass or center of mass");
    }
    next->names.push_back(next->model.names[i]);
    next->q_indices.push_back(joint.idx_q());
    next->v_indices.push_back(joint.idx_v());
  }
  if (next->names.empty()) {throw std::invalid_argument("URDF has no controlled joints");}
  next->data = std::make_unique<pinocchio::Data>(next->model);
  next->q = Eigen::VectorXd::Zero(next->model.nq);
  next->torques.resize(next->names.size());
  impl_ = std::move(next);
}

const std::vector<std::string> & GravityModel::joints() const
{
  if (!impl_) {throw std::logic_error("Gravity model is not configured");}
  return impl_->names;
}

const std::vector<double> & GravityModel::compute(const std::vector<double> & positions)
{
  if (!impl_ || positions.size() != impl_->names.size()) {
    throw std::invalid_argument("Gravity model joint state size mismatch");
  }
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (!std::isfinite(positions[i])) {throw std::invalid_argument("Nonfinite joint position");}
    impl_->q[impl_->q_indices[i]] = positions[i];
  }
  const auto & g = pinocchio::computeGeneralizedGravity(impl_->model, *impl_->data, impl_->q);
  for (std::size_t i = 0; i < positions.size(); ++i) {
    const double value = g[impl_->v_indices[i]];
    if (!std::isfinite(value)) {throw std::runtime_error("Nonfinite gravity torque");}
    impl_->torques[i] = value;
  }
  return impl_->torques;
}
}  // namespace kaixuan_can_hardware
