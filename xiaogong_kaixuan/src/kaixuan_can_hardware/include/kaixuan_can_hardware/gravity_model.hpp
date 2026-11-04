#pragma once
#include <array>
#include <memory>
#include <string>
#include <vector>

namespace kaixuan_can_hardware
{
// Fixed-base URDF dynamics. Angles are ROS joint radians; outputs are joint N.m.
class GravityModel
{
public:
  GravityModel();
  ~GravityModel();
  void configure(const std::string & urdf, const std::array<double, 3> & gravity,
    const std::string & payload_frame = "", double payload_mass = 0.0,
    const std::array<double, 3> & payload_com = {0.0, 0.0, 0.0});
  const std::vector<std::string> & joints() const;
  const std::vector<double> & compute(const std::vector<double> & positions);
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace kaixuan_can_hardware
