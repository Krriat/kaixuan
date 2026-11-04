#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace kaixuan_can_hardware
{
struct FrictionParameters
{
  std::string model{"disabled"};
  double coulomb_nm{0}, viscous_nm_s_per_rad{0};
  double min_speed_rad_s{0}, max_speed_rad_s{0};
  double limit_nm{0}, slew_nm_s{0};
  std::vector<double> speeds, torques;

  void validate(double effort_limit) const
  {
    if (model == "disabled") {return;}
    if (model != "coulomb_viscous" && model != "velocity_lookup") {
      throw std::invalid_argument("Unknown friction model: " + model);
    }
    for (double v : {coulomb_nm, viscous_nm_s_per_rad, min_speed_rad_s,
        max_speed_rad_s, limit_nm, slew_nm_s})
    {if (!std::isfinite(v) || v < 0) {throw std::invalid_argument("Invalid friction parameter");}}
    if (min_speed_rad_s <= 0 || max_speed_rad_s < min_speed_rad_s ||
      limit_nm <= 0 || limit_nm > effort_limit || slew_nm_s <= 0)
    {throw std::invalid_argument("Invalid friction speed/effort/slew bound");}
    if (model == "velocity_lookup") {
      if (speeds.size() < 2 || speeds.size() != torques.size()) {
        throw std::invalid_argument("Friction lookup requires matching speed/torque knots");
      }
      for (std::size_t i = 0; i < speeds.size(); ++i) {
        if (!std::isfinite(speeds[i]) || speeds[i] <= 0 ||
          !std::isfinite(torques[i]) || torques[i] < 0 ||
          (i && speeds[i] <= speeds[i-1]))
        {throw std::invalid_argument("Invalid friction lookup knots");}
      }
    }
  }

  double magnitude(double speed) const
  {
    if (model == "disabled") {return 0;}
    // Never extrapolate a low-speed viscous fit to the hardware speed limit.
    speed = std::clamp(speed, min_speed_rad_s, max_speed_rad_s);
    if (model == "coulomb_viscous") {return coulomb_nm + viscous_nm_s_per_rad * speed;}
    if (speed <= speeds.front()) {return torques.front();}
    if (speed >= speeds.back()) {return torques.back();}
    const auto upper = std::upper_bound(speeds.begin(), speeds.end(), speed);
    const auto i = static_cast<std::size_t>(upper - speeds.begin());
    const double weight = (speed-speeds[i-1]) / (speeds[i]-speeds[i-1]);
    return torques[i-1] + weight * (torques[i]-torques[i-1]);
  }
};

class FrictionCompensation
{
public:
  FrictionParameters parameters;
  void reset() {filtered_velocity_ = output_ = 0; direction_ = 0;}
  void track_applied(double value) {output_ = value;}
  double update(double velocity, double dt, double ramp, double scale,
    double deadband, double full_speed, double filter_time)
  {
    if (!std::isfinite(velocity) || !std::isfinite(dt) || dt <= 0 ||
      !std::isfinite(ramp) || ramp < 0 || ramp > 1 ||
      !std::isfinite(scale) || scale < 0 || scale > 1 ||
      !std::isfinite(deadband) || deadband < 0 ||
      !std::isfinite(full_speed) || full_speed <= deadband ||
      !std::isfinite(filter_time) || filter_time < 0)
    {throw std::invalid_argument("Invalid friction update input");}
    if (parameters.model == "disabled" || scale == 0 || ramp == 0 ||
      std::abs(velocity) <= deadband)
    {reset(); return 0;}
    const int direction = velocity > 0 ? 1 : -1;
    // Remove assistance immediately at a stop/reversal, rather than keeping an
    // old-direction torque while a filter or symmetric slew limiter decays.
    if (direction_ && direction != direction_) {
      reset(); direction_ = direction; return 0;
    }
    direction_ = direction;
    const double alpha = filter_time > 0 ? dt / (filter_time + dt) : 1;
    filtered_velocity_ += alpha * (velocity-filtered_velocity_);
    const double speed = std::abs(filtered_velocity_);
    const double x = std::clamp((speed-deadband)/(full_speed-deadband), 0.0, 1.0);
    const double gate = x*x*(3-2*x);
    const double target = direction * std::min(parameters.limit_nm,
      scale * ramp * gate * parameters.magnitude(speed));
    const double step = parameters.slew_nm_s * dt;
    output_ += std::clamp(target-output_, -step, step);
    return output_;
  }
private:
  double filtered_velocity_{0}, output_{0};
  int direction_{0};
};
}  // namespace kaixuan_can_hardware
