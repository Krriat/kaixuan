#include "kaixuan_can_hardware/joint_angle_mapping.hpp"
#include <cstdlib>
#include <iostream>
using namespace kaixuan_can_hardware;
void check(bool condition, const char *message)
{if (!condition) {std::cerr << message << '\n'; std::exit(1);}}
int main()
{
  JointAngleMapping joint5{101.0, 274.56318430, 1.0, true};
  const double current = joint5.position(27824384) * 180.0 / kJointAnglePi;
  check(std::abs(current + 179.98156) < 0.00001, "joint5 current angle should be -179.98156 degrees");
  for (double direction : {-1.0, 1.0}) {
    joint5.direction = direction;
    for (std::int64_t count = -2000000000LL; count <= 2000000000LL; count += 123456789LL) {
      const auto actual = static_cast<std::int32_t>(count);
      const double angle = joint5.position(actual);
      check(angle >= -kJointAnglePi && angle < kJointAnglePi, "normalized feedback range");
      check(joint5.target_counts(angle, actual) == actual, "normalized feedback must hold the same encoder turn");
      check(joint5.target_counts(angle + 6 * kJointAnglePi, actual) == actual, "equivalent commands must not move a full turn");
    }
  }
  joint5.direction = 1;
  for (std::int32_t count : {27817959, 27818959, 27819959}) {
    check(joint5.target_counts(joint5.position(count), count) == count, "round trip on both sides of -pi");
  }
  for (double degrees : {-900.0, -540.0, -180.0, 180.0, 540.0, 900.0}) {
    const double angle = normalize_joint_angle(degrees * kJointAnglePi / 180.0);
    check(std::abs(angle + kJointAnglePi) < 1e-12, "half-turn endpoints map to -pi");
  }
  JointAngleMapping linear{81.0, -69.48424021, -1.0, false};
  check(linear.target_counts(linear.position(-16393389), 0) == -16393389, "unwrapped joint conversion stays compatible");
  std::cout << "Joint angle normalization and encoder-turn preservation checks passed\n";
}
