#include "kaixuan_can_hardware/lqr_servo.hpp"
#include <fstream>
#include <iostream>
#include <vector>
#include <stdexcept>
using kaixuan_can_hardware::LqrServo;
using kaixuan_can_hardware::LqrSetpoint;
int main(int argc, char **argv)
{
  if (argc != 2) {return 2;}
  std::ifstream file(argv[1], std::ios::binary);
  auto read = [&]() {double x; file.read(reinterpret_cast<char *>(&x), sizeof(x));
    if (!file) {throw std::runtime_error("Incomplete numerical fixture");} return x;};
  LqrServo servo;
  const auto steps = static_cast<unsigned>(read());
  servo.channel_count = static_cast<unsigned>(read());
  for (unsigned i = 0; i < servo.channel_count; ++i) {
    servo.channels[i].joint = static_cast<unsigned>(read());
    servo.channels[i].delay = static_cast<unsigned>(read());
    servo.channels[i].time_constant = read();
  }
  for (auto * v : {&servo.limit, &servo.increase, &servo.release, &servo.integral_limit}) {
    for (auto & x : *v) {x = read();}
  }
  double maximum_error = 0.;
  for (unsigned step = 0; step < steps; ++step) {
    LqrSetpoint s; s.valid = true;
    s.mode = static_cast<int>(read()); s.phase = static_cast<int>(read()); s.scale = read();
    const double dt = read();
    LqrServo::Vec q{}, v{};
    for (auto * target : {&s.q, &s.v, &s.feedforward, &q, &v}) {
      for (auto & x : *target) {x = read();}
    }
    for (unsigned i = 0; i < 7; ++i) {
      for (unsigned j = 0; j < servo.states(); ++j) {s.gain[i*LqrSetpoint::MAX_STATE+j] = read();}
    }
    for (auto & x : s.inverse_integral) {x = read();}
    if (!servo.step(s, q, v, dt)) {throw std::runtime_error("Valid numerical step rejected");}
    for (const auto * actual : {&servo.previous, &servo.raw, &servo.integral, &servo.z,
        &servo.state_feedback, &servo.memory_feedback}) {
      for (const auto x : *actual) {maximum_error = std::max(maximum_error, std::abs(x-read()));}
    }
    for (unsigned i = 0; i < servo.states()-21; ++i) {
      maximum_error = std::max(maximum_error, std::abs(servo.input_state[i]-read()));
    }
    if (maximum_error > 1e-9) {
      std::cerr << "Python/C++ mismatch at step " << step << ": " << maximum_error << '\n'; return 1;
    }
  }
  std::cout << steps << " coupled LQI steps match Python; max error " << maximum_error << '\n';
  return 0;
}
