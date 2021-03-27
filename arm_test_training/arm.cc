#include "arm.hh"
#include <bits/types/error_t.h>

constexpr double ArmLoop::MAX_ANGLE, ArmLoop::MIN_ANGLE, ArmLoop::DT;

// Unused parameters are commented out to avoid compiler warnings.
double ArmLoop::update(double encoder, bool /*lower_limit*/, bool /*upper_limit*/, bool /*enabled*/) {
  double kP = 50.0;
  double kD = 1.;
  double error = goal_ - encoder;
  double vel = (error - last_error_) / DT;
  last_error_ = error;
  return kP * error + kD * vel;
}
