#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>

#include "training/arm.hh"

class ArmTest : public ::testing::Test {
protected:
  double velocity_ = 0;
  double position_ = 0;

  ArmTest() {}

  // Kt : T = Kt*i -> Kt = T / i
  // Kv : Vemf = 1/Kv * w -> Kv = w/Vemf -> Kv = w/(V - R*i)
  // R  : V / i (when acceleration is 0)

  // N*m
  static constexpr double STALL_TORQUE = 0.71;

  // A
  static constexpr double STALL_CURRENT = 134.0;

  // rad/s
  static constexpr double FREE_SPEED = 18730.0 / 60.0 * 2.0 * M_PI;

  // A
  static constexpr double FREE_CURRENT = 0.7;

  // Kg N / m^2
  static constexpr double MOMENT = 1.43750187;

  // unitless
  static constexpr double GEAR_RATIO = (12.0 / 60.0) * (18.0 / 84.0) * (24.0 / 84.0) * (12.0 / 60.0);

  // kg
  static constexpr double ARM_MASS = 6;

  // gravity (m/s^2)
  static constexpr double GRAVITY = 9.81;

  static constexpr double RESISTANCE = 12.0 / STALL_CURRENT;
  static constexpr double KV = FREE_SPEED / (12.0 - RESISTANCE - FREE_CURRENT);
  static constexpr double KT = STALL_TORQUE / STALL_CURRENT;

  double get_acceleration(double voltage) {
    return (KT / (GEAR_RATIO * RESISTANCE * MOMENT)) * voltage -
      (KT / (GEAR_RATIO * GEAR_RATIO * KV * RESISTANCE * MOMENT)) * velocity_;
  }

  void simulate_for_time(double voltage, double time) {
    // seconds
    double sim_dt = 0.0001;
    double current_time = 0;

    while (current_time < time) {
      const double accel = get_acceleration(voltage);
      position_ += velocity_ * sim_dt;
      velocity_ += accel * sim_dt;
      current_time += sim_dt;
    }
  }

  bool upper_limit_triggered() {
    return position_ > ArmLoop::MAX_ANGLE;
  }

  bool lower_limit_triggered() { return position_ < ArmLoop::MIN_ANGLE; }
};

TEST_F(ArmTest, Zeros) {
  ArmLoop arm;
  double current_time = 0.0;
  arm.set_goal(55.0 / 180.0 * M_PI);

  // Open a file so we can log the data and make pretty graphs
  FILE *fd = fopen("/tmp/dump", "w");
  fprintf(fd, "time, position, velocity, acceleration, voltage\n");

  // run for one second
  while (current_time < 1.0) {
    const double voltage = arm.update(position_, lower_limit_triggered(), upper_limit_triggered(), true);
    simulate_for_time(voltage, ArmLoop::DT);
    current_time += ArmLoop::DT;

    // write this timestamps data into the file
    fprintf(fd, "%f, %f, %f, %f, %f\n", current_time, position_, velocity_, get_acceleration(voltage), voltage);
  }

  EXPECT_NEAR(position_, 55.0 / 180 * M_PI, 0.01);
}

// theta'' = C2V - C1theta'
//       0 = C2V - C1theta'
//  theta' = C2/C1 * V
//  constant velocity in graph = C2/C1
