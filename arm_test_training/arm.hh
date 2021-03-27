#include <cmath>

class ArmLoop {
public: 
  double update(double encoder_position, bool lower_limit, bool upper_limit, bool enabled);

  void set_goal(double goal) { goal_ = goal; }

  static constexpr double DT = 0.005;

  static constexpr double MAX_ANGLE = 170.0 * M_PI / 180.0;
  static constexpr double MIN_ANGLE = 30.0 * M_PI / 180.0;

private:
  double goal_ = 0.0;
  double last_error_ = 0.0;
};
