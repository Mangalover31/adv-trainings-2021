#include "util.hh"
/* bazel test //src/util/... --test_output=errors
*/
bool check_range(float value, float lower, float upper) {
  if (value < lower) {
    return false;
  } else if (value > upper) {
    return false;
  }
  return true;
}

std::array<unsigned char, 4> encode_joystick(const Joystick &joystick) {
  std::array<unsigned char, 4> ret;

  // Place the header byte in the high byte
  ret[3] = 0xFE;

  if (!check_range(joystick.x, -1.0, 1.0) ||
      !check_range(joystick.y, -1.0, 1.0)) {
    throw std::runtime_error("Invalid joystick value");
  }

  // Add 1 to get value between 0 and 2, multiply by 127.5 to get between 0 and
  // 255
  ret[2] = static_cast<unsigned char>(std::floor(((joystick.x + 1) * 127.5)));
  ret[1] = static_cast<unsigned char>(std::floor(((joystick.y + 1) * 127.5)));
  ret[0] = 0;

  ret[0] |= (joystick.enabled << 7);//0001 << 2 = 0100
  ret[0] |= (joystick.gripper_toggle << 2);
  ret[0] |= (joystick.roller_fwd << 1);
  ret[0] |= (joystick.roller_rev << 0);
  return ret;
}

std::optional<Joystick>
decode_joystick(const std::array<unsigned char, 4> &raw) {
  if (raw[3] != 0xFE) {
    return std::nullopt;
  }

  Joystick ret;
  ret.x = ((static_cast<float>(raw[2]) - 127.5) / 127.5);
  ret.y = ((static_cast<float>(raw[1]) - 127.5) / 127.5);

  ret.enabled = raw[0] & (1 << 7);
  ret.gripper_toggle = raw[0] & (1 << 2);
  ret.roller_fwd = raw[0] & (1 << 1);
  ret.roller_rev = raw[0] & (1 << 0);

  return std::optional(ret);
}

std::array<unsigned char, 4> encode_output(const Output &raw) {
  std::array<unsigned char, 4> ret;
  ret[3] = static_cast<unsigned char>((raw.dt_left_voltage + 12) * 254 / 24);
  ret[2] = static_cast<unsigned char>((raw.dt_right_voltage + 12) * 254 / 24);
  ret[1] = static_cast<unsigned char>((raw.arm_voltage + 12) * 254 / 24);
  ret[0] = 0;
  ret[0] |= raw.gripper_open << 2;
  ret[0] |= raw.roller_forward << 1;
  ret[0] |= raw.roller_backwards;
  return ret;
}

std::optional<Output> decode_output(const std::array<unsigned char, 4> &raw) {
  if (raw[3] != 0xFE) { 
    return std::nullopt;
  }
  Output ret;
  ret.dt_left_voltage = ((raw[3] * 24) / 254) - 12;
  ret.dt_right_voltage = ((raw[2] * 24) / 254) - 12;
  ret.arm_voltage = ((raw[1] * 24) / 254) - 12;
  ret.gripper_open = raw[0] & (1 << 2);
  ret.roller_forward = raw[0] & (1 << 1);
  ret.roller_backwards = raw[0] & 1; 
  return std::optional(ret);
}

std::array<unsigned char, 4> encode_sensors(const Sensors &sensors) {
  std::array<unsigned char, 4> ret;
  ret[3] = 0xAF;
  ret[2] = ((sensors.arm_position / M_PI) * 4096);
  ret[1] = sensors.lower_limit_on;
  ret[0] = sensors.upper_limit_on;
  return ret;
}

std::optional<Sensors> decode_sensors(const std::array<unsigned char, 4> &raw) {
  if (raw[3] != 0xAF) { 
    return std::nullopt;
  }
  Sensors ret;
  ret.arm_position = (raw[2] * M_PI) / 4096;
  ret.lower_limit_on = 0;
  ret.upper_limit_on = 0;
  return std::optional(ret);
}