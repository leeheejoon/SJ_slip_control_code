#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace modbus_ros2_control::rh56e2_protocol
{
// RH56E2 User Manual V1.0.0, June 2025, Table 41 (printed pp.20-21).
// Use the documented start addresses, not the old actuator register map.
constexpr uint16_t kAngleSetRegister = 1486;
constexpr uint16_t kForceSetRegister = 1498;
constexpr uint16_t kSpeedSetRegister = 1522;
constexpr uint16_t kAngleActRegister = 1546;
constexpr uint16_t kForceActRegister = 1582;
constexpr uint16_t kModeRegister = 1625;

constexpr uint16_t kMaximumAngle = 1000;
constexpr uint16_t kMaximumSpeed = 1000;
constexpr uint16_t kMaximumForce = 3000;  // g (gram-force), not N or joint torque.
constexpr int kMaximumMeasuredForce = 4000;
// T1 resistive tactile map: manual Table 56. Addresses are byte addresses,
// while Modbus read counts are 16-bit words (same convention as FORCE_ACT).
struct TactileRegion
{
  const char* name;
  uint16_t address;
  uint16_t rows;
  uint16_t columns;
};
constexpr std::array<TactileRegion, 17> kTactileRegions{{
  {"little_tip", 3000, 3, 3},
  {"little_distal", 3018, 12, 8},
  {"little_pad", 3210, 10, 8},
  {"ring_tip", 3370, 3, 3},
  {"ring_distal", 3388, 12, 8},
  {"ring_pad", 3580, 10, 8},
  {"middle_tip", 3740, 3, 3},
  {"middle_distal", 3758, 12, 8},
  {"middle_pad", 3950, 10, 8},
  {"index_tip", 4110, 3, 3},
  {"index_distal", 4128, 12, 8},
  {"index_pad", 4320, 10, 8},
  {"thumb_tip", 4480, 3, 3},
  {"thumb_distal", 4498, 12, 8},
  {"thumb_middle", 4690, 3, 3},
  {"thumb_pad", 4708, 12, 8},
  {"palm", 4900, 8, 14},
}};
constexpr uint16_t kMaximumTactileValue = 4095;
constexpr std::size_t kTactileChunkWords = 96;

constexpr int kDefaultSlaveId = 1;  // Section 2.6.1: same factory ID for either side.

// Tables 45/46/47/49/50: little, ring, middle, index, thumb flexion, thumb rotation.
// ROS order: thumb rotation, thumb flexion, index, middle, ring, little.
constexpr std::array<std::size_t, 6> kJointToActuator{5, 4, 3, 2, 1, 0};

template<typename T>
inline std::array<T, 6> reverse_joint_order(const std::array<T, 6>& values)
{
  std::array<T, 6> result{};
  for (std::size_t i = 0; i < result.size(); ++i) {
    result[i] = values[kJointToActuator[i]];
  }
  return result;
}

// Table 55: six one-byte mode fields, i.e. six bytes = three Modbus words.
// All zero selects speed-force protection; do not write six 16-bit mode values.
constexpr int kModeWordCount = 3;

// Keep the existing ROS joint coordinate: lower=open, upper=closed.
// Manual sections 2.6.11-12: ANGLE=1000 is open, ANGLE=0 is closed.
// These are normalized angle commands, not POS_SET actuator stroke values.
inline uint16_t angle_to_register(double radians, double lower, double upper)
{
  if (!std::isfinite(radians) || upper <= lower) {
    return kMaximumAngle;
  }
  const double ratio = std::clamp((radians - lower) / (upper - lower), 0.0, 1.0);
  return static_cast<uint16_t>(std::lround((1.0 - ratio) * kMaximumAngle));
}

inline double register_to_angle(uint16_t value, double lower, double upper)
{
  if (value > kMaximumAngle) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (upper <= lower) {
    return lower;
  }
  return lower + (1.0 - static_cast<double>(value) / kMaximumAngle) * (upper - lower);
}

// Table 50: signed short, -4000..4000 g. Decode two's complement explicitly.
inline double force_from_register(uint16_t value)
{
  const int signed_value = value <= 32767 ? value : static_cast<int>(value) - 65536;
  return static_cast<double>(signed_value);
}
}  // namespace modbus_ros2_control::rh56e2_protocol
