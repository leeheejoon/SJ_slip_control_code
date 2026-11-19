#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "modbus_ros2_control/hands/inspire/rh56e2_protocol.h"

namespace protocol = modbus_ros2_control::rh56e2_protocol;

void require(bool condition)
{
  if (!condition) {
    throw std::runtime_error("RH56E2 manual protocol regression");
  }
}

int main()
{
  // Manual Table 41. Changing these back to the old map must fail this test.
  static_assert(protocol::kAngleSetRegister == 1486);
  static_assert(protocol::kForceSetRegister == 1498);
  static_assert(protocol::kSpeedSetRegister == 1522);
  static_assert(protocol::kAngleActRegister == 1546);
  static_assert(protocol::kForceActRegister == 1582);
  static_assert(protocol::kModeRegister == 1625);
  static_assert(protocol::kModeWordCount * sizeof(uint16_t) == 6);
  static_assert(protocol::kMaximumSpeed == 1000);
  static_assert(protocol::kMaximumForce == 3000);
  static_assert(protocol::kMaximumMeasuredForce == 4000);
  static_assert(protocol::kDefaultSlaveId == 1);

  // T1 Table 56 is a byte-addressed, contiguous 2124-byte map.
  // Check boundaries and word-count/address arithmetic to catch off-by-two reads.
  std::size_t tactile_points = 0;
  std::size_t tactile_chunks = 0;
  uint16_t next_address = 3000;
  for (const auto& region : protocol::kTactileRegions) {
    require(region.address == next_address);
    const std::size_t words = region.rows * region.columns;
    require(words > 0 && words <= 112);
    for (std::size_t offset = 0; offset < words; offset += protocol::kTactileChunkWords) {
      const auto count = std::min(protocol::kTactileChunkWords, words - offset);
      const auto start = region.address + 2 * offset;
      require(count <= 125);  // Modbus FC03 word limit.
      require(start + 2 * count <= region.address + 2 * words);
      ++tactile_chunks;
    }
    tactile_points += words;
    next_address = static_cast<uint16_t>(region.address + 2 * words);
  }
  require(next_address == 5124);
  require(tactile_points == 1062);
  require(tactile_chunks == 18);
  require(protocol::kTactileRegions[9].address == 4110);  // Index tip.
  require(protocol::kTactileRegions.back().rows == 8);
  require(protocol::kTactileRegions.back().columns == 14);

  // Distinct values expose swapped channels that equal six-value commands cannot detect.
  const std::array<uint16_t, 6> ros_values{100, 200, 300, 400, 500, 600};
  const std::array<uint16_t, 6> register_values{600, 500, 400, 300, 200, 100};
  require(protocol::reverse_joint_order(ros_values) == register_values);
  require(protocol::reverse_joint_order(register_values) == ros_values);
  for (std::size_t i = 0; i < ros_values.size(); ++i) {
    require(register_values[protocol::kJointToActuator[i]] == ros_values[i]);
  }

  const std::array<double, 6> upper{1.1641, 0.5864, 1.4381, 1.4381, 1.4381, 1.4381};
  for (double limit : upper) {
    require(protocol::angle_to_register(0.0, 0.0, limit) == 1000);
    require(protocol::angle_to_register(limit, 0.0, limit) == 0);
    require(protocol::angle_to_register(limit / 2, 0.0, limit) == 500);
    require(protocol::angle_to_register(-0.1, 0.0, limit) == 1000);
    require(protocol::angle_to_register(limit + 0.1, 0.0, limit) == 0);
    for (int raw = 0; raw <= 1000; ++raw) {
      const double angle = protocol::register_to_angle(raw, 0.0, limit);
      require(angle >= 0.0 && angle <= limit);
      require(protocol::angle_to_register(angle, 0.0, limit) == raw);
    }
    require(std::isnan(protocol::register_to_angle(1001, 0.0, limit)));
    require(std::isnan(protocol::register_to_angle(65535, 0.0, limit)));
  }
  require(protocol::force_from_register(0) == 0);
  require(protocol::force_from_register(300) == 300);
  require(protocol::force_from_register(4000) == 4000);
  require(protocol::force_from_register(65535) == -1);
  require(protocol::force_from_register(65536 - 4000) == -4000);
  std::cout << "PASS: RH56E2 V1.0.0 registers, six-byte modes, angle endpoints/round trips, signed force\n";
}
