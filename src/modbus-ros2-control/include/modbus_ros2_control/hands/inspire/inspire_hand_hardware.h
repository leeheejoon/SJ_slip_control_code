#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "modbus_ros2_control/communicator/modbus_rtu_communicator.h"
#include "modbus_ros2_control/hands/inspire/rh56e2_protocol.h"

namespace modbus_ros2_control
{

class InspireHandHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface::ConstSharedPtr>
  on_export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface::SharedPtr>
  on_export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) override;

  hardware_interface::return_type write(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) override;

private:
  static constexpr std::size_t kJointCount = 6;
  static constexpr std::size_t kActuatorCount = 6;

  static constexpr uint16_t kModeRegister = rh56e2_protocol::kModeRegister;
  static constexpr uint16_t kAngleSetRegister = rh56e2_protocol::kAngleSetRegister;
  static constexpr uint16_t kForceSetRegister = rh56e2_protocol::kForceSetRegister;
  static constexpr uint16_t kSpeedSetRegister = rh56e2_protocol::kSpeedSetRegister;
  static constexpr uint16_t kAngleActRegister = rh56e2_protocol::kAngleActRegister;
  static constexpr uint16_t kForceActRegister = rh56e2_protocol::kForceActRegister;

  void load_parameters();
  bool validate_joint_interfaces() const;
  bool connect_modbus();
  void disconnect_modbus();
  void start_background_thread();
  void stop_background_thread();
  void update_background_period(const rclcpp::Duration& period);
  void background_loop();
  void read_tactile_chunk();
  bool initialize_hand();
  bool read_feedback(std::array<double, kJointCount>& positions,
    std::array<double, kJointCount>& efforts);
  bool send_command(const std::array<uint16_t, kActuatorCount>& actuator_values);
  bool command_changed(const std::array<uint16_t, kActuatorCount>& actuator_values) const;

  std::array<uint16_t, kActuatorCount> joints_to_actuators(
    const std::array<double, kJointCount>& joints) const;
  std::array<double, kJointCount> actuators_to_joints(
    const std::array<uint16_t, kActuatorCount>& actuators) const;
  uint16_t joint_to_actuator_value(double radians, std::size_t joint_index) const;
  double actuator_value_to_joint(uint16_t value, std::size_t joint_index) const;

  static bool parse_bool(const std::string& value, bool default_value);
  static int parse_int(const std::string& value, int default_value);
  static double initial_value_for_joint(const hardware_interface::ComponentInfo& joint);
  static std::array<double, kJointCount> default_upper_limits(
    const std::vector<std::string>& joint_names);

  std::string serial_port_ = "/dev/ttyUSB0";
  std::string hand_side_ = "left";
  int baudrate_ = 115200;
  int slave_id_ = rh56e2_protocol::kDefaultSlaveId;
  bool read_feedback_enabled_ = true;
  // Opt-in: only enable for an RH56E2 T1 with the documented tactile map.
  bool read_tactile_enabled_ = false;
  int tactile_period_ms_ = 0;
  int tactile_feedback_period_ms_ = 100;
  std::vector<std::size_t> tactile_scan_regions_;
  std::size_t tactile_region_index_ = 0;
  std::size_t tactile_word_offset_ = 0;
  std::array<uint16_t, 112> tactile_values_{};
  std::array<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr, 17> tactile_publishers_{};
  std::chrono::steady_clock::time_point next_tactile_read_{};
  int background_period_ms_ = 20;
  std::atomic_bool background_period_initialized_{false};
  int command_deadband_raw_ = 1;
  // Application defaults within the manual's ranges, not factory defaults.
  uint16_t default_speed_ = 500;  // normalized 0..1000
  uint16_t default_force_ = 300;  // gram-force, allowed 0..3000

  std::unique_ptr<ModbusRtuCommunicator> modbus_;
  std::atomic_bool background_running_{false};
  std::thread background_thread_;
  std::mutex command_mutex_;
  std::mutex feedback_mutex_;

  std::vector<std::string> joint_names_;
  std::array<double, kJointCount> lower_limits_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, kJointCount> upper_limits_{1.1641, 0.5864, 1.4381, 1.4381, 1.4381, 1.4381};
  std::array<double, kJointCount> hw_positions_{};
  std::array<double, kJointCount> previous_positions_{};
  std::array<double, kJointCount> hw_velocities_{};
  std::array<double, kJointCount> hw_efforts_{};
  std::array<double, kJointCount> hw_commands_{};
  std::array<double, kJointCount> feedback_positions_{};
  std::array<double, kJointCount> feedback_efforts_{};

  std::array<uint16_t, kActuatorCount> last_command_values_{};
  std::array<uint16_t, kActuatorCount> pending_command_values_{};
  bool command_sent_ = false;
  bool pending_command_valid_ = false;
  bool pending_command_dirty_ = false;
  bool feedback_valid_ = false;
};

}  // namespace modbus_ros2_control
